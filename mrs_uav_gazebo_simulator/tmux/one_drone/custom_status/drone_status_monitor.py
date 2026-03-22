#!/usr/bin/env python3
"""
drone_status_monitor.py
Professional real-time status monitor for the drone controller.

Displays:
  - System-wide CPU and RAM (all cores, total memory)
  - drone_node process CPU and RAM (per-process)
  - Disk usage for /tmp/drone_logs
  - Real altitude vs. target altitude with visual bar
  - Altitude margin indicator (target ± margin)
  - OFFBOARD + ARM status (prominent, colour-coded)
  - Flight state label (desarmado / armado / decolagem / voo / pouso)
  - Linear velocity (VX, VY, VZ) and total 3-D speed
  - Horizontal distance from origin (XY)
  - Battery voltage / current / percentage with bar
  - GPS satellite count and fix type (if sensor_msgs/NavSatFix available)
  - Loop frequency, cycle count and uptime
  - Up to 5 prioritised alerts

Refresh rate: 4 Hz (250 ms timer)
"""

import rclpy
from rclpy.node import Node
import curses
import math
import os
import time
import signal
import psutil

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, String


# ── Helpers ───────────────────────────────────────────────────────────────────

def ascii_bar(value: float, max_value: float, width: int = 20,
              warn_frac: float = 0.75, crit_frac: float = 0.90,
              reverse: bool = False) -> tuple[str, bool, bool]:
    """
    Return (bar_str, is_warn, is_crit).

    Default (reverse=False): high fraction is bad → CPU/RAM style.
      is_warn when value/max >= warn_frac.

    reverse=True: low fraction is bad → battery style.
      is_warn when (1 - value/max) >= warn_frac, i.e. value/max is low.
    """
    if max_value <= 0:
        return "[" + "?" * width + "]", False, False
    fraction = max(0.0, min(1.0, value / max_value))
    filled = int(fraction * width)
    empty = width - filled
    bar = "[" + "█" * filled + "░" * empty + "]"
    if reverse:
        # Low value = bad (battery: warn when nearly empty)
        is_warn = (1.0 - fraction) >= warn_frac
        is_crit = (1.0 - fraction) >= crit_frac
    else:
        # High value = bad (CPU/RAM: warn when nearly full)
        is_warn = fraction >= warn_frac
        is_crit = fraction >= crit_frac
    return bar, is_warn, is_crit


def fmt_uptime(seconds: float) -> str:
    """Format seconds as HH:MM:SS."""
    s = int(seconds)
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{sec:02d}"


def fmt_bytes(n_bytes: int) -> str:
    """Format bytes as human-readable string."""
    for unit in ("B", "KB", "MB", "GB"):
        if n_bytes < 1024:
            return f"{n_bytes:.1f}{unit}"
        n_bytes /= 1024  # type: ignore[assignment]
    return f"{n_bytes:.1f}TB"


# sensor_msgs/msg/NavSatStatus constants
GPS_FIX_NAMES = {-1: "SEM FIX", 0: "FIX BÁSICO", 1: "SBAS FIX", 2: "GBAS FIX"}


class DroneStatusMonitor(Node):
    def __init__(self, stdscr: "curses.window") -> None:
        super().__init__("drone_status_monitor")
        self.stdscr = stdscr
        self.start_time = time.monotonic()
        self.cycle_count = 0
        self.loop_hz = 0.0
        self._last_draw_time = time.monotonic()

        # Subscribed data
        self.pose: PoseStamped | None = None
        self.vel: TwistStamped | None = None
        self.state: State | None = None
        self.batt: BatteryState | None = None
        self.gps: NavSatFix | None = None
        self.target_altitude: float | None = None
        self.flight_state_str: str | None = None  # state label from drone_node

        # Timestamps for staleness detection
        self._last_pose_t: float = 0.0
        self._last_state_t: float = 0.0

        # drone_node process (optional – best-effort)
        self._drone_proc: psutil.Process | None = self._find_drone_proc()

        # Alerts list (strings, max 5 shown)
        self.alerts: list[str] = []

        # ROS subscriptions
        self.create_subscription(
            PoseStamped, "/uav1/mavros/local_position/pose", self._pose_cb, 10)
        self.create_subscription(
            TwistStamped, "/uav1/mavros/local_position/velocity_local", self._vel_cb, 10)
        self.create_subscription(
            State, "/uav1/mavros/state", self._state_cb, 10)
        self.create_subscription(
            BatteryState, "/uav1/mavros/battery", self._batt_cb, 10)
        self.create_subscription(
            NavSatFix, "/uav1/mavros/global_position/raw/fix", self._gps_cb, 10)
        # Optional: target altitude published by drone_node
        self.create_subscription(
            Float64, "/drone/target_altitude", self._target_alt_cb, 10)
        # Optional: flight state string published by drone_node
        self.create_subscription(
            String, "/drone/flight_state", self._flight_state_cb, 10)

        # 4 Hz refresh
        self.create_timer(0.25, self._draw)

        self.get_logger().info("DroneStatusMonitor iniciado (4 Hz)")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg
        self._last_pose_t = time.monotonic()

    def _vel_cb(self, msg: TwistStamped) -> None:
        self.vel = msg

    def _state_cb(self, msg: State) -> None:
        self.state = msg
        self._last_state_t = time.monotonic()

    def _batt_cb(self, msg: BatteryState) -> None:
        self.batt = msg

    def _gps_cb(self, msg: NavSatFix) -> None:
        self.gps = msg

    def _target_alt_cb(self, msg: Float64) -> None:
        self.target_altitude = msg.data

    def _flight_state_cb(self, msg: String) -> None:
        self.flight_state_str = msg.data

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _find_drone_proc() -> "psutil.Process | None":
        """Best-effort: find the drone_node process for per-process monitoring."""
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                cmdline = " ".join(proc.info.get("cmdline") or [])
                if "drone_node" in cmdline and "drone_status_monitor" not in cmdline:
                    return psutil.Process(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return None

    def _get_system_resources(self) -> dict:
        """Return system-wide and drone_node-specific resource usage."""
        result: dict = {}

        # System CPU (non-blocking; first call returns 0 which is acceptable)
        result["sys_cpu"] = psutil.cpu_percent(interval=None)
        result["sys_cpu_count"] = psutil.cpu_count(logical=True) or 1

        # System RAM
        vm = psutil.virtual_memory()
        result["sys_ram_used_mb"] = vm.used / (1024 * 1024)
        result["sys_ram_total_mb"] = vm.total / (1024 * 1024)
        result["sys_ram_pct"] = vm.percent

        # Disk usage for drone logs (/tmp/drone_logs or /tmp if not present)
        log_path = "/tmp/drone_logs" if os.path.isdir("/tmp/drone_logs") else "/tmp"
        try:
            du = psutil.disk_usage(log_path)
            result["disk_used_mb"] = du.used / (1024 * 1024)
            result["disk_total_mb"] = du.total / (1024 * 1024)
            result["disk_pct"] = du.percent
            result["disk_path"] = log_path
        except Exception:
            result["disk_used_mb"] = 0.0
            result["disk_total_mb"] = 1.0
            result["disk_pct"] = 0.0
            result["disk_path"] = log_path

        # drone_node process stats
        if self._drone_proc is None:
            self._drone_proc = self._find_drone_proc()
        proc = self._drone_proc
        if proc is not None:
            try:
                result["proc_cpu"] = proc.cpu_percent(interval=None)
                mem = proc.memory_info()
                result["proc_ram_mb"] = mem.rss / (1024 * 1024)
                result["proc_ram_pct"] = proc.memory_percent()
                result["proc_pid"] = proc.pid
                result["proc_found"] = True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                self._drone_proc = None
                result["proc_found"] = False
        else:
            result["proc_found"] = False

        return result

    def _infer_flight_state(self) -> str:
        """Infer a human-readable flight state when drone_node doesn't publish one."""
        if self.flight_state_str:
            return self.flight_state_str
        if not self.state:
            return "SEM DADOS"
        if not self.state.armed:
            return "DESARMADO"
        alt = abs(self.pose.pose.position.z) if self.pose else 0.0
        if alt < 0.3:
            return "ARMADO / SOLO"
        if self.target_altitude is not None:
            tgt = self.target_altitude
            margin = 0.3
            if abs(alt - tgt) <= margin:
                return "HOVER / ALVO"
            if alt < tgt - margin:
                return "DECOLANDO"
            if alt > tgt + margin:
                return "POUSANDO"
        return "EM VOO"

    def _build_alerts(self, res: dict) -> list[str]:
        alerts: list[str] = []
        now = time.monotonic()

        # FCU communication loss
        if not self.state or (now - self._last_state_t) > 3.0:
            alerts.append("🔴 Sem dados do FCU (mavros/state) há >3s")

        # Pose staleness
        if self._last_pose_t > 0 and (now - self._last_pose_t) > 3.0:
            alerts.append("🔴 Sem pose há >3s – verificar odometria")

        # OFFBOARD+ARM while in flight but not confirmed
        if self.state and self.pose:
            alt = abs(self.pose.pose.position.z)
            in_flight = alt > 0.5
            offboard_arm = self.state.mode == "OFFBOARD" and self.state.armed
            if in_flight and not offboard_arm:
                alerts.append(f"🟡 EM VOO sem OFFBOARD+ARM! (modo={self.state.mode})")

        # System CPU
        sys_cpu = res.get("sys_cpu", 0.0)
        if sys_cpu > 90.0:
            alerts.append(f"🔴 CPU SISTEMA: {sys_cpu:.1f}% (crítico)")
        elif sys_cpu > 75.0:
            alerts.append(f"🟡 CPU SISTEMA: {sys_cpu:.1f}% (elevado)")

        # System RAM
        sys_ram_pct = res.get("sys_ram_pct", 0.0)
        if sys_ram_pct > 85.0:
            alerts.append(f"🔴 RAM SISTEMA: {sys_ram_pct:.1f}%")

        # Battery
        if self.batt:
            pct = self.batt.percentage
            if 0.0 <= pct < 0.10:
                alerts.append(f"🔴 BATERIA CRÍTICA: {pct*100:.1f}%")
            elif 0.0 <= pct < 0.25:
                alerts.append(f"🟡 BATERIA BAIXA: {pct*100:.1f}%")

        # Disk nearly full
        disk_pct = res.get("disk_pct", 0.0)
        if disk_pct > 90.0:
            alerts.append(f"🔴 DISCO CHEIO: {disk_pct:.0f}% ({res.get('disk_path')})")

        return alerts[:5]

    # ── Draw ──────────────────────────────────────────────────────────────────

    def _draw(self) -> None:
        now = time.monotonic()
        dt = now - self._last_draw_time
        self._last_draw_time = now
        self.cycle_count += 1
        if dt > 0:
            self.loop_hz = 1.0 / dt

        res = self._get_system_resources()
        self.alerts = self._build_alerts(res)

        stdscr = self.stdscr
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        W = min(w - 1, 90)   # working width (wider for more info)

        def put(row: int, col: int, text: str, attr: int = 0) -> None:
            if row < h and col >= 0 and col < w:
                try:
                    stdscr.addstr(row, col, text[: w - col], attr)
                except curses.error:
                    pass

        C_TITLE   = curses.color_pair(1) | curses.A_BOLD
        C_OK      = curses.color_pair(2) | curses.A_BOLD
        C_WARN    = curses.color_pair(3) | curses.A_BOLD
        C_CRIT    = curses.color_pair(5) | curses.A_BOLD
        C_SECTION = curses.color_pair(4) | curses.A_BOLD
        C_DIM     = curses.A_DIM
        C_BOLD    = curses.A_BOLD
        C_NORMAL  = curses.A_NORMAL

        border = "═" * (W - 2)
        row = 0

        # ── Header ──────────────────────────────────────────────────────────
        put(row, 0, f"╔{border}╗", C_TITLE); row += 1
        now_str = time.strftime("%H:%M:%S")
        title = f"🚁 DRONE STATUS MONITOR  [{now_str}]  4 Hz"
        padding = W - 2 - len(title)
        put(row, 0, f"║ {title}{' ' * max(0, padding)}║", C_TITLE); row += 1
        put(row, 0, f"╠{border}╣", C_TITLE); row += 1

        # ── OFFBOARD + ARM banner (most critical info first) ─────────────────
        put(row, 0, "║ 🔐 MODO DE VOO", C_SECTION); row += 1

        if self.state:
            offboard = self.state.mode == "OFFBOARD"
            armed = self.state.armed
            offboard_arm = offboard and armed

            oa_label = "✅ OFFBOARD+ARM  ATIVO" if offboard_arm else "❌ OFFBOARD+ARM  INATIVO"
            oa_color = C_OK if offboard_arm else C_WARN
            mode_label = f"Modo FCU: {self.state.mode:<12}"
            arm_label = "Armado: ✅ SIM" if armed else "Armado: ❌ NÃO"
            arm_color = C_OK if armed else C_DIM

            flight_label = self._infer_flight_state()

            put(row, 0, f"║   {oa_label:<30}", oa_color)
            put(row, 34, f" {mode_label}", C_BOLD)
            put(row, 34 + 1 + len(mode_label), f"  {arm_label}", arm_color)
            row += 1
            put(row, 0, f"║   Estado de Voo: {flight_label}", C_BOLD); row += 1
        else:
            put(row, 0, "║   ❌ FCU NÃO CONECTADO – aguardando mavros/state", C_CRIT); row += 1
            put(row, 0, "║", C_NORMAL); row += 1

        put(row, 0, f"║{' ' * (W - 2)}║"); row += 1

        # ── Altitude ─────────────────────────────────────────────────────────
        put(row, 0, "║ 🏔️  ALTITUDE", C_SECTION); row += 1

        if self.pose:
            alt = self.pose.pose.position.z  # Z positivo = acima do solo (ENU)
            alt_abs = abs(alt)
            tgt = self.target_altitude

            # Altitude bar (0–max_display)
            max_display = max(10.0, (tgt or 0.0) * 1.5, alt_abs * 1.5)
            alt_bar, _, _ = ascii_bar(alt_abs, max_display, width=22)
            alt_status = "🟢 EM VOO" if alt_abs > 0.3 else "🔵 NO SOLO"

            put(row, 0, f"║   Real : {alt_abs:6.2f}m  {alt_bar} {alt_status}")
            row += 1

            if tgt is not None:
                diff = alt_abs - tgt
                diff_str = f"{diff:+.2f}m"
                diff_color = C_OK if abs(diff) < 0.3 else C_WARN
                tgt_bar, _, _ = ascii_bar(tgt, max_display, width=22)
                put(row, 0, f"║   Alvo : {tgt:6.2f}m  {tgt_bar}")
                put(row, 0 + 9 + 6 + 2 + 22 + 1, f"  Δ={diff_str}", diff_color)
            else:
                put(row, 0, "║   Alvo :   ---")
            row += 1
        else:
            put(row, 0, "║   --- (aguardando pose)"); row += 1
            put(row, 0, "║"); row += 1

        put(row, 0, f"║{' ' * (W - 2)}║"); row += 1

        # ── Posição e Velocidade ──────────────────────────────────────────────
        put(row, 0, "║ 🗺️  POSIÇÃO E VELOCIDADE", C_SECTION); row += 1

        if self.pose:
            p = self.pose.pose.position
            dist_xy = math.sqrt(p.x ** 2 + p.y ** 2)
            put(row, 0,
                f"║   X:{p.x:8.2f}m  Y:{p.y:8.2f}m  Z:{p.z:8.2f}m  "
                f"DistXY:{dist_xy:6.2f}m"); row += 1

            if self.vel:
                v = self.vel.twist.linear
                vxy = math.sqrt(v.x ** 2 + v.y ** 2)
                vxyz = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
                put(row, 0,
                    f"║   VX:{v.x:7.2f}m/s  VY:{v.y:7.2f}m/s  VZ:{v.z:7.2f}m/s  "
                    f"Speed:{vxyz:5.2f}m/s  VXY:{vxy:5.2f}m/s")
            else:
                put(row, 0, "║   VX: ---  VY: ---  VZ: ---")
            row += 1
        else:
            put(row, 0, "║   --- (aguardando pose)"); row += 1
            put(row, 0, "║"); row += 1

        put(row, 0, f"║{' ' * (W - 2)}║"); row += 1

        # ── GPS ──────────────────────────────────────────────────────────────
        put(row, 0, "║ 📡 GPS", C_SECTION); row += 1

        if self.gps:
            fix_name = GPS_FIX_NAMES.get(self.gps.status.status, "DESCONHECIDO")
            fix_ok = self.gps.status.status >= 0
            fix_color = C_OK if fix_ok else C_WARN
            put(row, 0,
                f"║   Lat:{self.gps.latitude:11.6f}°  Lon:{self.gps.longitude:11.6f}°  "
                f"AltGPS:{self.gps.altitude:7.2f}m")
            row += 1
            put(row, 0, f"║   Fix: {fix_name}", fix_color)
            row += 1
        else:
            put(row, 0, "║   --- (aguardando mavros/global_position/raw/fix)"); row += 1
            put(row, 0, "║"); row += 1

        put(row, 0, f"║{' ' * (W - 2)}║"); row += 1

        # ── Bateria ──────────────────────────────────────────────────────────
        put(row, 0, "║ 🔋 BATERIA", C_SECTION); row += 1

        if self.batt:
            pct = max(0.0, self.batt.percentage if self.batt.percentage >= 0 else 0.0)
            volt = self.batt.voltage if self.batt.voltage > 0 else 0.0
            curr = self.batt.current if hasattr(self.batt, "current") else 0.0
            b_bar, b_warn, b_crit = ascii_bar(pct * 100, 100.0, width=22, reverse=True)
            b_color = C_CRIT if b_crit else (C_WARN if b_warn else C_OK)
            put(row, 0, f"║   {b_bar}")
            put(row, 4 + len(b_bar), f" {pct*100:5.1f}%", b_color)
            put(row, 4 + len(b_bar) + 9,
                f"  Volt:{volt:5.2f}V  Curr:{curr:5.2f}A")
        else:
            put(row, 0, "║   --- (aguardando mavros/battery)")
        row += 1

        put(row, 0, f"║{' ' * (W - 2)}║"); row += 1

        # ── Sistema (CPU / RAM / Disco) ───────────────────────────────────────
        put(row, 0, "║ 💻 SISTEMA", C_SECTION); row += 1

        sys_cpu = res.get("sys_cpu", 0.0)
        sys_cpu_n = res.get("sys_cpu_count", 1)
        sys_ram_mb = res.get("sys_ram_used_mb", 0.0)
        sys_ram_total = res.get("sys_ram_total_mb", 1.0)
        sys_ram_pct = res.get("sys_ram_pct", 0.0)
        disk_pct = res.get("disk_pct", 0.0)
        disk_used_mb = res.get("disk_used_mb", 0.0)
        disk_total_mb = res.get("disk_total_mb", 1.0)

        cpu_bar, cpu_warn, cpu_crit = ascii_bar(sys_cpu, 100.0, width=16)
        cpu_color = C_CRIT if cpu_crit else (C_WARN if cpu_warn else C_OK)

        ram_bar, ram_warn, ram_crit = ascii_bar(sys_ram_pct, 100.0, width=16)
        ram_color = C_CRIT if ram_crit else (C_WARN if ram_warn else C_NORMAL)

        disk_bar, disk_warn, disk_crit = ascii_bar(disk_pct, 100.0, width=14)
        disk_color = C_CRIT if disk_crit else (C_WARN if disk_warn else C_NORMAL)

        put(row, 0, f"║   CPU {sys_cpu_n}cores: {cpu_bar}")
        put(row, 22 + len(cpu_bar), f" {sys_cpu:5.1f}%", cpu_color)
        put(row, 22 + len(cpu_bar) + 8,
            f"  RAM: {ram_bar}")
        put(row, 22 + len(cpu_bar) + 8 + 7 + len(ram_bar),
            f" {sys_ram_pct:4.1f}%  ({sys_ram_mb:.0f}/{sys_ram_total:.0f}MB)", ram_color)
        row += 1

        put(row, 0,
            f"║   Disco ({res.get('disk_path','/tmp')}): {disk_bar}")
        put(row, 17 + len(res.get('disk_path', '/tmp')) + len(disk_bar),
            f" {disk_pct:.1f}%  ({disk_used_mb:.0f}/{disk_total_mb:.0f}MB)", disk_color)
        row += 1

        # drone_node process stats
        if res.get("proc_found"):
            proc_cpu = res.get("proc_cpu", 0.0)
            proc_ram_mb = res.get("proc_ram_mb", 0.0)
            proc_ram_pct = res.get("proc_ram_pct", 0.0)
            proc_pid = res.get("proc_pid", 0)
            p_cpu_bar, p_cpu_warn, _ = ascii_bar(proc_cpu, 100.0, width=14)
            p_cpu_color = C_WARN if p_cpu_warn else C_NORMAL
            put(row, 0,
                f"║   drone_node (PID {proc_pid}): CPU {p_cpu_bar}")
            put(row, 35 + len(str(proc_pid)) + len(p_cpu_bar),
                f" {proc_cpu:5.1f}%  RAM:{proc_ram_mb:.1f}MB ({proc_ram_pct:.1f}%)",
                p_cpu_color)
        else:
            put(row, 0, "║   drone_node: não encontrado (aguardando startup...)", C_DIM)
        row += 1

        put(row, 0, f"║{' ' * (W - 2)}║"); row += 1

        # ── Monitor stats ─────────────────────────────────────────────────────
        uptime = fmt_uptime(time.monotonic() - self.start_time)
        put(row, 0,
            f"║   Monitor → Loop:{self.loop_hz:5.1f}Hz  Ciclos:{self.cycle_count:6d}  "
            f"Uptime:{uptime}")
        row += 1

        put(row, 0, f"║{' ' * (W - 2)}║"); row += 1

        # ── Alertas ───────────────────────────────────────────────────────────
        put(row, 0, "║ ⚠️  ALERTAS", C_SECTION); row += 1

        if self.alerts:
            for alert in self.alerts[:5]:
                put(row, 0, f"║   {alert}", C_WARN); row += 1
        else:
            put(row, 0, "║   ✅ Sem alertas ativos", C_OK); row += 1

        # ── Footer ────────────────────────────────────────────────────────────
        if row < h:
            put(row, 0, f"╚{border}╝", C_TITLE)

        stdscr.refresh()


# ── Entry point ───────────────────────────────────────────────────────────────

def _run(stdscr: "curses.window") -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN,    -1)   # title / header
    curses.init_pair(2, curses.COLOR_GREEN,   -1)   # ok / good
    curses.init_pair(3, curses.COLOR_YELLOW,  -1)   # warn / alert
    curses.init_pair(4, curses.COLOR_CYAN,    -1)   # section headers
    curses.init_pair(5, curses.COLOR_RED,     -1)   # critical
    stdscr.nodelay(True)
    curses.curs_set(0)

    rclpy.init()
    node = DroneStatusMonitor(stdscr)

    def _sigint(_sig, _frame) -> None:  # graceful Ctrl+C
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        curses.endwin()


def main() -> None:
    curses.wrapper(_run)


if __name__ == "__main__":
    main()
