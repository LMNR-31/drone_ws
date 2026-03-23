#!/usr/bin/env python3
"""
drone_status_monitor.py
Professional real-time status monitor for the drone controller.

Displays:
  - CPU and RAM usage (ASCII bar charts)
  - Real altitude vs. target altitude
  - OFFBOARD + ARM state
  - Linear velocity and total speed
  - Battery voltage / current / percentage
  - Loop frequency and uptime
  - Up to 3 critical alerts

Refresh rate: 2 Hz (500 ms timer)
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
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64


# ── Helpers ───────────────────────────────────────────────────────────────────

def ascii_bar(value: float, max_value: float, width: int = 28) -> str:
    """Return a filled ASCII bar like [████░░░░░░░░] for the given value/max."""
    if max_value <= 0:
        return "[" + "?" * width + "]"
    fraction = max(0.0, min(1.0, value / max_value))
    filled = int(fraction * width)
    empty = width - filled
    return "[" + "█" * filled + "░" * empty + "]"


def fmt_uptime(seconds: float) -> str:
    """Format seconds as HH:MM:SS."""
    s = int(seconds)
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{sec:02d}"


# ── Constants ─────────────────────────────────────────────────────────────────

RAM_MAX_MB: float = 512.0   # Upper bound used for the RAM ASCII bar chart


class DroneStatusMonitor(Node):
    def __init__(self, stdscr):
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
        self.target_altitude: float | None = None

        # drone_node process (optional – best-effort)
        self._drone_proc: psutil.Process | None = self._find_drone_proc()

        # Alerts list (strings, max 3 shown)
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
        # Optional: target altitude published by drone_node
        self.create_subscription(
            Float64, "/drone/target_altitude", self._target_alt_cb, 10)

        # 2 Hz refresh
        self.create_timer(0.5, self._draw)

        self.get_logger().info("DroneStatusMonitor iniciado (2 Hz)")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg

    def _vel_cb(self, msg: TwistStamped) -> None:
        self.vel = msg

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _batt_cb(self, msg: BatteryState) -> None:
        self.batt = msg

    def _target_alt_cb(self, msg: Float64) -> None:
        self.target_altitude = msg.data

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _find_drone_proc() -> "psutil.Process | None":
        """Best-effort: find the drone_node process for CPU/RAM monitoring."""
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                cmdline = " ".join(proc.info.get("cmdline") or [])
                if "drone_node" in cmdline or "drone_controller" in cmdline:
                    return psutil.Process(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return None

    def _get_cpu_ram(self) -> tuple[float, float, float]:
        """Return (cpu_pct, ram_mb, ram_pct). Falls back to monitor's own process."""
        if self._drone_proc is None:
            self._drone_proc = self._find_drone_proc()
        proc = self._drone_proc or psutil.Process(os.getpid())
        try:
            cpu = proc.cpu_percent(interval=None)
            mem = proc.memory_info()
            ram_mb = mem.rss / (1024 * 1024)
            ram_pct = proc.memory_percent()
            return cpu, ram_mb, ram_pct
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            self._drone_proc = None
            return 0.0, 0.0, 0.0

    def _build_alerts(self, cpu: float, ram_pct: float) -> list[str]:
        alerts: list[str] = []

        if cpu > 80.0:
            alerts.append(f"🔴 CPU ALTO: {cpu:.1f}% (limite 80%)")
        if ram_pct > 70.0:
            alerts.append(f"🔴 RAM ALTA: {ram_pct:.1f}% (limite 70%)")

        if self.batt:
            pct = self.batt.percentage
            if 0.0 <= pct < 0.20:
                alerts.append(f"🟡 BATERIA BAIXA: {pct*100:.1f}%")

        if self.state:
            if self.pose:
                alt = abs(self.pose.pose.position.z)
                in_flight = alt > 0.3
            else:
                in_flight = False

            offboard_arm = self.state.mode == "OFFBOARD" and self.state.armed
            if in_flight and not offboard_arm:
                alerts.append("🟡 OFFBOARD+ARM não confirmado em voo!")
        else:
            alerts.append("🔴 Sem comunicação com FCU (mavros/state)")

        return alerts[:3]

    # ── Draw ──────────────────────────────────────────────────────────────────

    def _draw(self) -> None:
        now = time.monotonic()
        dt = now - self._last_draw_time
        self._last_draw_time = now
        self.cycle_count += 1
        if dt > 0:
            self.loop_hz = 1.0 / dt

        cpu, ram_mb, ram_pct = self._get_cpu_ram()
        self.alerts = self._build_alerts(cpu, ram_pct)

        stdscr = self.stdscr
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        W = min(w - 1, 80)  # working width

        def put(row: int, col: int, text: str, attr: int = 0) -> None:
            if row < h and col < w:
                try:
                    stdscr.addstr(row, col, text[:w - col], attr)
                except curses.error:
                    pass

        C_TITLE = curses.color_pair(1) | curses.A_BOLD
        C_OK = curses.color_pair(2)
        C_WARN = curses.color_pair(3)
        C_DIM = curses.A_DIM
        C_BOLD = curses.A_BOLD
        C_SECTION = curses.color_pair(4) | curses.A_BOLD

        border = "═" * (W - 2)
        row = 0

        # ── Header ──
        put(row, 0, f"╔{border}╗", C_TITLE)
        row += 1
        title = "🚁 DRONE CONTROLLER STATUS MONITOR (2 Hz Refresh)"
        padding = W - 2 - len(title)
        put(row, 0, f"║ {title}{' ' * max(0, padding)}║", C_TITLE)
        row += 1
        put(row, 0, f"╠{border}╣", C_TITLE)
        row += 1

        # ── Estado ──
        put(row, 0, "║ 📊 ESTADO", C_SECTION)
        row += 1

        if self.state:
            offboard_arm = self.state.mode == "OFFBOARD" and self.state.armed
            oa_str = "✅ SIM" if offboard_arm else "❌ NÃO"
            oa_color = C_OK if offboard_arm else C_WARN

            if self.pose and abs(self.pose.pose.position.z) > 0.3:
                estado_str = "✈️  TRAJETÓRIA/VOO"
            elif self.state.armed:
                estado_str = "⚙️  ARMADO NO SOLO"
            else:
                estado_str = "🛑 DESARMADO"

            line = f"   Estado: {estado_str:<24} | OFFBOARD+ARM: "
            put(row, 0, f"║{line}", C_BOLD)
            put(row, 1 + len(line), oa_str, oa_color)
        else:
            put(row, 0, "║   Estado: --- (aguardando mavros/state)", C_WARN)
        row += 1

        put(row, 0, f"║{' ' * (W - 2)}║")
        row += 1

        # ── Posição e Velocidade ──
        put(row, 0, "║ 🗺️  POSIÇÃO E VELOCIDADE", C_SECTION)
        row += 1

        if self.pose:
            p = self.pose.pose.position
            # NOTA: MAVROS publica em NED, mas MRS converte internamente para ENU
            # Z positivo = altitude (UP)
            put(row, 0,
                f"║   X: {p.x:7.2f}m  Y: {p.y:7.2f}m  Z: {p.z:7.2f}m (NED - MRS Internal)")
            row += 1
            # Altitude já é Z positivo em ENU
            # Z já é altura positiva
            v = self.vel.twist.linear if self.vel else None
            if v:
                speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                put(row, 0,
                    f"║   VX:{v.x:6.2f}m/s  VY:{v.y:6.2f}m/s  VZ:{v.z:6.2f}m/s  "
                    f"Speed:{speed:5.2f}m/s")
            else:
                put(row, 0, "║   VX: ---  VY: ---  VZ: ---")
            row += 1

            alt = abs(p.z)
            alt_status = "🟢 EM VOO" if alt > 0.3 else "🔵 NO SOLO"
            tgt_str = f"{self.target_altitude:.2f}m" if self.target_altitude is not None else "  ---  "
            put(row, 0,
                f"║   Altitude Real:{alt:6.2f}m {alt_status:<14}  Target: {tgt_str}")
        else:
            put(row, 0, "║   --- (aguardando pose)")
            row += 1
            put(row, 0, "║")
            row += 1
            put(row, 0, "║")
        row += 1

        put(row, 0, f"║{' ' * (W - 2)}║")
        row += 1

        # ── Controlador (CPU / RAM / Loop) ──
        put(row, 0, "║ 🎛️  CONTROLADOR", C_SECTION)
        row += 1

        cpu_bar = ascii_bar(cpu, 100.0)
        cpu_color = C_WARN if cpu > 80 else C_OK
        put(row, 0, f"║   CPU: {cpu_bar}")
        put(row, 9 + len(cpu_bar), f" {cpu:5.1f}%", cpu_color | C_BOLD)

        ram_bar = ascii_bar(ram_mb, RAM_MAX_MB)
        ram_color = C_WARN if ram_pct > 70 else curses.A_NORMAL
        ram_info = f"RAM:{ram_mb:6.1f}MB ({ram_pct:4.1f}%)"
        put(row, 9 + len(cpu_bar) + 8, f" | {ram_info}", ram_color)
        row += 1

        uptime = fmt_uptime(time.monotonic() - self.start_time)
        put(row, 0,
            f"║   Loop:{self.loop_hz:6.1f} Hz | Ciclos:{self.cycle_count:8d} | Uptime: {uptime}")
        row += 1

        put(row, 0, f"║{' ' * (W - 2)}║")
        row += 1

        # ── Bateria ──
        put(row, 0, "║ 🔋 BATERIA", C_SECTION)
        row += 1

        if self.batt:
            pct = self.batt.percentage if self.batt.percentage >= 0 else 0.0
            volt = self.batt.voltage if self.batt.voltage > 0 else 0.0
            curr = self.batt.current if hasattr(self.batt, "current") else 0.0
            b_bar = ascii_bar(pct * 100, 100.0)
            b_color = C_WARN if pct < 0.20 else C_OK
            put(row, 0, f"║   {b_bar}")
            put(row, 4 + len(b_bar), f" {pct*100:5.1f}%", b_color | C_BOLD)
            put(row, 4 + len(b_bar) + 8,
                f" | Voltage:{volt:5.2f}V | Current:{curr:5.2f}A")
        else:
            put(row, 0, "║   --- (aguardando mavros/battery)")
        row += 1

        put(row, 0, f"║{' ' * (W - 2)}║")
        row += 1

        # ── Alertas ──
        put(row, 0, "║ ⚠️  ALERTAS", C_SECTION)
        row += 1

        if self.alerts:
            for alert in self.alerts[:3]:
                put(row, 0, f"║   {alert}", C_WARN | C_BOLD)
                row += 1
        else:
            put(row, 0, "║   ✅ Sem alertas ativos", C_OK)
            row += 1

        # ── Footer ──
        if row < h:
            put(row, 0, f"╚{border}╝", C_TITLE)

        stdscr.refresh()


# ── Entry point ───────────────────────────────────────────────────────────────

def _run(stdscr: "curses.window") -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN,    -1)   # title
    curses.init_pair(2, curses.COLOR_GREEN,   -1)   # ok
    curses.init_pair(3, curses.COLOR_RED,     -1)   # warn / alert
    curses.init_pair(4, curses.COLOR_YELLOW,  -1)   # section headers
    stdscr.nodelay(True)
    curses.curs_set(0)

    rclpy.init()
    node = DroneStatusMonitor(stdscr)

    def _sigint(_sig, _frame):  # graceful Ctrl+C
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
