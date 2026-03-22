#!/usr/bin/env python3
"""
enhanced_sim_status.py
Enhanced version of sim_status.py – maintains full backwards compatibility
while adding new metrics and improved layout.

New additions over sim_status.py:
  - Euler angles (roll/pitch/yaw)
  - Total speed alongside horizontal speed
  - Better FCU state colouring (OFFBOARD detection)
  - Battery current display
  - CPU / RAM of *this* monitor process
  - Sim-time display
  - Improved section headers

Refresh rate: 5 Hz (200 ms timer, same as original)
"""

import rclpy
from rclpy.node import Node
import curses
import math
import os
import psutil
import signal

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState


class EnhancedSimStatus(Node):
    def __init__(self, stdscr):
        super().__init__("enhanced_sim_status")
        self.stdscr = stdscr
        self.pose: PoseStamped | None = None
        self.vel: TwistStamped | None = None
        self.state: State | None = None
        self.batt: BatteryState | None = None

        # Monitor its own process for CPU / RAM (same behaviour as original)
        self.process = psutil.Process(os.getpid())

        self.create_subscription(
            PoseStamped,
            "/uav1/mavros/local_position/pose",
            self._pose_cb, 10)
        self.create_subscription(
            TwistStamped,
            "/uav1/mavros/local_position/velocity_local",
            self._vel_cb, 10)
        self.create_subscription(
            State,
            "/uav1/mavros/state",
            self._state_cb, 10)
        self.create_subscription(
            BatteryState,
            "/uav1/mavros/battery",
            self._batt_cb, 10)

        # 5 Hz refresh (same as original sim_status.py)
        self.create_timer(0.2, self._draw)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped) -> None:   self.pose = msg
    def _vel_cb(self, msg: TwistStamped) -> None:   self.vel = msg
    def _state_cb(self, msg: State) -> None:        self.state = msg
    def _batt_cb(self, msg: BatteryState) -> None:  self.batt = msg

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _quat_to_euler(q) -> tuple[float, float, float]:
        sinr = 2 * (q.w * q.x + q.y * q.z)
        cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr, cosr))

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = (math.degrees(math.asin(sinp))
                 if abs(sinp) <= 1
                 else math.degrees(math.copysign(math.pi / 2, sinp)))

        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny, cosy))
        return roll, pitch, yaw

    # ── Draw ──────────────────────────────────────────────────────────────────

    def _draw(self) -> None:
        stdscr = self.stdscr
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        C_TITLE   = curses.color_pair(1) | curses.A_BOLD
        C_OK      = curses.color_pair(2)
        C_WARN    = curses.color_pair(3)
        C_DIM     = curses.A_DIM
        C_BOLD    = curses.A_BOLD

        def put(row: int, col: int, text: str, attr: int = 0) -> None:
            if row < h and col < w:
                try:
                    stdscr.addstr(row, col, text[:w - col], attr)
                except curses.error:
                    pass

        title = " SIM STATUS MELHORADO - UAV1 "
        put(0, (w - len(title)) // 2, title, C_TITLE)

        row = 2
        # ── Posição e Atitude ──
        put(row, 0, "── POSIÇÃO E ATITUDE ─────────────────────────", C_BOLD)
        row += 1
        if self.pose:
            p = self.pose.pose.position
            put(row, 2, f"Posição   X: {p.x:7.2f}   Y: {p.y:7.2f}   Z: {p.z:7.2f} m")
            row += 1
            r, pi, y = self._quat_to_euler(self.pose.pose.orientation)
            put(row, 2, f"Atitude  Roll: {r:6.1f}°  Pitch: {pi:6.1f}°  Yaw: {y:6.1f}°")
            row += 1
            alt = abs(p.z)
            alt_lbl = "🟢 EM VOO" if alt > 0.3 else "🔵 NO SOLO"
            put(row, 2, f"Altitude  {alt:.2f} m  {alt_lbl}")
        else:
            put(row, 2, "Posição   --- (aguardando pose)")
            row += 1
            put(row, 2, "")
            row += 1
            put(row, 2, "")
        row += 2

        # ── Velocidade ──
        put(row, 0, "── VELOCIDADE ────────────────────────────────", C_BOLD)
        row += 1
        if self.vel:
            v = self.vel.twist.linear
            speed_h = math.sqrt(v.x**2 + v.y**2)
            speed_t = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            put(row, 2, f"Linear  VX: {v.x:6.2f}  VY: {v.y:6.2f}  VZ: {v.z:6.2f} m/s")
            row += 1
            put(row, 2, f"Speed   Horizontal: {speed_h:5.2f} m/s   Total: {speed_t:5.2f} m/s")
        else:
            put(row, 2, "Linear  --- (aguardando velocity)")
            row += 1
            put(row, 2, "")
        row += 2

        # ── FCU Status ──
        put(row, 0, "── FCU STATUS ────────────────────────────────", C_BOLD)
        row += 1
        if self.state:
            armed_str = "ARMED   " if self.state.armed else "DISARMED"
            mode = self.state.mode or "UNKNOWN"
            offboard = mode == "OFFBOARD"
            color = (C_OK if (self.state.armed and offboard)
                     else C_WARN if self.state.armed
                     else curses.A_NORMAL)
            put(row, 2, f"Estado  {armed_str}   Mode: {mode}", color)
        else:
            put(row, 2, "Estado  --- (aguardando mavros/state)", C_WARN)
        row += 2

        # ── Bateria ──
        put(row, 0, "── BATERIA ───────────────────────────────────", C_BOLD)
        row += 1
        if self.batt:
            voltage = self.batt.voltage if self.batt.voltage > 0 else 0.0
            pct = self.batt.percentage
            pct_str = f"{pct*100:5.1f}%" if pct >= 0 else "  ??%"
            curr = getattr(self.batt, "current", 0.0)
            b_color = C_WARN if (pct >= 0 and pct < 0.20) else curses.A_NORMAL
            put(row, 2, f"Voltage: {voltage:5.2f} V   Percentual: {pct_str}   Corrente: {curr:.2f} A", b_color)
        else:
            put(row, 2, "--- (aguardando mavros/battery)")
        row += 2

        # ── CPU / RAM ──
        put(row, 0, "── PROCESSO (MONITOR) ────────────────────────", C_BOLD)
        row += 1
        cpu_pct = self.process.cpu_percent(interval=None)
        mem_info = self.process.memory_info()
        ram_mb = mem_info.rss / (1024 * 1024)
        put(row, 2,
            f"CPU: {cpu_pct:5.1f}%   RAM: {ram_mb:6.1f} MB", C_DIM)
        row += 2

        # ── Footer ──
        sim_sec = self.get_clock().now().to_msg().sec
        put(h - 2, 2, f"Sim time: {sim_sec} s", C_DIM)

        stdscr.refresh()


# ── Entry point ───────────────────────────────────────────────────────────────

def _run(stdscr: "curses.window") -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN,  -1)
    curses.init_pair(2, curses.COLOR_GREEN, -1)
    curses.init_pair(3, curses.COLOR_RED,   -1)
    stdscr.nodelay(True)
    curses.curs_set(0)

    rclpy.init()
    node = EnhancedSimStatus(stdscr)

    def _sigint(_sig, _frame):
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
