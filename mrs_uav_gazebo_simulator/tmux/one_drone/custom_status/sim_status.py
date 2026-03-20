#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import curses
import math
import psutil      # <--- nova biblioteca para CPU e RAM
import os

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

class SimStatus(Node):
    def __init__(self, stdscr):
        super().__init__('sim_status')
        self.stdscr = stdscr
        self.pose = None
        self.vel = None
        self.state = None
        self.batt = None

        # Pegamos o PID do processo atual para monitorar CPU/RAM
        self.process = psutil.Process(os.getpid())

        self.create_subscription(PoseStamped,   '/uav1/mavros/local_position/pose',           self.pose_cb, 10)
        self.create_subscription(TwistStamped,  '/uav1/mavros/local_position/velocity_local', self.vel_cb,  10)
        self.create_subscription(State,         '/uav1/mavros/state',                         self.state_cb,10)
        self.create_subscription(BatteryState,  '/uav1/mavros/battery',                       self.batt_cb, 10)

        self.timer = self.create_timer(0.2, self.draw)

    def pose_cb(self, msg):   self.pose = msg
    def vel_cb(self, msg):    self.vel = msg
    def state_cb(self, msg):  self.state = msg
    def batt_cb(self, msg):   self.batt = msg

    def quat_to_euler(self, q):
        sinr = 2 * (q.w * q.x + q.y * q.z)
        cosr = 1 - 2 * (q.x*q.x + q.y*q.y)
        roll = math.degrees(math.atan2(sinr, cosr))

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(sinp)) if abs(sinp) <= 1 else math.degrees(math.copysign(math.pi/2, sinp))

        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        yaw = math.degrees(math.atan2(siny, cosy))
        return roll, pitch, yaw

    def draw(self):
        self.stdscr.erase()
        h, w = self.stdscr.getmaxyx()

        title = " SIM STATUS - UAV1 "
        self.stdscr.addstr(0, (w - len(title)) // 2, title, curses.A_BOLD | curses.color_pair(1))

        # Posição e Atitude
        if self.pose:
            p = self.pose.pose.position
            self.stdscr.addstr(2, 2, f"Posição     X: {p.x:7.2f}   Y: {p.y:7.2f}   Z: {p.z:7.2f} m")
            r, pi, y = self.quat_to_euler(self.pose.pose.orientation)
            self.stdscr.addstr(3, 2, f"Atitude    Roll: {r:6.1f}°  Pitch: {pi:6.1f}°  Yaw: {y:6.1f}°")
        else:
            self.stdscr.addstr(2, 2, "Posição     --- (aguardando pose)")

        # Velocidade
        if self.vel:
            v = self.vel.twist.linear
            speed_h = math.sqrt(v.x**2 + v.y**2)
            speed_total = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            self.stdscr.addstr(5, 2, f"Velocidade  VX: {v.x:6.2f}  VY: {v.y:6.2f}  VZ: {v.z:6.2f} m/s")
            self.stdscr.addstr(6, 2, f"            Horizontal: {speed_h:5.2f} m/s   Total: {speed_total:5.2f} m/s")
        else:
            self.stdscr.addstr(5, 2, "Velocidade  --- (aguardando velocity)")

        # FCU Status
        if self.state:
            armed = "ARMED  " if self.state.armed else "DISARMED"
            mode = self.state.mode if self.state.mode else "UNKNOWN"
            color = curses.color_pair(2) if self.state.armed else curses.color_pair(3)
            self.stdscr.addstr(8, 2, f"FCU Status  {armed}   Mode: {mode}", color)
        else:
            self.stdscr.addstr(8, 2, "FCU Status  --- (aguardando mavros/state)")

        # Bateria
        if self.batt:
            voltage = self.batt.voltage if self.batt.voltage > 0 else 0.0
            percentage = int(self.batt.percentage * 100) if self.batt.percentage >= 0 else -1
            perc_str = f"{percentage:3d}%" if percentage >= 0 else "??%"
            self.stdscr.addstr(10, 2, f"Bateria     {voltage:5.2f} V   {perc_str}")
        else:
            self.stdscr.addstr(10, 2, "Bateria     --- (aguardando mavros/battery)")

        # Novo: CPU e RAM do processo
        cpu_percent = self.process.cpu_percent(interval=None)  # None = último valor conhecido
        mem_info = self.process.memory_info()
        ram_mb = mem_info.rss / (1024 * 1024)  # Resident Set Size em MB

        self.stdscr.addstr(12, 2, f"Processo    CPU: {cpu_percent:5.1f}%   RAM: {ram_mb:6.1f} MB", curses.A_DIM)

        # Tempo de simulação
        self.stdscr.addstr(h-2, 2, f"Sim time: {self.get_clock().now().to_msg().sec} s", curses.A_DIM)

        self.stdscr.refresh()

def main(stdscr):
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN,  curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_RED,   curses.COLOR_BLACK)
    stdscr.nodelay(True)
    stdscr.clear()

    rclpy.init()
    node = SimStatus(stdscr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        curses.endwin()

if __name__ == '__main__':
    curses.wrapper(main)