#!/usr/bin/env python3
"""hover_supervisor_node – persistent MAVROS hover supervisor for PX4/MAVROS.

Keeps a drone hovering at a latched position by continuously publishing to
``/{uav_name}/mavros/setpoint_position/local``.  Unlike ``mavros_takeoff_node``,
this node never exits: it remains alive after takeoff so OFFBOARD mode is
maintained even if the client node that triggered takeoff shuts down.

Services
--------
~/takeoff  (std_srvs/Trigger)
    Latch current XY from odom, climb to ``takeoff_altitude``, then hold.

~/latch_here  (std_srvs/Trigger)
    Latch current XYZ from odom immediately and hold at that position.

Topics subscribed
-----------------
~/set_altitude  (std_msgs/Float64)
    Override the Z setpoint while hovering (XY stays fixed).

Frame convention
----------------
All setpoints use ``frame_id = "map"`` (MAVROS local-origin ENU frame).
Altitude is positive-UP (ENU), consistent with
``/mavros/local_position/odom pose.position.z``.

State machine
-------------
IDLE → WARMUP → REQ_OFFBOARD → REQ_ARM → CONFIRM → TAKEOFF → HOVER

The node stays in HOVER indefinitely, continuously streaming setpoints.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# FSM states
# ---------------------------------------------------------------------------
_IDLE = 0          # waiting for ~/takeoff or ~/latch_here
_WARMUP = 1        # streaming setpoints before requesting OFFBOARD
_REQ_OFFBOARD = 2  # requesting OFFBOARD mode
_REQ_ARM = 3       # requesting ARM
_CONFIRM = 4       # waiting for OFFBOARD+ARM confirmation
_TAKEOFF = 5       # climbing to target altitude
_HOVER = 6         # hovering indefinitely

_STATE_NAMES = {
    _IDLE: 'IDLE',
    _WARMUP: 'WARMUP',
    _REQ_OFFBOARD: 'REQ_OFFBOARD',
    _REQ_ARM: 'REQ_ARM',
    _CONFIRM: 'CONFIRM',
    _TAKEOFF: 'TAKEOFF',
    _HOVER: 'HOVER',
}


class HoverSupervisorNode(Node):
    """Persistent hover supervisor node.

    Streams position setpoints continuously to maintain OFFBOARD mode.
    Provides services to trigger takeoff and latch hover position.
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(self):
        """Initialise publishers, subscribers, services, and timer."""
        super().__init__('hover_supervisor_node')

        # ---- parameters -----------------------------------------------
        self.declare_parameter('uav_name', 'uav1')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('altitude_threshold', 1.8)
        self.declare_parameter('warmup_streaming_time', 2.0)
        self.declare_parameter('timeout_confirm', 10.0)
        self.declare_parameter('timeout_takeoff', 20.0)
        self.declare_parameter('auto_offboard_arm', True)
        self.declare_parameter('z_step_per_tick', 0.0)  # 0 = unlimited

        uav = self.get_parameter('uav_name').get_parameter_value().string_value
        self._uav = uav
        self._frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self._rate_hz = rate_hz
        self._takeoff_alt = (
            self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        )
        self._alt_threshold = (
            self.get_parameter('altitude_threshold').get_parameter_value().double_value
        )
        self._warmup_time = (
            self.get_parameter('warmup_streaming_time').get_parameter_value().double_value
        )
        self._timeout_confirm = (
            self.get_parameter('timeout_confirm').get_parameter_value().double_value
        )
        self._timeout_takeoff = (
            self.get_parameter('timeout_takeoff').get_parameter_value().double_value
        )
        self._auto_offboard_arm = (
            self.get_parameter('auto_offboard_arm').get_parameter_value().bool_value
        )
        self._z_step = (
            self.get_parameter('z_step_per_tick').get_parameter_value().double_value
        )

        # ---- FSM state ---------------------------------------------------
        self._fsm = _IDLE
        self._state_entry_time = self.get_clock().now()

        # ---- sensor data -------------------------------------------------
        self._mavros_state = State()
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_z: float = 0.0
        self._odom_received: bool = False

        # ---- service call bookkeeping ------------------------------------
        _cooldown = self._timeout_confirm + 1.0
        self._last_offboard_req = self.get_clock().now() - Duration(
            seconds=_cooldown
        )
        self._last_arm_req = self.get_clock().now() - Duration(
            seconds=_cooldown
        )

        # ---- status log throttle -----------------------------------------
        self._last_status_log: float = 0.0
        self._last_takeoff_log: float = 0.0

        # ---- setpoint (always publishing) --------------------------------
        self._setpoint = PoseStamped()
        self._setpoint.header.frame_id = self._frame_id
        self._setpoint.pose.orientation.w = 1.0  # identity quaternion (yaw=0)
        self._setpoint.pose.position.x = 0.0
        self._setpoint.pose.position.y = 0.0
        self._setpoint.pose.position.z = 0.0

        # Target Z (may be rate-limited toward _setpoint.pose.position.z)
        self._target_z: float = 0.0

        # ---- publishers --------------------------------------------------
        self._setpoint_pub = self.create_publisher(
            PoseStamped, f'/{uav}/mavros/setpoint_position/local', 10
        )

        # ---- subscribers -------------------------------------------------
        self._state_sub = self.create_subscription(
            State, f'/{uav}/mavros/state', self._state_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, f'/{uav}/mavros/local_position/odom', self._odom_cb, 10
        )
        self._alt_sub = self.create_subscription(
            Float64, '~/set_altitude', self._set_altitude_cb, 10
        )

        # ---- service clients ---------------------------------------------
        self._mode_client = self.create_client(SetMode, f'/{uav}/mavros/set_mode')
        self._arm_client = self.create_client(
            CommandBool, f'/{uav}/mavros/cmd/arming'
        )

        # ---- services offered --------------------------------------------
        self._takeoff_srv = self.create_service(
            Trigger, '~/takeoff', self._handle_takeoff
        )
        self._latch_srv = self.create_service(
            Trigger, '~/latch_here', self._handle_latch_here
        )

        # ---- main timer --------------------------------------------------
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'hover_supervisor_node started | uav={uav} '
            f'takeoff_alt={self._takeoff_alt}m '
            f'frame={self._frame_id} rate={rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: State) -> None:
        self._mavros_state = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_z = msg.pose.pose.position.z
        if not self._odom_received:
            self._odom_received = True
            # Initialise setpoint to current position on first odom
            self._setpoint.pose.position.x = self._odom_x
            self._setpoint.pose.position.y = self._odom_y
            self._setpoint.pose.position.z = self._odom_z
            self._target_z = self._odom_z
            self.get_logger().info(
                f'First odom received: x={self._odom_x:.3f} '
                f'y={self._odom_y:.3f} z={self._odom_z:.3f}'
            )

    def _set_altitude_cb(self, msg: Float64) -> None:
        """Override Z setpoint while hovering (XY stays fixed)."""
        self._target_z = msg.data
        self.get_logger().info(f'set_altitude: target_z -> {msg.data:.3f}m')

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_takeoff(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle ~/takeoff: latch XY from odom and climb to takeoff_altitude."""
        if self._fsm not in (_IDLE, _HOVER):
            state_name = _STATE_NAMES.get(self._fsm, str(self._fsm))
            response.success = False
            response.message = (
                f'Already in state {state_name}; ignoring takeoff request'
            )
            self.get_logger().warn(response.message)
            return response

        if not self._odom_received:
            response.success = False
            response.message = 'Odom not yet received; cannot latch takeoff position'
            self.get_logger().warn(response.message)
            return response

        # Latch XY from current odom; keep Z at ground level during warmup
        self._setpoint.pose.position.x = self._odom_x
        self._setpoint.pose.position.y = self._odom_y
        self._setpoint.pose.position.z = self._odom_z
        self._target_z = self._takeoff_alt

        self.get_logger().info(
            f'Takeoff requested: XY=({self._odom_x:.3f},{self._odom_y:.3f}) '
            f'target_z={self._takeoff_alt:.2f}m'
        )

        already_active = (
            self._mavros_state.armed and self._mavros_state.mode == 'OFFBOARD'
        )
        if self._auto_offboard_arm and not already_active:
            self._transition(_WARMUP)
        else:
            # Already OFFBOARD+ARMED or auto disabled: jump straight to TAKEOFF
            self._target_z = self._takeoff_alt
            self._transition(_TAKEOFF)

        response.success = True
        response.message = 'Takeoff initiated'
        return response

    def _handle_latch_here(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle ~/latch_here: latch current XYZ from odom and hover."""
        if not self._odom_received:
            response.success = False
            response.message = 'Odom not yet received; cannot latch position'
            self.get_logger().warn(response.message)
            return response

        self._setpoint.pose.position.x = self._odom_x
        self._setpoint.pose.position.y = self._odom_y
        self._setpoint.pose.position.z = self._odom_z
        self._target_z = self._odom_z
        self._transition(_HOVER)

        response.success = True
        response.message = (
            f'Latched at x={self._odom_x:.3f} y={self._odom_y:.3f} '
            f'z={self._odom_z:.3f}'
        )
        self.get_logger().info(f'latch_here: {response.message}')
        return response

    # ------------------------------------------------------------------
    # Main timer tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        # Apply rate-limited altitude step if z_step_per_tick > 0
        if self._z_step > 0.0:
            current_z = self._setpoint.pose.position.z
            diff = self._target_z - current_z
            if abs(diff) > self._z_step:
                self._setpoint.pose.position.z = (
                    current_z + (self._z_step if diff > 0 else -self._z_step)
                )
            else:
                self._setpoint.pose.position.z = self._target_z
        else:
            self._setpoint.pose.position.z = self._target_z

        # Always publish setpoint to keep OFFBOARD mode alive
        self._publish_setpoint()

        # FSM dispatch
        if self._fsm == _IDLE:
            self._do_idle()
        elif self._fsm == _WARMUP:
            self._do_warmup()
        elif self._fsm == _REQ_OFFBOARD:
            self._do_req_offboard()
        elif self._fsm == _REQ_ARM:
            self._do_req_arm()
        elif self._fsm == _CONFIRM:
            self._do_confirm()
        elif self._fsm == _TAKEOFF:
            self._do_takeoff()
        elif self._fsm == _HOVER:
            self._do_hover()

    # ------------------------------------------------------------------
    # FSM state handlers
    # ------------------------------------------------------------------

    def _do_idle(self) -> None:
        self._log_status()

    def _do_warmup(self) -> None:
        elapsed = self._elapsed_in_state()
        if elapsed >= self._warmup_time:
            if not self._odom_received:
                self.get_logger().warn(
                    'Odom not yet received during warmup; waiting...'
                )
                return
            self.get_logger().info(
                f'Warmup complete ({elapsed:.1f}s) – requesting OFFBOARD'
            )
            self._transition(_REQ_OFFBOARD)

    def _do_req_offboard(self) -> None:
        now = self.get_clock().now()
        since_last = (now - self._last_offboard_req).nanoseconds * 1e-9
        if since_last < 1.0:
            return
        if not self._mode_client.service_is_ready():
            self.get_logger().warn('set_mode service not ready')
            return
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self._mode_client.call_async(req)
        future.add_done_callback(self._offboard_resp_cb)
        self._last_offboard_req = now
        self.get_logger().info('OFFBOARD mode requested')
        self._transition(_REQ_ARM)

    def _do_req_arm(self) -> None:
        now = self.get_clock().now()
        since_last = (now - self._last_arm_req).nanoseconds * 1e-9
        if since_last < 1.0:
            return
        if not self._arm_client.service_is_ready():
            self.get_logger().warn('arming service not ready')
            return
        req = CommandBool.Request()
        req.value = True
        future = self._arm_client.call_async(req)
        future.add_done_callback(self._arm_resp_cb)
        self._last_arm_req = now
        self.get_logger().info('ARM requested')
        self._transition(_CONFIRM)

    def _do_confirm(self) -> None:
        armed = self._mavros_state.armed
        mode = self._mavros_state.mode
        elapsed = self._elapsed_in_state()

        if armed and mode == 'OFFBOARD':
            self.get_logger().info(
                f'OFFBOARD+ARM confirmed (armed={armed} mode={mode}) – climbing'
            )
            self._setpoint.pose.position.z = self._takeoff_alt
            self._target_z = self._takeoff_alt
            self._transition(_TAKEOFF)
            return

        if elapsed > self._timeout_confirm:
            self.get_logger().warn(
                f'Timeout waiting for OFFBOARD+ARM ({elapsed:.1f}s) – retrying'
            )
            self._transition(_REQ_OFFBOARD)

    def _do_takeoff(self) -> None:
        elapsed = self._elapsed_in_state()
        alt = self._odom_z

        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._last_takeoff_log >= 1.0:
            self._last_takeoff_log = now_s
            self.get_logger().info(
                f'TAKEOFF: z_enu={alt:.2f}m target={self._takeoff_alt:.2f}m '
                f't={elapsed:.1f}s'
            )

        if alt >= self._alt_threshold:
            self.get_logger().info(
                f'Altitude reached: z={alt:.2f}m >= {self._alt_threshold:.2f}m'
                ' – HOVER'
            )
            self._transition(_HOVER)
            return

        if elapsed > self._timeout_takeoff:
            self.get_logger().error(
                f'Takeoff timeout ({elapsed:.1f}s) without reaching '
                f'{self._alt_threshold:.2f}m – resetting timer, keep trying'
            )
            # Reset timer by re-entering TAKEOFF to avoid blocking indefinitely
            self._state_entry_time = self.get_clock().now()

    def _do_hover(self) -> None:
        self._log_status()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_setpoint(self) -> None:
        self._setpoint.header.stamp = self.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._setpoint)

    def _transition(self, new_state: int) -> None:
        old_name = _STATE_NAMES.get(self._fsm, str(self._fsm))
        new_name = _STATE_NAMES.get(new_state, str(new_state))
        self.get_logger().info(f'FSM: {old_name} -> {new_name}')
        self._fsm = new_state
        self._state_entry_time = self.get_clock().now()

    def _elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self._state_entry_time).nanoseconds * 1e-9

    def _log_status(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._last_status_log < 5.0:
            return
        self._last_status_log = now_s
        state_name = _STATE_NAMES.get(self._fsm, str(self._fsm))
        self.get_logger().info(
            f'STATUS | FSM={state_name} '
            f'pos=({self._odom_x:.2f},{self._odom_y:.2f},{self._odom_z:.2f}) '
            f'setpoint_z={self._setpoint.pose.position.z:.2f} '
            f'armed={self._mavros_state.armed} mode={self._mavros_state.mode}'
        )

    # ------------------------------------------------------------------
    # Service response callbacks
    # ------------------------------------------------------------------

    def _offboard_resp_cb(self, future) -> None:
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info('OFFBOARD mode request accepted by FCU')
            else:
                self.get_logger().warn('OFFBOARD mode request rejected by FCU')
        except Exception as exc:
            self.get_logger().error(f'set_mode call failed: {exc}')

    def _arm_resp_cb(self, future) -> None:
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('ARM request accepted by FCU')
            else:
                self.get_logger().warn('ARM request rejected by FCU')
        except Exception as exc:
            self.get_logger().error(f'arming call failed: {exc}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """ROS 2 entry point."""
    rclpy.init(args=args)
    node = HoverSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
