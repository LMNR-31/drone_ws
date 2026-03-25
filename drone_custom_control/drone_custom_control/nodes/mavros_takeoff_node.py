#!/usr/bin/env python3
"""mavros_takeoff_node – robust position-setpoint takeoff for PX4/MAVROS.

Uses /mavros/setpoint_position/local (PX4 internal position controller) to
get the drone stably airborne before handing off to an attitude-based
controller such as lpv_mpc_drone_node.

Frame convention
----------------
All setpoints are published with ``frame_id = "map"`` (the default MAVROS
local-origin frame).  Altitude is positive-UP (ENU), consistent with
``/mavros/local_position/odom`` ``pose.position.z``.

State machine
-------------
WAIT_FCU → STREAM_SETPOINT → REQUEST_OFFBOARD → REQUEST_ARM →
CONFIRM_ACTIVATION → TAKEOFF → HOLD → SUCCESS | FAIL
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool


# ---------------------------------------------------------------------------
# State indices
# ---------------------------------------------------------------------------
_WAIT_FCU = 0
_STREAM_SETPOINT = 1
_REQUEST_OFFBOARD = 2
_REQUEST_ARM = 3
_CONFIRM_ACTIVATION = 4
_TAKEOFF = 5
_HOLD = 6
_SUCCESS = 7
_FAIL = 8

_STATE_NAMES = {
    _WAIT_FCU: 'WAIT_FCU',
    _STREAM_SETPOINT: 'STREAM_SETPOINT',
    _REQUEST_OFFBOARD: 'REQUEST_OFFBOARD',
    _REQUEST_ARM: 'REQUEST_ARM',
    _CONFIRM_ACTIVATION: 'CONFIRM_ACTIVATION',
    _TAKEOFF: 'TAKEOFF',
    _HOLD: 'HOLD',
    _SUCCESS: 'SUCCESS',
    _FAIL: 'FAIL',
}


class MavrosTakeoffNode(Node):
    """Position-setpoint takeoff helper node."""

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(self):
        """Initialise publishers, subscribers, service clients, and timer."""
        super().__init__('mavros_takeoff_node')

        # ---- parameters -----------------------------------------------
        self.declare_parameter('uav_name', 'uav1')
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('altitude_threshold', 0.5)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('streaming_time', 3.0)
        self.declare_parameter('timeout_offboard', 10.0)
        self.declare_parameter('timeout_takeoff', 20.0)
        self.declare_parameter('hold_time', 2.0)
        self.declare_parameter('use_current_xy', True)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_waypoint_goal_on_success', False)

        uav = self.get_parameter('uav_name').get_parameter_value().string_value
        self._takeoff_alt = (
            self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        )
        self._alt_threshold = (
            self.get_parameter('altitude_threshold').get_parameter_value().double_value
        )
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self._streaming_time = (
            self.get_parameter('streaming_time').get_parameter_value().double_value
        )
        self._timeout_offboard = (
            self.get_parameter('timeout_offboard').get_parameter_value().double_value
        )
        self._timeout_takeoff = (
            self.get_parameter('timeout_takeoff').get_parameter_value().double_value
        )
        self._hold_time = (
            self.get_parameter('hold_time').get_parameter_value().double_value
        )
        self._use_current_xy = (
            self.get_parameter('use_current_xy').get_parameter_value().bool_value
        )
        self._param_x = self.get_parameter('x').get_parameter_value().double_value
        self._param_y = self.get_parameter('y').get_parameter_value().double_value
        self._frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )
        self._pub_wp_goal = (
            self.get_parameter('publish_waypoint_goal_on_success')
            .get_parameter_value()
            .bool_value
        )

        # ---- state -------------------------------------------------------
        self._fsm = _WAIT_FCU
        self._state_entry_time = self.get_clock().now()

        self._mavros_state = State()
        self._odom_z: float = 0.0
        self._odom_received: bool = False
        self._takeoff_x: float = self._param_x
        self._takeoff_y: float = self._param_y
        self._xy_latched: bool = False

        # service call bookkeeping
        self._offboard_future = None
        self._arm_future = None
        self._last_offboard_request = self.get_clock().now() - Duration(
            seconds=self._timeout_offboard + 1.0
        )
        self._last_arm_request = self.get_clock().now() - Duration(
            seconds=self._timeout_offboard + 1.0
        )

        # hold phase
        self._hold_start: rclpy.time.Time | None = None
        self._rate_hz = rate_hz

        # ---- setpoint ----------------------------------------------------
        self._setpoint = PoseStamped()
        self._setpoint.header.frame_id = self._frame_id
        self._setpoint.pose.orientation.w = 1.0  # identity quaternion (yaw=0)

        # ---- pubs / subs / clients --------------------------------------
        self._setpoint_pub = self.create_publisher(
            PoseStamped, f'/{uav}/mavros/setpoint_position/local', 10
        )
        self._waypoint_goal_pub = self.create_publisher(
            PoseStamped, '/waypoint_goal', 10
        )

        self._state_sub = self.create_subscription(
            State, f'/{uav}/mavros/state', self._state_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, f'/{uav}/mavros/local_position/odom', self._odom_cb, 10
        )

        self._mode_client = self.create_client(SetMode, f'/{uav}/mavros/set_mode')
        self._arm_client = self.create_client(
            CommandBool, f'/{uav}/mavros/cmd/arming'
        )

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'mavros_takeoff_node started | uav={uav} '
            f'alt={self._takeoff_alt}m frame={self._frame_id}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: State) -> None:
        self._mavros_state = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_z = msg.pose.pose.position.z
        if not self._odom_received:
            self._odom_received = True

        # Latch XY on first message when use_current_xy is enabled
        if self._use_current_xy and not self._xy_latched:
            self._takeoff_x = msg.pose.pose.position.x
            self._takeoff_y = msg.pose.pose.position.y
            self._xy_latched = True
            self.get_logger().info(
                f'XY latched from odom: x={self._takeoff_x:.3f} y={self._takeoff_y:.3f}'
            )

    # ------------------------------------------------------------------
    # Main timer tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        # Always publish setpoint (required to maintain OFFBOARD mode)
        self._publish_setpoint()

        if self._fsm == _WAIT_FCU:
            self._do_wait_fcu()
        elif self._fsm == _STREAM_SETPOINT:
            self._do_stream_setpoint()
        elif self._fsm == _REQUEST_OFFBOARD:
            self._do_request_offboard()
        elif self._fsm == _REQUEST_ARM:
            self._do_request_arm()
        elif self._fsm == _CONFIRM_ACTIVATION:
            self._do_confirm_activation()
        elif self._fsm == _TAKEOFF:
            self._do_takeoff()
        elif self._fsm == _HOLD:
            self._do_hold()
        elif self._fsm in (_SUCCESS, _FAIL):
            # Terminal states – node remains alive so setpoints keep streaming
            pass

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _do_wait_fcu(self) -> None:
        if self._mavros_state.connected:
            self.get_logger().info('FCU connected – streaming setpoints')
            self._transition(_STREAM_SETPOINT)

    def _do_stream_setpoint(self) -> None:
        elapsed = self._elapsed_in_state()
        if elapsed >= self._streaming_time:
            if not self._odom_received:
                self.get_logger().warn(
                    'Odom not yet received; waiting before requesting OFFBOARD'
                )
                return
            self.get_logger().info(
                f'Setpoints streamed for {elapsed:.1f}s – requesting OFFBOARD'
            )
            self._transition(_REQUEST_OFFBOARD)

    def _do_request_offboard(self) -> None:
        now = self.get_clock().now()
        since_last = (now - self._last_offboard_request).nanoseconds * 1e-9
        if since_last < 1.0:
            return
        if not self._mode_client.service_is_ready():
            self.get_logger().warn('set_mode service not ready')
            return
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self._offboard_future = self._mode_client.call_async(req)
        self._offboard_future.add_done_callback(self._offboard_response_cb)
        self._last_offboard_request = now
        self.get_logger().info('OFFBOARD mode requested')
        self._transition(_REQUEST_ARM)

    def _do_request_arm(self) -> None:
        now = self.get_clock().now()
        since_last = (now - self._last_arm_request).nanoseconds * 1e-9
        if since_last < 1.0:
            return
        if not self._arm_client.service_is_ready():
            self.get_logger().warn('arming service not ready')
            return
        req = CommandBool.Request()
        req.value = True
        self._arm_future = self._arm_client.call_async(req)
        self._arm_future.add_done_callback(self._arm_response_cb)
        self._last_arm_request = now
        self.get_logger().info('ARM requested')
        self._transition(_CONFIRM_ACTIVATION)

    def _do_confirm_activation(self) -> None:
        armed = self._mavros_state.armed
        mode = self._mavros_state.mode
        elapsed = self._elapsed_in_state()

        if armed and mode == 'OFFBOARD':
            self.get_logger().info(
                f'OFFBOARD+ARM confirmed (armed={armed} mode={mode}) – starting climb'
            )
            # Bake in the takeoff setpoint before entering TAKEOFF
            self._setpoint.pose.position.x = self._takeoff_x
            self._setpoint.pose.position.y = self._takeoff_y
            self._setpoint.pose.position.z = self._takeoff_alt
            self._transition(_TAKEOFF)
            return

        if elapsed > self._timeout_offboard:
            self.get_logger().warn(
                f'Timeout waiting for OFFBOARD+ARM ({elapsed:.1f}s) – retrying'
            )
            self._transition(_REQUEST_OFFBOARD)

    def _do_takeoff(self) -> None:
        elapsed = self._elapsed_in_state()
        alt = self._odom_z

        if elapsed % 1.0 < (1.0 / self._rate_hz):  # throttled log ~1 Hz
            self.get_logger().info(
                f'TAKEOFF: z_enu={alt:.2f}m target={self._takeoff_alt:.2f}m '
                f't={elapsed:.1f}s'
            )

        if alt >= self._alt_threshold:
            self.get_logger().info(
                f'Altitude threshold reached: z={alt:.2f}m >= {self._alt_threshold:.2f}m'
            )
            self._hold_start = self.get_clock().now()
            self._transition(_HOLD)
            return

        if elapsed > self._timeout_takeoff:
            self.get_logger().error(
                f'Takeoff timeout ({elapsed:.1f}s) without reaching '
                f'{self._alt_threshold:.2f}m – FAIL'
            )
            self._transition(_FAIL)

    def _do_hold(self) -> None:
        if self._hold_start is None:
            self._hold_start = self.get_clock().now()

        held = (self.get_clock().now() - self._hold_start).nanoseconds * 1e-9
        if held >= self._hold_time:
            self.get_logger().info(
                f'Hold complete ({held:.1f}s) at z={self._odom_z:.2f}m – SUCCESS'
            )
            if self._pub_wp_goal:
                self._publish_waypoint_goal()
            self._transition(_SUCCESS)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_setpoint(self) -> None:
        self._setpoint.header.stamp = self.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._setpoint)

    def _publish_waypoint_goal(self) -> None:
        wp = PoseStamped()
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.header.frame_id = self._frame_id
        wp.pose.position.x = self._takeoff_x
        wp.pose.position.y = self._takeoff_y
        wp.pose.position.z = self._takeoff_alt
        wp.pose.orientation.w = 1.0
        self._waypoint_goal_pub.publish(wp)
        self.get_logger().info(
            f'Published /waypoint_goal: x={self._takeoff_x:.2f} '
            f'y={self._takeoff_y:.2f} z={self._takeoff_alt:.2f}'
        )

    def _transition(self, new_state: int) -> None:
        old_name = _STATE_NAMES.get(self._fsm, str(self._fsm))
        new_name = _STATE_NAMES.get(new_state, str(new_state))
        self.get_logger().info(f'FSM: {old_name} -> {new_name}')
        self._fsm = new_state
        self._state_entry_time = self.get_clock().now()

    def _elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self._state_entry_time).nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Service response callbacks
    # ------------------------------------------------------------------

    def _offboard_response_cb(self, future) -> None:
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info('OFFBOARD mode request accepted by FCU')
            else:
                self.get_logger().warn('OFFBOARD mode request rejected by FCU')
        except Exception as exc:
            self.get_logger().error(f'set_mode call failed: {exc}')

    def _arm_response_cb(self, future) -> None:
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
    node = MavrosTakeoffNode()
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
