# drone_custom_control

ROS 2 Python nodes for custom drone control, including an LPV-MPC
attitude controller and a robust MAVROS position-setpoint takeoff helper.

---

## Nodes

### `lpv_mpc_drone_node`

LPV-MPC attitude-based controller for hover and waypoint navigation.

### `mavros_takeoff_node`

Robust takeoff node that uses `/mavros/setpoint_position/local` (PX4
**internal position controller**) to arm the drone, switch to OFFBOARD mode,
and climb to a target altitude before handing control off to `lpv_mpc_drone_node`.

#### Why use this instead of letting `lpv_mpc_drone_node` take off?

The attitude-based MPC controller occasionally causes the drone to climb and
then descend during takeoff.  `mavros_takeoff_node` delegates vertical control
to the PX4 position controller, which is more reliable for the initial climb.
Once the drone is stably airborne the MPC node can take over.

#### Frame convention

All setpoints use `frame_id = "map"` (the MAVROS local-origin ENU frame).
Altitude values are **positive-UP** (ENU), matching
`/mavros/local_position/odom pose.position.z`.

#### State machine

```
WAIT_FCU → STREAM_SETPOINT → REQUEST_OFFBOARD → REQUEST_ARM →
CONFIRM_ACTIVATION → TAKEOFF → HOLD → SUCCESS | FAIL
```

Setpoints are streamed throughout **all** states to keep OFFBOARD active.

#### Parameters

| Parameter | Default | Description |
|---|---|---|
| `uav_name` | `uav1` | MAVROS namespace prefix |
| `takeoff_altitude` | `2.0` | Target altitude in metres (positive-UP) |
| `altitude_threshold` | `0.5` | Minimum altitude (m) to consider takeoff successful |
| `rate_hz` | `20.0` | Setpoint publish rate (Hz) |
| `streaming_time` | `3.0` | Seconds to stream setpoints before requesting OFFBOARD |
| `timeout_offboard` | `10.0` | Seconds to wait for OFFBOARD+ARM confirmation |
| `timeout_takeoff` | `20.0` | Seconds to wait for reaching `altitude_threshold` |
| `hold_time` | `2.0` | Seconds to hold position at altitude before exiting |
| `use_current_xy` | `true` | Latch current X/Y from odom as takeoff position |
| `x` | `0.0` | Explicit takeoff X (used when `use_current_xy` is false) |
| `y` | `0.0` | Explicit takeoff Y (used when `use_current_xy` is false) |
| `frame_id` | `map` | `frame_id` for published `PoseStamped` messages |
| `publish_waypoint_goal_on_success` | `false` | Publish a waypoint to `/waypoint_goal` on success for MPC handoff |

#### Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/{uav_name}/mavros/state` | `mavros_msgs/msg/State` |
| Subscribe | `/{uav_name}/mavros/local_position/odom` | `nav_msgs/msg/Odometry` |
| Publish | `/{uav_name}/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` |
| Publish | `/waypoint_goal` | `geometry_msgs/msg/PoseStamped` |

#### Services called

| Service | Type |
|---|---|
| `/{uav_name}/mavros/set_mode` | `mavros_msgs/srv/SetMode` |
| `/{uav_name}/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` |

#### Example commands

Minimal run (latches current XY, climbs to 2 m):

```bash
ros2 run drone_custom_control mavros_takeoff_node
```

Custom altitude and explicit XY:

```bash
ros2 run drone_custom_control mavros_takeoff_node --ros-args \
  -p uav_name:=uav1 \
  -p takeoff_altitude:=2.0 \
  -p altitude_threshold:=0.5 \
  -p use_current_xy:=true \
  -p hold_time:=2.0 \
  -p publish_waypoint_goal_on_success:=true
```

With simulation time:

```bash
ros2 run drone_custom_control mavros_takeoff_node --ros-args \
  -p use_sim_time:=true \
  -p uav_name:=uav1
```

#### Handoff to `lpv_mpc_drone_node`

1. Start `mavros_takeoff_node` with
   `publish_waypoint_goal_on_success:=true`.
2. In a second terminal, start `lpv_mpc_drone_node`.  Because the drone is
   already OFFBOARD+ARMED and at altitude, `lpv_mpc_drone_node` will detect
   the existing state (skip its own warmup) and receive the `/waypoint_goal`
   message published by the takeoff node.

```bash
# Terminal 1 – takeoff
ros2 run drone_custom_control mavros_takeoff_node --ros-args \
  -p uav_name:=uav1 \
  -p takeoff_altitude:=2.0 \
  -p publish_waypoint_goal_on_success:=true

# Terminal 2 – MPC (start any time; it waits for /waypoint_goal)
ros2 run drone_custom_control lpv_mpc_drone_node --ros-args \
  -p uav_name:=uav1 \
  -p use_velocity_body:=true \
  -p max_vz_accel:=4.0 \
  -p max_tilt_rad:=0.40
```
