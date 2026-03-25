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

---

## `hover_supervisor_node`

Persistent hover supervisor that **never exits**.  It continuously streams
`PoseStamped` setpoints to `/{uav_name}/mavros/setpoint_position/local` so
OFFBOARD mode remains active even after the client node that triggered takeoff
shuts down.

> **Important – single publisher rule:** Only **one** node should publish to
> `/{uav_name}/mavros/setpoint_position/local` at a time.  Run
> `hover_supervisor_node` as the sole publisher.  If you use
> `mavros_takeoff_node` for the initial climb, stop it (Ctrl-C or kill) as
> soon as `hover_supervisor_node` has latched position; otherwise the two
> nodes will fight over the setpoint topic.

### When to use this instead of `mavros_takeoff_node`?

Use `hover_supervisor_node` when you need a *daemon-style* process to maintain
hover indefinitely.  Use `mavros_takeoff_node` when you only need a
one-shot takeoff helper that exits after the drone is airborne.

### State machine

```
IDLE → WARMUP → REQ_OFFBOARD → REQ_ARM → CONFIRM → TAKEOFF → HOVER
```

The node stays in `HOVER` indefinitely, streaming setpoints.

### Safety: no publish before odometry

The node does **not** publish any setpoints until the first odometry message
arrives.  This prevents a stale `(0, 0, 0)` setpoint from being sent to an
airborne vehicle while the node is starting up.

### Auto-latch on airborne handoff (`auto_latch_airborne_on_start`)

When `auto_latch_airborne_on_start` is `true` (the default), the node watches
for the drone to be **airborne** and already in OFFBOARD+ARM.  Once all
conditions are met *continuously* for `auto_latch_hold_time` seconds, it
latches the current position (`x`, `y`, `z`) and transitions to `HOVER`.

**Altitude safety gate:** The latch is only allowed when
`odom_z >= max(auto_latch_min_altitude, airborne_z_threshold)`.  If OFFBOARD+ARM
becomes active while the drone is still below this threshold (e.g. during a
transient climb with another publisher still running), the node emits a
throttled `WARN` log and waits.  This prevents the supervisor from latching a
low setpoint that would cause the vehicle to descend once the other publisher
stops.

If `auto_latch_requires_takeoff_altitude` is `true` (default `false`), the
latch additionally requires `odom_z >= altitude_threshold`, which is useful
when the supervisor must wait for `mavros_takeoff_node` to complete its full
climb before taking over.

This enables a clean handoff from `mavros_takeoff_node`:

1. Start `hover_supervisor_node` (it will detect the active OFFBOARD session).
2. Let `mavros_takeoff_node` finish its climb.
3. Stop `mavros_takeoff_node`.
4. `hover_supervisor_node` already holds position – the drone does not descend.

The auto-latch runs **at most once** on startup and does not continuously
relatch in HOVER.

### Auto-takeoff on start (`auto_takeoff_on_start`)

When `auto_takeoff_on_start` is `true` (default `false`) the node triggers
the same flow as calling `~/takeoff` automatically after receiving the first
odometry message.  Useful for fully-automated tmux sessions where no manual
service call is desired.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `uav_name` | `uav1` | MAVROS namespace prefix |
| `frame_id` | `map` | `frame_id` for published `PoseStamped` messages |
| `rate_hz` | `20.0` | Setpoint publish rate (Hz) |
| `takeoff_altitude` | `2.0` | Target altitude in metres (positive-UP, ENU) |
| `altitude_threshold` | `1.8` | Minimum altitude (m) to consider takeoff complete |
| `warmup_streaming_time` | `2.0` | Seconds to stream setpoints before requesting OFFBOARD |
| `timeout_confirm` | `10.0` | Seconds to wait for OFFBOARD+ARM confirmation |
| `timeout_takeoff` | `20.0` | Seconds before retrying if altitude not reached |
| `auto_offboard_arm` | `true` | Automatically request OFFBOARD + ARM on takeoff |
| `z_step_per_tick` | `0.0` | Max Z change per timer tick (0 = unlimited) |
| `auto_latch_airborne_on_start` | `true` | Auto-latch current position if already airborne+OFFBOARD+ARM on startup |
| `auto_takeoff_on_start` | `false` | Trigger takeoff automatically after first odom received |
| `airborne_z_threshold` | `0.3` | Minimum altitude (m) for auto-latch; combined with `auto_latch_min_altitude` as `max(airborne_z_threshold, auto_latch_min_altitude)` |
| `auto_latch_min_altitude` | `1.0` | Minimum `odom_z` (m) required before auto-latch is allowed; prevents latching during transient climb |
| `auto_latch_hold_time` | `1.0` | Seconds all auto-latch conditions must be continuously true before latching (stability window) |
| `auto_latch_requires_takeoff_altitude` | `false` | If `true`, also require `odom_z >= altitude_threshold` before auto-latching |

### Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/{uav_name}/mavros/state` | `mavros_msgs/msg/State` |
| Subscribe | `/{uav_name}/mavros/local_position/odom` | `nav_msgs/msg/Odometry` |
| Subscribe | `~/set_altitude` | `std_msgs/msg/Float64` |
| Publish | `/{uav_name}/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` |

### Services offered

| Service | Type | Description |
|---|---|---|
| `~/takeoff` | `std_srvs/Trigger` | Latch current XY, climb to `takeoff_altitude` |
| `~/latch_here` | `std_srvs/Trigger` | Latch current XYZ and hover |

### Services called

| Service | Type |
|---|---|
| `/{uav_name}/mavros/set_mode` | `mavros_msgs/srv/SetMode` |
| `/{uav_name}/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` |

### Example commands

Start the supervisor (runs indefinitely, auto-latches if already airborne):

```bash
ros2 run drone_custom_control hover_supervisor_node --ros-args \
  -p uav_name:=uav1 \
  -p auto_latch_airborne_on_start:=true \
  -p auto_latch_min_altitude:=1.0 \
  -p auto_latch_hold_time:=1.0 \
  -p auto_takeoff_on_start:=false
```

Trigger takeoff (from another terminal):

```bash
ros2 service call /hover_supervisor_node/takeoff std_srvs/srv/Trigger {}
```

Latch current position as hover target:

```bash
ros2 service call /hover_supervisor_node/latch_here std_srvs/srv/Trigger {}
```

Override hover altitude while airborne:

```bash
ros2 topic pub --once /hover_supervisor_node/set_altitude std_msgs/msg/Float64 \
  "{data: 3.0}"
```

### Recommended handoff sequence

Use `hover_supervisor_node` as the persistent publisher and `mavros_takeoff_node`
only for the one-shot climb:

```bash
# Terminal 1 – start supervisor first (waits for odom, then auto-latches)
# auto_latch_min_altitude=1.0 ensures the latch does not fire during the
# initial transient climb while mavros_takeoff_node is still publishing.
ros2 run drone_custom_control hover_supervisor_node --ros-args \
  -p uav_name:=uav1 \
  -p auto_latch_airborne_on_start:=true \
  -p auto_latch_min_altitude:=1.0 \
  -p auto_latch_hold_time:=1.0 \
  -p auto_takeoff_on_start:=false

# Terminal 2 – takeoff (supervisor will detect OFFBOARD+ARM and auto-latch
# once the drone has been above 1.0 m stably for 1 s)
ros2 run drone_custom_control mavros_takeoff_node --ros-args \
  -p uav_name:=uav1 \
  -p takeoff_altitude:=2.0

# Stop Terminal 2 after takeoff – supervisor holds position automatically
```

---

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
