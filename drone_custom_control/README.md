# drone_custom_control

ROS 2 Python nodes for custom drone control, including an LPV-MPC
attitude controller and a robust MAVROS position-setpoint takeoff helper.

---

## Nodes

### `lpv_mpc_drone_node`

LPV-MPC attitude-based controller for hover and waypoint navigation.

#### Parameters

| Parameter | Default | Description |
|---|---|---|
| `uav_name` | `uav1` | MAVROS namespace prefix |
| `enabled` | `true` | Enable/disable control loop |
| `override_active` | `false` | Freeze FSM and publish hold setpoint when true |
| `landing_mode` | `1` | 0 = standby on ground (State 5), 1 = DISARM |
| `hover_altitude` | `2.0` | Default hover altitude (m, positive-UP) |
| `hover_altitude_margin` | `0.3` | Arrival tolerance for takeoff (m) |
| `land_z_threshold` | `0.3` | Ground altitude threshold for landing detection (m) |
| `waypoint_duration` | `5.0` | Time per trajectory waypoint (s) |
| `activation_timeout` | `10.0` | Timeout waiting for OFFBOARD+ARM confirmation (s) |
| `landing_timeout` | `5.0` | Timeout for landing phase (s) |
| `max_ref_speed_xy` | `0.5` | Maximum XY reference velocity during approach (m/s) |
| `max_ref_speed_z` | `0.5` | Maximum vertical reference velocity (m/s) |
| `max_vz_accel` | `1.5` | Maximum vertical acceleration demand fed to pos_controller (m/s²) |
| `max_tilt_rad` | `0.25` | Maximum roll/pitch angle clamp (rad, ≈14°) |
| `thrust_warn_threshold` | `0.95` | Normalised thrust above which THRUST SATURATION is logged |
| `min_takeoff_alt_rel` | `0.8` | Minimum relative altitude (m) that must be reached before takeoff can complete |
| `takeoff_complete_hold_time` | `1.5` | Seconds the arrival conditions must be continuously met before switching to HOVER |
| `use_velocity_body` | `true` | Use body-frame velocity from `/local_position/velocity_body` |
| `invert_attitude` | `false` | Negate phi/theta before sending (workaround for frame mismatches) |

#### Thrust saturation recovery parameters

When the thrust command stays at or above `saturation_thrust_threshold` for
`saturation_recovery_cycles` consecutive MPC steps (≈0.1 s each at 10 Hz),
the node enters a *recovery phase* for `saturation_recovery_duration` MPC steps.
During recovery the tilt angle clamp and XY speed limit are tightened so the
vehicle prioritises regaining vertical control before resuming lateral tracking.

| Parameter | Default | Description |
|---|---|---|
| `saturation_recovery_enabled` | `true` | Enable/disable the saturation recovery feature |
| `saturation_thrust_threshold` | `0.95` | Normalised thrust level that increments the saturation counter |
| `saturation_recovery_cycles` | `5` | Consecutive saturated MPC steps (~0.5 s) before entering recovery |
| `saturation_recovery_duration` | `10` | MPC steps (~1.0 s) to remain in recovery mode |
| `saturation_recovery_max_tilt_rad` | `0.15` | Tighter roll/pitch clamp during recovery (rad, ≈9°) |
| `saturation_recovery_max_ref_speed_xy` | `0.2` | Reduced XY speed limit during recovery (m/s) |
| `saturation_recovery_freeze_xy` | `false` | If true, freeze the XY goal to current position during recovery to avoid lateral demands |

> **Tuning advice:** If repeated `THRUST SATURATION` warnings appear during
> takeoff/hover, first try reducing `max_tilt_rad` (e.g. 0.15–0.20) and
> `max_vz_accel` (e.g. 1.0–1.5).  The saturation recovery acts as a safety net
> when transient saturation occurs; it is not a substitute for correct PID/MPC
> tuning.

#### Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/{uav_name}/mavros/local_position/odom` | `nav_msgs/msg/Odometry` |
| Subscribe | `/{uav_name}/mavros/local_position/velocity_body` | `geometry_msgs/msg/TwistStamped` |
| Subscribe | `/{uav_name}/mavros/state` | `mavros_msgs/msg/State` |
| Subscribe | `/waypoints` | `geometry_msgs/msg/PoseArray` |
| Subscribe | `/waypoint_goal` | `geometry_msgs/msg/PoseStamped` |
| Publish | `/{uav_name}/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` |

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
| `auto_disable_when_mpc_active` | `false` | Automatically disable setpoint publishing when attitude-target messages are detected on `/{uav}/mavros/setpoint_raw/attitude` |
| `handoff_timeout` | `0.5` | Seconds without attitude-target messages before considering MPC inactive (used by `auto_disable_when_mpc_active` and `auto_reenable`) |
| `auto_reenable` | `false` | Re-enable setpoint publishing automatically after no attitude-target messages for `handoff_timeout` seconds |

### Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/{uav_name}/mavros/state` | `mavros_msgs/msg/State` |
| Subscribe | `/{uav_name}/mavros/local_position/odom` | `nav_msgs/msg/Odometry` |
| Subscribe | `~/set_altitude` | `std_msgs/msg/Float64` |
| Subscribe | `/{uav_name}/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` (used by `auto_disable_when_mpc_active`) |
| Publish | `/{uav_name}/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` |

### Services offered

| Service | Type | Description |
|---|---|---|
| `~/takeoff` | `std_srvs/Trigger` | Latch current XY, climb to `takeoff_altitude`. Returns error if disabled. |
| `~/latch_here` | `std_srvs/Trigger` | Latch current XYZ and hover. Returns error if disabled. |
| `~/set_enabled` | `std_srvs/SetBool` | Enable (`data=true`) or disable (`data=false`) setpoint publishing. |

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

Enable or disable setpoint publishing:

```bash
# Disable – stop publishing so MPC can take over
ros2 service call /hover_supervisor_node/set_enabled std_srvs/srv/SetBool "{data: false}"

# Re-enable – resume hover supervisor control
ros2 service call /hover_supervisor_node/set_enabled std_srvs/srv/SetBool "{data: true}"
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

Recommended sequence using `~/set_enabled` for a clean handoff:

1. Use `hover_supervisor_node` to take off and hold hover.
2. Call `~/set_enabled false` before starting the MPC trajectory so the
   supervisor stops publishing position setpoints.
3. Start `lpv_mpc_drone_node` (or send waypoints to an already-running
   instance). The drone is now controlled exclusively by the MPC.
4. Call `~/set_enabled true` to resume hover supervisor control when MPC
   finishes.

```bash
# Step 1 – supervisor takes off and holds hover
ros2 run drone_custom_control hover_supervisor_node --ros-args \
  -p uav_name:=uav1 \
  -p auto_takeoff_on_start:=true \
  -p takeoff_altitude:=2.0

# Step 2 – disable supervisor before launching MPC
ros2 service call /hover_supervisor_node/set_enabled std_srvs/srv/SetBool "{data: false}"

# Step 3 – start MPC controller (now the only publisher)
ros2 run drone_custom_control lpv_mpc_drone_node --ros-args \
  -p uav_name:=uav1 \
  -p use_velocity_body:=true

# Step 4 – re-enable supervisor to resume hover
ros2 service call /hover_supervisor_node/set_enabled std_srvs/srv/SetBool "{data: true}"
```

**Optional – automatic handoff using `auto_disable_when_mpc_active`:**

When `auto_disable_when_mpc_active` is `true` the supervisor disables itself
automatically as soon as it detects attitude-target messages from the MPC node,
and can re-enable itself when the MPC goes silent (requires `auto_reenable:=true`):

```bash
ros2 run drone_custom_control hover_supervisor_node --ros-args \
  -p uav_name:=uav1 \
  -p auto_disable_when_mpc_active:=true \
  -p handoff_timeout:=0.5 \
  -p auto_reenable:=true
```

With this configuration the recommended sequence simplifies to:

1. Start `hover_supervisor_node` with `auto_disable_when_mpc_active:=true`.
2. Use the supervisor to take off and hold hover.
3. Start `lpv_mpc_drone_node` – the supervisor disables itself automatically.
4. Stop `lpv_mpc_drone_node` – the supervisor re-enables itself automatically
   (if `auto_reenable:=true`).

---

#### Alternative: handoff via `mavros_takeoff_node`

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
  -p use_velocity_body:=true
```
