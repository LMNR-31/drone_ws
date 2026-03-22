#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

namespace drone_control {

/**
 * @brief Centralised configuration constants for the drone controller.
 *
 * All magic numbers that were previously hard-coded throughout
 * main_codegen.cpp are collected here as named fields with sensible
 * defaults.  At runtime the node loads each field from a ROS 2 parameter
 * so the values can be overridden via drone_config.yaml without
 * recompilation.
 */
struct DroneConfig {
  // ── Altitude ──────────────────────────────────────────────────────────
  /// Target hover altitude [m]
  double hover_altitude{2.0};
  /// Hysteresis margin below hover_altitude used to detect arrival [m]
  double hover_altitude_margin{0.05};
  /// Maximum permitted altitude for incoming waypoints [m]
  double max_altitude{500.0};
  /// Time spent publishing each waypoint during trajectory execution [s]
  double waypoint_duration{4.0};

  // ── Validation limits ─────────────────────────────────────────────────
  /// Maximum allowed |X| or |Y| distance for incoming waypoints [m]
  double max_waypoint_distance{100.0};
  /// Z threshold below which the drone is considered to be landing [m]
  double land_z_threshold{0.1};

  // ── Timeouts ──────────────────────────────────────────────────────────
  /// Timeout waiting for OFFBOARD + ARM confirmation from FCU [s]
  double activation_timeout{5.0};
  /// Timeout before a pending command is automatically marked TIMEOUT [s]
  double command_timeout{15.0};
  /// Time to wait after landing is detected before requesting DISARM [s]
  double landing_timeout{3.0};
};

}  // namespace drone_control

#endif  // DRONE_CONFIG_H
