#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "drone_control/msg/yaw_override.hpp"
#include "drone_control/msg/waypoint4_d.hpp"
#include "drone_control/msg/waypoint4_d_array.hpp"
#include "drone_config.h"
#include <vector>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace drone_control {

// ============================================================
// SISTEMA DE FILA DE COMANDOS COM RASTREABILIDADE
// ============================================================

/// @brief Type of drone command being tracked.
enum class CommandType {
  ARM,
  DISARM,
  SET_MODE_OFFBOARD,
  TAKEOFF,
  HOVER,
  TRAJECTORY,
  LAND
};

/// @brief Lifecycle status of a command.
enum class CommandStatus {
  PENDING,    // PENDENTE
  CONFIRMED,  // CONFIRMADO
  FAILED,     // FALHO
  TIMEOUT     // TIMEOUT
};

/// @brief Represents a single drone command with full audit metadata.
struct Command {
  uint64_t id{0};
  CommandType type{CommandType::ARM};
  CommandStatus status{CommandStatus::PENDING};
  std::chrono::system_clock::time_point timestamp{};
  std::chrono::system_clock::time_point confirm_time{};
  std::map<std::string, std::string> data;

  std::string type_str() const {
    switch (type) {
      case CommandType::ARM:               return "ARM";
      case CommandType::DISARM:            return "DISARM";
      case CommandType::SET_MODE_OFFBOARD: return "SET_MODE_OFFBOARD";
      case CommandType::TAKEOFF:           return "TAKEOFF";
      case CommandType::HOVER:             return "HOVER";
      case CommandType::TRAJECTORY:        return "TRAJECTORY";
      case CommandType::LAND:              return "LAND";
      default:                             return "DESCONHECIDO";
    }
  }

  std::string status_str() const {
    switch (status) {
      case CommandStatus::PENDING:   return "PENDENTE";
      case CommandStatus::CONFIRMED: return "CONFIRMADO";
      case CommandStatus::FAILED:    return "FALHO";
      case CommandStatus::TIMEOUT:   return "TIMEOUT";
      default:                       return "DESCONHECIDO";
    }
  }
};

/**
 * @brief Thread-safe queue that tracks drone commands with full history.
 *
 * Commands are enqueued with a unique ID.  Callers later confirm
 * (success/failure) the command via that ID.  A periodic timeout check
 * moves stale pending commands to TIMEOUT status.  The full history can be
 * persisted to a structured log file.
 */
class CommandQueue {
public:
  CommandQueue() : next_id_(1) {}

  /// Destructor: clears all pending commands and history safely.
  ~CommandQueue() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear();
    history_.clear();
  }

  /// @brief Enqueue a new command and return its unique ID.
  uint64_t enqueue(CommandType type,
                   const std::map<std::string, std::string> & data = {}) {
    std::lock_guard<std::mutex> lock(mutex_);
    Command cmd;
    cmd.id = next_id_++;
    cmd.type = type;
    cmd.status = CommandStatus::PENDING;
    cmd.timestamp = std::chrono::system_clock::now();
    cmd.data = data;
    pending_[cmd.id] = cmd;
    history_.push_back(cmd);
    return cmd.id;
  }

  /// @brief Confirm or mark as failed a pending command.
  /// @return true if the command was found and updated.
  bool confirm(uint64_t id, bool success = true) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = pending_.find(id);
    if (it == pending_.end()) {
      return false;
    }
    CommandStatus new_status = success ? CommandStatus::CONFIRMED : CommandStatus::FAILED;
    it->second.status = new_status;
    it->second.confirm_time = std::chrono::system_clock::now();
    for (auto & h : history_) {
      if (h.id == id) {
        h.status = new_status;
        h.confirm_time = it->second.confirm_time;
        break;
      }
    }
    pending_.erase(it);
    return true;
  }

  /**
   * @brief Check for timed-out pending commands and move them to TIMEOUT.
   *
   * The entire operation is performed under the queue mutex so that history_
   * and pending_ are always modified together atomically.
   *
   * @return IDs of commands that expired.
   */
  std::vector<uint64_t> check_timeouts(double timeout_seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint64_t> timed_out;
    auto now = std::chrono::system_clock::now();
    for (auto it = pending_.begin(); it != pending_.end(); ) {
      double elapsed =
        std::chrono::duration<double>(now - it->second.timestamp).count();
      if (elapsed > timeout_seconds) {
        it->second.status = CommandStatus::TIMEOUT;
        // Update the history entry under the same lock to avoid data races
        // between check_timeouts() and save_log().
        for (auto & h : history_) {
          if (h.id == it->second.id) {
            h.status = CommandStatus::TIMEOUT;
            break;
          }
        }
        timed_out.push_back(it->second.id);
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
    return timed_out;
  }

  /// @brief Return a snapshot copy of the command history (thread-safe).
  std::vector<Command> get_history() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_;
  }

  size_t pending_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.size();
  }

  /// @brief Persist the full command history to a structured log file.
  void save_log(const std::string & filename) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "[CommandQueue] ERRO: Nao foi possivel abrir arquivo de log: "
                << filename << "\n";
      return;
    }
    file << "=== HISTORICO DE COMANDOS DO DRONE ===\n";
    file << "Total: " << history_.size() << " comandos\n\n";
    for (const auto & cmd : history_) {
      std::time_t t = std::chrono::system_clock::to_time_t(cmd.timestamp);
      std::tm tm_buf{};
#ifdef _WIN32
      localtime_s(&tm_buf, &t);
#else
      localtime_r(&t, &tm_buf);
#endif
      file << std::put_time(&tm_buf, "[%Y-%m-%d %H:%M:%S]")
           << " ID=" << std::setw(4) << std::right << cmd.id
           << " | TIPO="   << std::setw(18) << std::left << cmd.type_str()
           << " | STATUS=" << std::setw(10) << std::left << cmd.status_str();
      if (!cmd.data.empty()) {
        file << " | DADOS={";
        bool first = true;
        for (const auto & kv : cmd.data) {
          if (!first) { file << ", "; }
          file << kv.first << "=" << kv.second;
          first = false;
        }
        file << "}";
      }
      if (cmd.status == CommandStatus::CONFIRMED ||
          cmd.status == CommandStatus::FAILED) {
        double elapsed = std::chrono::duration<double>(
          cmd.confirm_time - cmd.timestamp).count();
        file << " | TEMPO=" << std::fixed << std::setprecision(2) << elapsed << "s";
      }
      file << "\n";
    }
    file.close();
  }

private:
  mutable std::mutex mutex_;
  uint64_t next_id_;
  std::map<uint64_t, Command> pending_;
  std::vector<Command> history_;
};

// ============================================================
// VALIDACAO DE WAYPOINTS
// ============================================================

/**
 * @brief Validate a PoseStamped waypoint against physical limits.
 *
 * Checks for NaN/Inf coordinates and enforces the XY distance and altitude
 * limits defined in @p config.
 *
 * @return true if the waypoint is safe to use.
 */
bool validate_waypoint(const geometry_msgs::msg::PoseStamped & msg,
                       const DroneConfig & config)
{
  const auto & pos = msg.pose.position;

  if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) return false;
  if (std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) return false;

  // Z < land_z_threshold: landing intent – always accepted regardless of min_altitude.
  // Skip altitude range checks and fall through to XY validation.
  if (pos.z >= config.land_z_threshold) {
    if (pos.z < config.min_altitude) {
      // Between land_z_threshold and min_altitude: invalid flight altitude
      RCLCPP_WARN(rclcpp::get_logger("validate_waypoint"),
        "❌ Waypoint Z=%.2fm REJEITADO: abaixo da altitude mínima de voo (%.2fm)",
        pos.z, config.min_altitude);
      return false;
    }
    if (pos.z > config.max_altitude) {
      RCLCPP_WARN(rclcpp::get_logger("validate_waypoint"),
        "❌ Waypoint Z=%.2fm REJEITADO: acima da altitude máxima (%.2fm)",
        pos.z, config.max_altitude);
      return false;
    }
  }

  if (std::abs(pos.x) > config.max_waypoint_distance)           return false;
  if (std::abs(pos.y) > config.max_waypoint_distance)           return false;

  return true;
}

/**
 * @brief Validate a plain Pose (without header) against physical limits.
 *
 * Convenience wrapper around validate_waypoint().
 */
bool validate_pose(const geometry_msgs::msg::Pose & pose,
                   const DroneConfig & config)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.pose = pose;
  return validate_waypoint(ps, config);
}

// ============================================================
// CONTROLADOR PRINCIPAL DO DRONE
// ============================================================

/**
 * @brief Complete drone controller node with command tracking and safety.
 *
 * Implements a 6-state flight state machine:
 *  - State 0: Waiting for waypoint goal
 *  - State 1: Takeoff (OFFBOARD + ARM, climb to hover_altitude)
 *  - State 2: Hover (waiting for trajectory)
 *  - State 3: Executing trajectory
 *  - State 4: Landing / paused
 *  - State 5: Standby on ground (Modo A only – OFFBOARD+ARMED, publishing
 *              low-Z setpoint to hold the drone on the ground without
 *              disarming; exits to State 1 on new flight waypoint)
 *
 * All drone commands are registered in a CommandQueue so that every
 * operation can be audited and timed out automatically.
 *
 * The `landing_mode` ROS 2 parameter selects the end-of-landing behaviour:
 *  - 0 (Modo A): transition to State 5 – stay OFFBOARD+ARMED on the ground.
 *  - 1 (Modo B): DISARM and full reset to State 0 (default / current behaviour).
 *
 * NOTE: drone_control/src/drone_soft_land.cpp also issues a DISARM after its
 * own landing sequence.  When landing_mode=0 that node must NOT be running
 * alongside this controller, otherwise it will disarm the drone and prevent
 * State 5 from functioning correctly.
 */
class DroneControllerCompleto : public rclcpp::Node {
public:

  // type_mask bits for PositionTarget (1 = ignore that field)
  static constexpr uint16_t IGNORE_VX       = (1 << 3);
  static constexpr uint16_t IGNORE_VY       = (1 << 4);
  static constexpr uint16_t IGNORE_VZ       = (1 << 5);
  static constexpr uint16_t IGNORE_AFX      = (1 << 6);
  static constexpr uint16_t IGNORE_AFY      = (1 << 7);
  static constexpr uint16_t IGNORE_AFZ      = (1 << 8);
  static constexpr uint16_t IGNORE_YAW      = (1 << 10);
  static constexpr uint16_t IGNORE_YAW_RATE = (1 << 11);

  // Position + yaw_rate; ignore velocity, acceleration, yaw angle
  static constexpr uint16_t MASK_POS_YAWRATE =
    IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW;

  // Position + absolute yaw; ignore velocity, acceleration, yaw_rate
  static constexpr uint16_t MASK_POS_YAW =
    IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;

  // Position only; ignore velocity, acceleration, yaw angle and yaw_rate
  static constexpr uint16_t MASK_POS_ONLY = MASK_POS_YAWRATE | IGNORE_YAW_RATE;

  DroneControllerCompleto() : Node("drone_controller_completo") {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🚁 CONTROLADOR INTELIGENTE DE DRONE - VERSÃO FINAL      ║");
    RCLCPP_INFO(this->get_logger(), "║     COM ATIVAÇÃO OFFBOARD + ARM + DETECÇÃO DE POUSO     ║");
    RCLCPP_INFO(this->get_logger(), "║               345 LINHAS - CÓDIGO COMPLETO               ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════════╝\n");

    // ==========================================
    // CARREGAR CONFIGURAÇÃO (ROS 2 parâmetros)
    // ==========================================
    this->declare_parameter("hover_altitude",        config_.hover_altitude);
    this->declare_parameter("hover_altitude_margin", config_.hover_altitude_margin);
    this->declare_parameter("max_altitude",          config_.max_altitude);
    this->declare_parameter("min_altitude",          config_.min_altitude);
    this->declare_parameter("waypoint_duration",     config_.waypoint_duration);
    this->declare_parameter("max_waypoint_distance", config_.max_waypoint_distance);
    this->declare_parameter("land_z_threshold",      config_.land_z_threshold);
    this->declare_parameter("activation_timeout",    config_.activation_timeout);
    this->declare_parameter("command_timeout",       config_.command_timeout);
    this->declare_parameter("landing_timeout",       config_.landing_timeout);

    config_.hover_altitude        = this->get_parameter("hover_altitude").as_double();
    config_.hover_altitude_margin = this->get_parameter("hover_altitude_margin").as_double();
    config_.max_altitude          = this->get_parameter("max_altitude").as_double();
    config_.min_altitude          = this->get_parameter("min_altitude").as_double();
    config_.waypoint_duration     = this->get_parameter("waypoint_duration").as_double();
    config_.max_waypoint_distance = this->get_parameter("max_waypoint_distance").as_double();
    config_.land_z_threshold      = this->get_parameter("land_z_threshold").as_double();
    config_.activation_timeout    = this->get_parameter("activation_timeout").as_double();
    config_.command_timeout       = this->get_parameter("command_timeout").as_double();
    config_.landing_timeout       = this->get_parameter("landing_timeout").as_double();

    // landing_mode: 0=Modo A (standby no chão, OFFBOARD+ARMED), 1=Modo B (DISARM, padrão)
    this->declare_parameter<int>("landing_mode", 1);
    landing_mode_ = this->get_parameter("landing_mode").as_int();
    if (landing_mode_ != 0 && landing_mode_ != 1) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  landing_mode=%d inválido; usando 1 (DISARM)", landing_mode_);
      landing_mode_ = 1;
    }
    RCLCPP_INFO(this->get_logger(),
      "⚙️  landing_mode=%d (%s)", landing_mode_,
      landing_mode_ == 0 ? "Modo A - standby no chão" : "Modo B - DISARM");

    // enabled: se false, o controle loop não publica setpoints e pausa a FSM
    this->declare_parameter<bool>("enabled", true);
    enabled_ = this->get_parameter("enabled").as_bool();
    RCLCPP_INFO(this->get_logger(), "⚙️  enabled=%s", enabled_ ? "true" : "false");

    // override_active: quando true, um nó externo (ex: drone_yaw_360) está
    // sinalizando override.  O control_loop congela a FSM mas CONTINUA
    // publicando hold setpoints (posição atual do odom) para manter OFFBOARD
    // ativo.  Alternar via: ros2 param set /drone_controller_completo override_active true
    this->declare_parameter<bool>("override_active", false);
    override_active_ = this->get_parameter("override_active").as_bool();
    RCLCPP_INFO(this->get_logger(), "⚙️  override_active=%s", override_active_ ? "true" : "false");

    // Callback para atualização dinâmica dos parâmetros em runtime
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DroneControllerCompleto::onSetParameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "⚙️  Configuração carregada:");
    RCLCPP_INFO(this->get_logger(), "   hover_altitude=%.2f m  margin=%.3f m",
      config_.hover_altitude, config_.hover_altitude_margin);
    RCLCPP_INFO(this->get_logger(), "   waypoint_duration=%.1f s  command_timeout=%.1f s",
      config_.waypoint_duration, config_.command_timeout);
    RCLCPP_INFO(this->get_logger(),
      "⚙️  Configuração de Altitude: Mínima=%.2f m | Pouso detectado: Z < %.2f m | Máxima=%.2f m",
      config_.min_altitude, config_.land_z_threshold, config_.max_altitude);

    // ==========================================
    // PUBLISHERS
    // ==========================================
    raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
      "/uav1/mavros/setpoint_raw/local", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /uav1/mavros/setpoint_raw/local");

    // ==========================================
    // SUBSCRIBERS
    // ==========================================
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
      });

    waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/waypoints", 1,
      std::bind(&DroneControllerCompleto::waypoints_callback, this, std::placeholders::_1)
    );

    waypoint_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/waypoint_goal", 1,
      std::bind(&DroneControllerCompleto::waypoint_goal_callback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&DroneControllerCompleto::odometry_callback, this, std::placeholders::_1));

    yaw_override_sub_ = this->create_subscription<drone_control::msg::YawOverride>(
      "/uav1/yaw_override/cmd", 10,
      std::bind(&DroneControllerCompleto::yaw_override_callback, this, std::placeholders::_1));

    waypoint_goal_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4D>(
      "/waypoint_goal_4d", 1,
      std::bind(&DroneControllerCompleto::waypoint_goal_4d_callback, this, std::placeholders::_1));

    waypoints_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4DArray>(
      "/waypoints_4d", 1,
      std::bind(&DroneControllerCompleto::waypoints_4d_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "✓ Subscribers criados: /uav1/mavros/state, /waypoints, /waypoint_goal, odometria, /uav1/yaw_override/cmd, /waypoint_goal_4d e /waypoints_4d");

    // ==========================================
    // SERVICE CLIENTS - ATIVAÇÃO
    // ==========================================
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

    RCLCPP_INFO(this->get_logger(), "✓ Service Clients criados: set_mode e arming");

    // ==========================================
    // AGUARDAR SERVIÇOS DISPONÍVEIS
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "⏳ Aguardando serviços MAVROS...");
    
    while (!mode_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/set_mode...");
    }
    RCLCPP_INFO(this->get_logger(), "✓ Serviço set_mode disponível");

    while (!arm_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/cmd/arming...");
    }
    RCLCPP_INFO(this->get_logger(), "✓ Serviço arming disponível\n");

    // ==========================================
    // TIMER DO CONTROLE
    // ==========================================
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&DroneControllerCompleto::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "✓ Timer criado: 100 Hz (10ms)");

    // ==========================================
    // INICIALIZAÇÃO DE VARIÁVEIS
    // ==========================================
    state_voo_ = 0;
    controlador_ativo_ = false;
    pouso_em_andamento_ = false;
    cycle_count_ = 0;
    offboard_activated_ = false;
    activation_confirmed_ = false;
    takeoff_counter_ = 0;
    waypoint_goal_received_ = false;
    last_z_ = 0.0;
    pouso_start_time_set_ = false;
    pouso_start_time_ = this->now();
    last_waypoint_goal_.pose.position.x = 0.0;
    last_waypoint_goal_.pose.position.y = 0.0;
    last_waypoint_goal_.pose.position.z = config_.hover_altitude;
    trajectory_setpoint_[0] = 0.0;
    trajectory_setpoint_[1] = 0.0;
    trajectory_setpoint_[2] = config_.hover_altitude;
    trajectory_waypoints_.clear();
    trajectory_started_ = false;
    current_waypoint_idx_ = 0;
    waypoint_duration_ = config_.waypoint_duration;
    current_z_real_ = 0.0;
    current_x_ned_ = 0.0;
    current_y_ned_ = 0.0;
    current_z_ned_ = 0.0;
    yaw_override_enabled_ = false;
    yaw_rate_cmd_ = 0.0;
    yaw_override_timeout_s_ = 0.3;
    hold_valid_ = false;
    hold_x_ned_ = 0.0;
    hold_y_ned_ = 0.0;
    hold_z_ned_ = 0.0;
    last_yaw_cmd_time_ = this->now();
    ground_hold_x_ = 0.0;
    ground_hold_y_ = 0.0;
    ground_hold_z_ = std::max(0.01, config_.land_z_threshold);
    state5_recovery_time_ = this->now();
    current_yaw_rad_ = 0.0;
    goal_yaw_rad_ = 0.0;
    using_4d_goal_ = false;
    trajectory_4d_mode_ = false;

    RCLCPP_INFO(this->get_logger(), "\n📊 STATUS INICIAL:");
    RCLCPP_INFO(this->get_logger(), "   Estado: %d (aguardando waypoint)", state_voo_);
    RCLCPP_INFO(this->get_logger(), "   Controlador: %s", controlador_ativo_ ? "ATIVO" : "INATIVO");
    RCLCPP_INFO(this->get_logger(), "   Pouso: %s\n", pouso_em_andamento_ ? "SIM" : "NÃO");
  }

private:

  // ==========================================
  // CALLBACK DE PARÂMETROS EM RUNTIME
  // ==========================================
  /**
   * @brief Validates and applies dynamic changes to ROS 2 parameters.
   *
   * Handles:
   *  - `landing_mode`: integer 0 (Modo A) or 1 (Modo B).
   *  - `enabled`: bool; when false the control loop skips publishing and FSM
   *    state advances, allowing another node (e.g. drone_yaw_360) to take
   *    full control of the setpoint stream.
   *  - `override_active`: bool; when true an external node has requested FSM
   *    freeze.  The controller CONTINUES publishing hold setpoints (at current
   *    odom position) to keep OFFBOARD alive.  Unlike `enabled`, the setpoint
   *    stream is never dropped.
   */
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      if (p.get_name() == "landing_mode") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          result.successful = false;
          result.reason = "landing_mode deve ser inteiro (0 ou 1)";
          return result;
        }
        int64_t new_val = p.as_int();
        if (new_val != 0 && new_val != 1) {
          result.successful = false;
          result.reason = "landing_mode deve ser 0 (Modo A - standby no chão) ou 1 (Modo B - DISARM)";
          return result;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        int old_val = landing_mode_;
        landing_mode_ = static_cast<int>(new_val);
        RCLCPP_INFO(this->get_logger(),
          "🔄 landing_mode atualizado: %d → %d (%s)",
          old_val, landing_mode_,
          landing_mode_ == 0 ? "Modo A - standby no chão" : "Modo B - DISARM");
      } else if (p.get_name() == "enabled") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "enabled deve ser bool (true ou false)";
          return result;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        bool old_val = enabled_;
        enabled_ = p.as_bool();
        if (old_val != enabled_) {
          if (enabled_) {
            RCLCPP_INFO(this->get_logger(),
              "✅ enabled atualizado: false → true (publicação de setpoints RETOMADA)");
          } else {
            RCLCPP_INFO(this->get_logger(),
              "🚫 enabled atualizado: true → false (publicação de setpoints PAUSADA)");
          }
        }
      } else if (p.get_name() == "override_active") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "override_active deve ser bool (true ou false)";
          return result;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        bool old_val = override_active_;
        override_active_ = p.as_bool();
        if (old_val != override_active_) {
          if (override_active_) {
            // Capture hold position from current odom NED on activation
            hold_x_ned_ = current_x_ned_;
            hold_y_ned_ = current_y_ned_;
            hold_z_ned_ = current_z_ned_;
            hold_valid_ = true;
            RCLCPP_INFO(this->get_logger(),
              "🔒 override_active: false → true (override externo ATIVO: FSM congelada, publicando hold setpoint X=%.2f Y=%.2f Z=%.2f)",
              hold_x_ned_, hold_y_ned_, hold_z_ned_);
          } else {
            RCLCPP_INFO(this->get_logger(),
              "🔓 override_active: true → false (override externo DESATIVADO: operação normal RETOMADA)");
          }
        }
      }
    }
    return result;
  }

  // ==========================================
  // SOLICITA OFFBOARD MODE
  // ==========================================
  /**
   * @brief Asynchronously request OFFBOARD mode from the FCU.
   *
   * The result callback uses a std::weak_ptr to avoid dangling-pointer
   * undefined behaviour if the node is destroyed before the response
   * arrives.
   */
  void request_offboard() {
    if (!mode_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";

    uint64_t cmd_id = cmd_queue_.enqueue(
      CommandType::SET_MODE_OFFBOARD, {{"mode", "OFFBOARD"}});
    offboard_cmd_id_ = cmd_id;

    // Capture a weak reference so the callback is safe even if the node is
    // destroyed before the service response arrives.
    std::weak_ptr<DroneControllerCompleto> weak_self(
      std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

    auto callback = [weak_self, cmd_id](
      rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      auto self = weak_self.lock();
      if (!self) { return; }  // Node was destroyed – abort safely
      auto result = future.get();
      bool mode_set_success = result && result->mode_sent;
      self->cmd_queue_.confirm(cmd_id, mode_set_success);
      if (mode_set_success) {
        RCLCPP_INFO(self->get_logger(),
          "✅ [ID=%lu] SET_MODE OFFBOARD aceito pelo FCU", cmd_id);
      } else {
        RCLCPP_WARN(self->get_logger(),
          "⚠️ [ID=%lu] SET_MODE OFFBOARD rejeitado pelo FCU", cmd_id);
      }
    };

    mode_client_->async_send_request(request, callback);
    RCLCPP_INFO(this->get_logger(),
      "📡 [ID=%lu] Solicitando OFFBOARD MODE...", cmd_id);
  }

  // ==========================================
  // SOLICITA ARM (ARMAR DRONE)
  // ==========================================
  /**
   * @brief Asynchronously request ARM from the FCU.
   *
   * Uses weak_ptr capture to prevent dangling-pointer UB.
   */
  void request_arm() {
    if (!arm_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    uint64_t cmd_id = cmd_queue_.enqueue(CommandType::ARM);
    arm_cmd_id_ = cmd_id;

    std::weak_ptr<DroneControllerCompleto> weak_self(
      std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

    auto callback = [weak_self, cmd_id](
      rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      auto self = weak_self.lock();
      if (!self) { return; }
      auto result = future.get();
      bool arm_confirmed = result && result->success;
      self->cmd_queue_.confirm(cmd_id, arm_confirmed);
      if (arm_confirmed) {
        RCLCPP_INFO(self->get_logger(),
          "✅ [ID=%lu] ARM confirmado pelo FCU", cmd_id);
      } else {
        RCLCPP_WARN(self->get_logger(),
          "⚠️ [ID=%lu] ARM rejeitado pelo FCU", cmd_id);
      }
    };

    arm_client_->async_send_request(request, callback);
    RCLCPP_INFO(this->get_logger(), "🔋 [ID=%lu] Solicitando ARM...", cmd_id);
  }

  // ==========================================
  // SOLICITA DISARM (DESARMAR DRONE)
  // ==========================================
  /**
   * @brief Asynchronously request DISARM from the FCU.
   *
   * Uses weak_ptr capture to prevent dangling-pointer UB.
   */
  void request_disarm() {
    if (!arm_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;

    uint64_t cmd_id = cmd_queue_.enqueue(CommandType::DISARM);
    disarm_cmd_id_ = cmd_id;

    std::weak_ptr<DroneControllerCompleto> weak_self(
      std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

    auto callback = [weak_self, cmd_id](
      rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      auto self = weak_self.lock();
      if (!self) { return; }
      auto result = future.get();
      bool disarm_confirmed = result && result->success;
      self->cmd_queue_.confirm(cmd_id, disarm_confirmed);
      if (disarm_confirmed) {
        RCLCPP_INFO(self->get_logger(),
          "✅ [ID=%lu] DISARM confirmado pelo FCU", cmd_id);
      } else {
        RCLCPP_WARN(self->get_logger(),
          "⚠️ [ID=%lu] DISARM rejeitado pelo FCU", cmd_id);
      }
    };

    arm_client_->async_send_request(request, callback);
    RCLCPP_INFO(this->get_logger(), "🔴 [ID=%lu] Solicitando DISARM...", cmd_id);
  }

  // ==========================================
  // CALLBACK: ODOMETRIA (Z REAL DO DRONE)
  // ==========================================
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // /uav1/mavros/local_position/odom usa NED (North-East-Down).
    // Z negativo em NED = altura; usamos o valor absoluto para ter
    // a altitude sempre positiva (0 no solo, ~hover_altitude em hover).
    current_x_ned_ = msg->pose.pose.position.x;
    current_y_ned_ = msg->pose.pose.position.y;
    current_z_ned_ = msg->pose.pose.position.z;
    current_z_real_ = std::abs(current_z_ned_);
    // Extrair yaw do quaternion do odom (wrap para [-pi, pi])
    const auto & q = msg->pose.pose.orientation;
    current_yaw_rad_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // ==========================================
  // CALLBACK: YAW OVERRIDE COMMAND
  // ==========================================
  void yaw_override_callback(const drone_control::msg::YawOverride::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->enable) {
      // On false->true transition: capture hold position from current odom NED
      if (!yaw_override_enabled_) {
        hold_x_ned_ = current_x_ned_;
        hold_y_ned_ = current_y_ned_;
        hold_z_ned_ = current_z_ned_;
        hold_valid_ = true;
        RCLCPP_INFO(this->get_logger(),
          "🔒 Yaw override ATIVADO: hold X=%.2f Y=%.2f Z=%.2f (NED), yaw_rate=%.3f rad/s",
          hold_x_ned_, hold_y_ned_, hold_z_ned_, static_cast<double>(msg->yaw_rate));
      }
      yaw_override_enabled_ = true;
      yaw_rate_cmd_ = static_cast<double>(msg->yaw_rate);
      if (msg->timeout > 0.0f) {
        yaw_override_timeout_s_ = static_cast<double>(msg->timeout);
      }
    } else {
      if (yaw_override_enabled_) {
        RCLCPP_INFO(this->get_logger(), "🔓 Yaw override DESATIVADO: retomando operação normal da FSM.");
      }
      yaw_override_enabled_ = false;
      yaw_rate_cmd_ = 0.0;
    }
    last_yaw_cmd_time_ = this->now();
  }

  // ==========================================
  // CALLBACK: RECEBE WAYPOINTS
  // ==========================================
  void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Quando recebe trajetória 3D, limpa o modo 4D
    trajectory_4d_mode_ = false;
    // ✅ VALIDAÇÃO: Mínimo 1 waypoint
    if (msg->poses.size() < 1) {
      RCLCPP_WARN(this->get_logger(), "❌ Waypoints insuficientes: %zu", msg->poses.size());
      return;
    }

    // ✅ VALIDAÇÃO: Verifica cada waypoint contra limites físicos
    for (size_t i = 0; i < msg->poses.size(); ++i) {
      if (!validate_pose(msg->poses[i], config_)) {
        RCLCPP_WARN(this->get_logger(),
          "❌ Waypoint[%zu] inválido (NaN/Inf ou fora dos limites físicos) - mensagem ignorada", i);
        return;
      }
    }

    double last_z = msg->poses.back().position.z;

    // ==========================================
    // DETECTA POUSO (Z < land_z_threshold)
    // ==========================================
    if (msg->poses.size() == 1 && last_z < config_.land_z_threshold) {
      // Se já estiver em standby no chão (estado 5), ignorar comando de pouso
      if (state_voo_ == 5) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "ℹ️ Comando de pouso ignorado: drone já em standby no chão (estado 5)");
        return;
      }
      RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z_final = %.2f m", last_z);
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4; // VAI DIRETO PARA ESTADO 4!
      land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(last_z)}});
      RCLCPP_WARN(this->get_logger(),
        "📋 [ID=%lu] Comando LAND enfileirado", *land_cmd_id_);
      // Salva ponto de ancoragem para Mode A (ground hold durante e após pouso)
      ground_hold_x_ = msg->poses[0].position.x;
      ground_hold_y_ = msg->poses[0].position.y;
      ground_hold_z_ = std::max(0.01, config_.land_z_threshold);
      RCLCPP_WARN(this->get_logger(),
        "📍 Ancoragem de pouso definida: X=%.2f, Y=%.2f, Z=%.3f",
        ground_hold_x_, ground_hold_y_, ground_hold_z_);
      RCLCPP_WARN(this->get_logger(), "🛬 CONTROLADOR DESLIGADO - DEIXANDO drone_soft_land POUSAR\n");
      return;
    }

    // ==========================================
    // ESTRATÉGIA 1: 1 WAYPOINT = LEVANTAMENTO
    // ==========================================
    if (msg->poses.size() == 1 && last_z >= config_.land_z_threshold) {
      RCLCPP_INFO(this->get_logger(), "\n⬆️ WAYPOINT DE LEVANTAMENTO recebido:");
      RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.2f",
        msg->poses[0].position.x,
        msg->poses[0].position.y,
        msg->poses[0].position.z);

      last_waypoint_goal_.pose = msg->poses[0];

      // RESET: Limpa flags do ciclo anterior
      pouso_em_andamento_ = false;
      controlador_ativo_ = false;

      // Reset adicional de trajetória
      trajectory_started_ = false;
      trajectory_waypoints_.clear();
      current_waypoint_idx_ = 0;

      // Log de debug ANTES da verificação
      RCLCPP_INFO(this->get_logger(), "🔍 DEBUG FLAGS ANTES:");
      RCLCPP_INFO(this->get_logger(), "   offboard_activated_=%d", offboard_activated_);
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d", state_voo_);
      RCLCPP_INFO(this->get_logger(), "   activation_confirmed_=%d", activation_confirmed_);

      // Se estiver em estado 5 (standby no chão, Modo A) e já OFFBOARD+ARM, não
      // precisamos resetar as flags nem solicitar ativação novamente.
      if (state_voo_ == 5 &&
          current_state_.armed && current_state_.mode == "OFFBOARD") {
        RCLCPP_INFO(this->get_logger(),
          "🔋 Estado 5: já OFFBOARD+ARM – reutilizando ativação para levantamento...\n");
        // offboard_activated_ e activation_confirmed_ já estão ok; não reseta
      } else {
        // Força reativação, independentemente do estado anterior
        RCLCPP_INFO(this->get_logger(), "🔋 Ativando OFFBOARD+ARM para levantamento...\n");

        // Reset explícito ANTES de reativar
        offboard_activated_ = false;
        activation_confirmed_ = false;

        // Solicita OFFBOARD MODE e ARM
        request_offboard();
        request_arm();

        // Marca como ativado e aguarda confirmação do FCU
        offboard_activated_ = true;
        activation_time_ = this->now();
      }

      // Enfileira comando de TAKEOFF
      takeoff_cmd_id_ = cmd_queue_.enqueue(
        CommandType::TAKEOFF,
        {{"x", std::to_string(msg->poses[0].position.x)},
         {"y", std::to_string(msg->poses[0].position.y)},
         {"z", std::to_string(config_.hover_altitude)}});
      RCLCPP_INFO(this->get_logger(),
        "📋 [ID=%lu] Comando TAKEOFF enfileirado", *takeoff_cmd_id_);

      // Vai direto para ESTADO 1 (decolagem) aguardando OFFBOARD+ARM confirmados
      state_voo_ = 1;
      takeoff_counter_ = 0;

      // Log de debug DEPOIS da verificação
      RCLCPP_INFO(this->get_logger(), "🔍 DEBUG FLAGS DEPOIS:");
      RCLCPP_INFO(this->get_logger(), "   offboard_activated_=%d", offboard_activated_);
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d\n", state_voo_);

      return;
    }

    // ==========================================
    // ESTRATÉGIA 2: 2+ WAYPOINTS = TRAJETÓRIA
    // (COM ou SEM descida de pouso)
    // ==========================================
    if (msg->poses.size() >= 2) {

      // Se pouso em andamento (estado 4 ou 5), ignora novos waypoints de trajetória
      // para não corromper last_waypoint_goal_
      if (state_voo_ == 4 || state_voo_ == 5) {
        RCLCPP_WARN(this->get_logger(),
          "⚠️ Ignorando waypoints de trajetória durante pouso/standby (estado %d)", state_voo_);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "🔍 Trajetória (2+ waypoints) recebida");
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d (esperado 2 para ativar)", state_voo_);

      // SEMPRE armazena waypoints (mesmo se ESTADO != 2)
      trajectory_waypoints_ = msg->poses;
      trajectory_yaws_.clear();  // 3D: sem yaws por waypoint
      current_waypoint_idx_ = 0;
      trajectory_started_ = false;
      last_waypoint_goal_.pose = msg->poses[0];

      RCLCPP_INFO(this->get_logger(), "✈️ WAYPOINTS DE TRAJETÓRIA armazenados: %zu pontos", msg->poses.size());
      for (size_t i = 0; i < msg->poses.size(); i++) {
        RCLCPP_INFO(this->get_logger(),
          "   WP[%zu]: X=%.2f, Y=%.2f, Z=%.2f",
          i,
          msg->poses[i].position.x,
          msg->poses[i].position.y,
          msg->poses[i].position.z);
      }
      RCLCPP_INFO(this->get_logger(), " ");

      // Se NÃO em HOVER (ESTADO 2), apenas armazena e aguarda
      if (state_voo_ != 2) {
        RCLCPP_INFO(this->get_logger(),
          "⏸️ Trajetória armazenada - Será ativada quando drone chegar em HOVER (ESTADO 2)");
        controlador_ativo_ = false;
        pouso_em_andamento_ = false;
        return;
      }

      // Se EM HOVER (ESTADO 2), ativa trajetória IMEDIATAMENTE
      RCLCPP_INFO(this->get_logger(), "\n✅ TRAJETÓRIA ACEITA E ATIVADA! Drone em HOVER pronto!\n");

      // Confirma comando de HOVER e inicia TRAJECTORY
      if (hover_cmd_id_) {
        cmd_queue_.confirm(*hover_cmd_id_, true);
        RCLCPP_INFO(this->get_logger(),
          "✅ [ID=%lu] HOVER confirmado - iniciando trajetória", *hover_cmd_id_);
        hover_cmd_id_.reset();
      }
      trajectory_cmd_id_ = cmd_queue_.enqueue(
        CommandType::TRAJECTORY,
        {{"waypoints", std::to_string(msg->poses.size())}});
      RCLCPP_INFO(this->get_logger(),
        "📋 [ID=%lu] Comando TRAJECTORY enfileirado (%zu WPs)",
        *trajectory_cmd_id_, msg->poses.size());

      controlador_ativo_ = true;
      pouso_em_andamento_ = false;
      state_voo_ = 3; // ESTADO 3: TRAJETÓRIA

      RCLCPP_INFO(this->get_logger(), "✅ Trajetória ativada - Entrando em ESTADO 3\n");
      return;
    }
  }

  // ==========================================
  // CALLBACK: RECEBE WAYPOINT ÚNICO (PoseStamped)
  // ==========================================
  /**
   * @brief Accept a single PoseStamped waypoint goal.
   *
   * Validates the incoming position before accepting it.
   */
  void waypoint_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Quando recebe waypoint 3D, limpa o modo 4D
    using_4d_goal_ = false;
    // ✅ VALIDAÇÃO: Rejeita coordenadas inválidas antes de qualquer uso
    if (!validate_waypoint(*msg, config_)) {
      RCLCPP_WARN(this->get_logger(),
        "❌ waypoint_goal inválido (NaN/Inf ou fora dos limites físicos) - ignorado");
      return;
    }

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    last_z_ = z;

    // DETECTA POUSO (só em voo: ESTADO 2 ou 3)
    if ((state_voo_ == 2 || state_voo_ == 3) && z < config_.land_z_threshold) {
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4;
      land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(z)}});
      RCLCPP_WARN(this->get_logger(),
        "🛬 [ID=%lu] POUSO DETECTADO! Z = %.2f m - Comando LAND enfileirado",
        *land_cmd_id_, z);
      // Salva ponto de ancoragem para Mode A (ground hold durante e após pouso)
      ground_hold_x_ = x;
      ground_hold_y_ = y;
      ground_hold_z_ = std::max(0.01, config_.land_z_threshold);
      RCLCPP_WARN(this->get_logger(),
        "📍 Ancoragem de pouso definida: X=%.2f, Y=%.2f, Z=%.3f",
        ground_hold_x_, ground_hold_y_, ground_hold_z_);
      return;
    }

    // ESTADO 4: POUSO EM ANDAMENTO
    // Nunca atualiza last_waypoint_goal_ durante pouso, independente do estado de arme
    if (state_voo_ == 4) {
      if (!current_state_.armed) {
        RCLCPP_INFO(this->get_logger(), "✅ DRONE DESARMADO! Pronto para novo ciclo");

        // RESETAR FLAGS PARA NOVO CICLO
        offboard_activated_ = false;
        activation_confirmed_ = false;
        state_voo_ = 0;
        takeoff_counter_ = 0;
        pouso_em_andamento_ = false;
      }
      return;
    }

    // ESTADO 5: STANDBY NO CHÃO (Modo A)
    // Aceita waypoints de voo (z >= land_z_threshold) para iniciar novo ciclo
    if (state_voo_ == 5) {
      if (z >= config_.land_z_threshold) {
        RCLCPP_INFO(this->get_logger(),
          "\n📍 [ESTADO 5] WAYPOINT DE VÔO RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f – saindo do standby\n",
          x, y, z);
        last_waypoint_goal_ = *msg;
        waypoint_goal_received_ = true;
        pouso_em_andamento_ = false;
        controlador_ativo_ = false;
        trajectory_started_ = false;
        trajectory_waypoints_.clear();
        current_waypoint_idx_ = 0;

        // Se já estiver OFFBOARD+ARM, reutiliza a ativação
        if (current_state_.armed && current_state_.mode == "OFFBOARD") {
          RCLCPP_INFO(this->get_logger(),
            "🔋 [ESTADO 5] Já OFFBOARD+ARM – iniciando decolagem diretamente...");
          // offboard_activated_ e activation_confirmed_ já estão ok
        } else {
          RCLCPP_INFO(this->get_logger(),
            "🔋 [ESTADO 5] Reativando OFFBOARD+ARM para decolagem...");
          offboard_activated_ = false;
          activation_confirmed_ = false;
          request_offboard();
          request_arm();
          offboard_activated_ = true;
          activation_time_ = this->now();
        }

        takeoff_cmd_id_ = cmd_queue_.enqueue(
          CommandType::TAKEOFF,
          {{"x", std::to_string(x)},
           {"y", std::to_string(y)},
           {"z", std::to_string(config_.hover_altitude)}});
        RCLCPP_INFO(this->get_logger(),
          "📋 [ID=%lu] Comando TAKEOFF enfileirado (saída do estado 5)", *takeoff_cmd_id_);

        state_voo_ = 1;
        takeoff_counter_ = 0;
      } else {
        // Waypoint de pouso recebido em estado 5: ignora (já no chão)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "ℹ️ [ESTADO 5] Waypoint de pouso ignorado (drone já no chão)");
      }
      return;
    }

    // Em ESTADO 3 (trajetória), armazena setpoints relativos em trajectory_setpoint_
    // para que last_waypoint_goal_ permaneça como posição de hover (offset)
    if (state_voo_ == 3) {
      trajectory_setpoint_[0] = x;
      trajectory_setpoint_[1] = y;
      trajectory_setpoint_[2] = z;
      controlador_ativo_ = true;
      pouso_em_andamento_ = false;
      return;
    }

    // NOVO WAYPOINT RECEBIDO (aceita mesmo que repetido)
    RCLCPP_INFO(this->get_logger(), "\n📍 NOVO WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

    last_waypoint_goal_ = *msg;
    waypoint_goal_received_ = true;

    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
  }

  // ==========================================
  // CALLBACK: WAYPOINT 4D ÚNICO (Waypoint4D)
  // Suporta posição + yaw absoluto opcional.
  // yaw=NaN → mantém yaw atual do drone.
  // ==========================================
  void waypoint_goal_4d_callback(const drone_control::msg::Waypoint4D::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Valida posição
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = msg->pose;
    if (!validate_waypoint(ps, config_)) {
      RCLCPP_WARN(this->get_logger(),
        "❌ waypoint_goal_4d inválido (NaN/Inf ou fora dos limites físicos) - ignorado");
      return;
    }

    // Determina o yaw alvo: NaN → usa yaw atual; caso contrário normaliza para [-pi, pi]
    if (std::isnan(static_cast<double>(msg->yaw))) {
      goal_yaw_rad_ = current_yaw_rad_;
      RCLCPP_INFO(this->get_logger(),
        "📍 4D Waypoint: yaw=NaN → usando yaw atual (%.3f rad)", goal_yaw_rad_);
    } else {
      double raw_yaw = static_cast<double>(msg->yaw);
      goal_yaw_rad_ = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));
      RCLCPP_INFO(this->get_logger(),
        "📍 4D Waypoint: yaw=%.3f rad (normalizado para %.3f rad)", raw_yaw, goal_yaw_rad_);
    }

    using_4d_goal_ = true;
    trajectory_4d_mode_ = false;

    // Delegate a lógica de estado ao waypoint_goal_callback existente via PoseStamped
    auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_stamped->pose = msg->pose;
    pose_stamped->header.stamp = this->now();
    pose_stamped->header.frame_id = "map";

    // Extrai coordenadas e replica a lógica do waypoint_goal_callback
    double x = pose_stamped->pose.position.x;
    double y = pose_stamped->pose.position.y;
    double z = pose_stamped->pose.position.z;
    last_z_ = z;

    // Detecta pouso (só em voo)
    if ((state_voo_ == 2 || state_voo_ == 3) && z < config_.land_z_threshold) {
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4;
      land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(z)}});
      RCLCPP_WARN(this->get_logger(),
        "🛬 [ID=%lu] 4D POUSO DETECTADO! Z = %.2f m - Comando LAND enfileirado",
        *land_cmd_id_, z);
      ground_hold_x_ = x;
      ground_hold_y_ = y;
      ground_hold_z_ = std::max(0.01, config_.land_z_threshold);
      return;
    }

    // Estado 4: aguarda desarme
    if (state_voo_ == 4) {
      if (!current_state_.armed) {
        offboard_activated_ = false;
        activation_confirmed_ = false;
        state_voo_ = 0;
        takeoff_counter_ = 0;
        pouso_em_andamento_ = false;
      }
      return;
    }

    // Estado 5: aceita waypoints de voo para sair do standby
    if (state_voo_ == 5) {
      if (z >= config_.land_z_threshold) {
        RCLCPP_INFO(this->get_logger(),
          "\n📍 [ESTADO 5] 4D WAYPOINT DE VÔO: X=%.2f, Y=%.2f, Z=%.2f yaw=%.3f rad – saindo do standby\n",
          x, y, z, goal_yaw_rad_);
        last_waypoint_goal_ = *pose_stamped;
        waypoint_goal_received_ = true;
        pouso_em_andamento_ = false;
        controlador_ativo_ = false;
        trajectory_started_ = false;
        trajectory_waypoints_.clear();
        current_waypoint_idx_ = 0;
        if (current_state_.armed && current_state_.mode == "OFFBOARD") {
          // já ativado
        } else {
          offboard_activated_ = false;
          activation_confirmed_ = false;
          request_offboard();
          request_arm();
          offboard_activated_ = true;
          activation_time_ = this->now();
        }
        takeoff_cmd_id_ = cmd_queue_.enqueue(
          CommandType::TAKEOFF,
          {{"x", std::to_string(x)}, {"y", std::to_string(y)},
           {"z", std::to_string(config_.hover_altitude)}});
        RCLCPP_INFO(this->get_logger(),
          "📋 [ID=%lu] 4D Comando TAKEOFF enfileirado (saída do estado 5)", *takeoff_cmd_id_);
        state_voo_ = 1;
        takeoff_counter_ = 0;
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "ℹ️ [ESTADO 5] 4D Waypoint de pouso ignorado (drone já no chão)");
      }
      return;
    }

    // Estado 3: atualiza setpoint relativo
    if (state_voo_ == 3) {
      trajectory_setpoint_[0] = x;
      trajectory_setpoint_[1] = y;
      trajectory_setpoint_[2] = z;
      controlador_ativo_ = true;
      pouso_em_andamento_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "\n📍 4D WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f, yaw=%.3f rad",
      x, y, z, goal_yaw_rad_);

    last_waypoint_goal_ = *pose_stamped;
    waypoint_goal_received_ = true;
    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
  }

  // ==========================================
  // CALLBACK: TRAJETÓRIA 4D (Waypoint4DArray)
  // Suporta múltiplos waypoints com yaw opcional.
  // yaw=NaN em cada ponto → usa yaw atual do drone naquele instante.
  // ==========================================
  void waypoints_4d_callback(const drone_control::msg::Waypoint4DArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (msg->waypoints.size() < 1) {
      RCLCPP_WARN(this->get_logger(),
        "❌ Waypoints4D insuficientes: %zu", msg->waypoints.size());
      return;
    }

    // Valida cada waypoint
    for (size_t i = 0; i < msg->waypoints.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose = msg->waypoints[i].pose;
      if (!validate_waypoint(ps, config_)) {
        RCLCPP_WARN(this->get_logger(),
          "❌ Waypoint4D[%zu] inválido (NaN/Inf ou fora dos limites físicos) - mensagem ignorada", i);
        return;
      }
    }

    // Extrai poses e yaws.
    // Nota: yaw=NaN é resolvido no momento da recepção da mensagem,
    // usando current_yaw_rad_ (heading atual do drone). Todos os waypoints NaN
    // da mesma trajetória receberão o mesmo heading de referência.
    std::vector<geometry_msgs::msg::Pose> poses;
    std::vector<double> yaws;
    poses.reserve(msg->waypoints.size());
    yaws.reserve(msg->waypoints.size());
    for (const auto & wp : msg->waypoints) {
      poses.push_back(wp.pose);
      if (std::isnan(static_cast<double>(wp.yaw))) {
        yaws.push_back(current_yaw_rad_);  // mantém yaw atual do drone
      } else {
        double raw_yaw = static_cast<double>(wp.yaw);
        yaws.push_back(std::atan2(std::sin(raw_yaw), std::cos(raw_yaw)));
      }
    }

    double last_z = poses.back().position.z;

    // Detecta pouso (1 waypoint com Z baixo)
    if (msg->waypoints.size() == 1 && last_z < config_.land_z_threshold) {
      if (state_voo_ == 5) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "ℹ️ 4D Comando de pouso ignorado: drone já em standby no chão (estado 5)");
        return;
      }
      RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 4D POUSO DETECTADO! Z_final = %.2f m", last_z);
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4;
      land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(last_z)}});
      ground_hold_x_ = poses[0].position.x;
      ground_hold_y_ = poses[0].position.y;
      ground_hold_z_ = std::max(0.01, config_.land_z_threshold);
      trajectory_4d_mode_ = false;
      return;
    }

    // 1 waypoint de voo = levantamento
    if (msg->waypoints.size() == 1) {
      RCLCPP_INFO(this->get_logger(), "\n⬆️ 4D WAYPOINT DE LEVANTAMENTO recebido: X=%.2f Y=%.2f Z=%.2f yaw=%.3f rad",
        poses[0].position.x, poses[0].position.y, poses[0].position.z, yaws[0]);

      goal_yaw_rad_ = yaws[0];
      using_4d_goal_ = true;
      trajectory_4d_mode_ = false;

      geometry_msgs::msg::PoseStamped ps;
      ps.pose = poses[0];
      last_waypoint_goal_ = ps;

      pouso_em_andamento_ = false;
      controlador_ativo_ = false;
      trajectory_started_ = false;
      trajectory_waypoints_.clear();
      trajectory_yaws_.clear();
      current_waypoint_idx_ = 0;

      if (state_voo_ == 5 && current_state_.armed && current_state_.mode == "OFFBOARD") {
        // reutiliza ativação
      } else {
        offboard_activated_ = false;
        activation_confirmed_ = false;
        request_offboard();
        request_arm();
        offboard_activated_ = true;
        activation_time_ = this->now();
      }
      takeoff_cmd_id_ = cmd_queue_.enqueue(
        CommandType::TAKEOFF,
        {{"x", std::to_string(poses[0].position.x)},
         {"y", std::to_string(poses[0].position.y)},
         {"z", std::to_string(config_.hover_altitude)}});
      state_voo_ = 1;
      takeoff_counter_ = 0;
      return;
    }

    // 2+ waypoints = trajetória 4D
    if (state_voo_ == 4 || state_voo_ == 5) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ Ignorando trajetória 4D durante pouso/standby (estado %d)", state_voo_);
      return;
    }

    trajectory_waypoints_ = poses;
    trajectory_yaws_ = yaws;
    trajectory_4d_mode_ = true;
    using_4d_goal_ = false;
    current_waypoint_idx_ = 0;
    trajectory_started_ = false;

    geometry_msgs::msg::PoseStamped ps;
    ps.pose = poses[0];
    last_waypoint_goal_ = ps;

    RCLCPP_INFO(this->get_logger(),
      "✈️ 4D WAYPOINTS DE TRAJETÓRIA armazenados: %zu pontos", msg->waypoints.size());
    for (size_t i = 0; i < msg->waypoints.size(); i++) {
      RCLCPP_INFO(this->get_logger(),
        "   WP4D[%zu]: X=%.2f Y=%.2f Z=%.2f yaw=%.3f rad",
        i, poses[i].position.x, poses[i].position.y, poses[i].position.z, yaws[i]);
    }

    if (state_voo_ != 2) {
      RCLCPP_INFO(this->get_logger(),
        "⏸️ Trajetória 4D armazenada - Será ativada quando drone chegar em HOVER (ESTADO 2)");
      controlador_ativo_ = false;
      pouso_em_andamento_ = false;
      return;
    }

    // Em HOVER: ativa imediatamente
    RCLCPP_INFO(this->get_logger(), "\n✅ TRAJETÓRIA 4D ACEITA E ATIVADA! Drone em HOVER pronto!\n");
    if (hover_cmd_id_) {
      cmd_queue_.confirm(*hover_cmd_id_, true);
      hover_cmd_id_.reset();
    }
    trajectory_cmd_id_ = cmd_queue_.enqueue(
      CommandType::TRAJECTORY, {{"waypoints", std::to_string(msg->waypoints.size())}});
    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
    state_voo_ = 3;
    RCLCPP_INFO(this->get_logger(), "✅ Trajetória 4D ativada - Entrando em ESTADO 3\n");
  }
  void publishPositionTarget(double x, double y, double z, double yaw_rate, uint16_t type_mask) {
    mavros_msgs::msg::PositionTarget pt;
    pt.header.stamp = this->now();
    pt.header.frame_id = "map";
    pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    pt.type_mask = type_mask;
    pt.position.x = static_cast<float>(x);
    pt.position.y = static_cast<float>(y);
    pt.position.z = static_cast<float>(z);
    pt.yaw_rate = static_cast<float>(yaw_rate);
    raw_pub_->publish(pt);
  }

  // Publish a position setpoint with absolute yaw (MASK_POS_YAW: ignores vel/acc/yaw_rate).
  void publishPositionTargetWithYaw(double x, double y, double z, double yaw_rad) {
    mavros_msgs::msg::PositionTarget pt;
    pt.header.stamp = this->now();
    pt.header.frame_id = "map";
    pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    pt.type_mask = MASK_POS_YAW;
    pt.position.x = static_cast<float>(x);
    pt.position.y = static_cast<float>(y);
    pt.position.z = static_cast<float>(z);
    pt.yaw = static_cast<float>(yaw_rad);
    raw_pub_->publish(pt);
  }

  // ==========================================
  // LOOP PRINCIPAL DE CONTROLE
  // ==========================================
  void control_loop() {
    std::lock_guard<std::mutex> lock(mutex_);

    cycle_count_++;

    // Quando disabled, não publica setpoints e não avança a FSM
    if (!enabled_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "🚫 Controller desabilitado (enabled=false): setpoints pausados");
      return;
    }

    // ── Watchdog: yaw override timeout ─────────────────────────────────────
    if (yaw_override_enabled_) {
      double since_last = (this->now() - last_yaw_cmd_time_).seconds();
      if (since_last > yaw_override_timeout_s_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "⚠️ Yaw override timeout (%.2fs sem cmd): desativando override e zerando yaw_rate.", since_last);
        yaw_override_enabled_ = false;
        yaw_rate_cmd_ = 0.0;
      }
    }

    // ── Yaw override ativo: FSM congelada, publica hold + yaw_rate ─────────
    if (yaw_override_enabled_ && hold_valid_) {
      publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, yaw_rate_cmd_, MASK_POS_YAWRATE);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "🔒 Yaw override ativo: hold X=%.2f Y=%.2f Z=%.2f  yaw_rate=%.3f rad/s",
        hold_x_ned_, hold_y_ned_, hold_z_ned_, yaw_rate_cmd_);
      return;  // FSM congelada enquanto override ativo
    }

    // Override externo ativo (via parâmetro): FSM congelada, mas continua publicando
    // setpoint de hold para manter OFFBOARD ativo.
    if (override_active_) {
      if (hold_valid_) {
        publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, 0.0, MASK_POS_ONLY);
      } else {
        publishPositionTarget(current_x_ned_, current_y_ned_, current_z_ned_, 0.0, MASK_POS_ONLY);
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "🔒 Override externo ativo (override_active=true): FSM congelada, publicando hold setpoint");
      return;
    }

    // Verificação periódica de timeouts e salvamento de log (a cada ~10 segundos)
    if (cycle_count_ % 1000 == 0) {
      auto timed_out = cmd_queue_.check_timeouts(config_.command_timeout);
      for (auto id : timed_out) {
        RCLCPP_WARN(this->get_logger(),
          "⏰ [ID=%lu] Comando TIMEOUT! (>%.0f s sem confirmação)", id, config_.command_timeout);
      }
      cmd_queue_.save_log("/tmp/drone_commands.log");
    }

    // ==========================================
    // ESTADO 0: AGUARDANDO NOVO WAYPOINT
    // ==========================================
    if (state_voo_ == 0) {

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "⏳ Aguardando novo comando de waypoint para decolar...");
      // Não faz nada! Apenas aguarda novo waypoint em waypoints_callback()
    }

    // ==========================================
    // ESTADO 1: DECOLAGEM
    // ==========================================
    else if (state_voo_ == 1) {

      // ──────────────────────────────────────────
      // CAMADA 1: SOLICITAR OFFBOARD+ARM
      // ──────────────────────────────────────────
      if (!offboard_activated_) {
        RCLCPP_INFO(this->get_logger(), "📡 Solicitando OFFBOARD+ARM...");
        request_offboard();
        request_arm();
        offboard_activated_ = true;
        activation_time_ = this->now();
        return;  // Aguarda próximo ciclo
      }

      // ──────────────────────────────────────────
      // CAMADA 2: VERIFICAR CONFIRMAÇÃO DO FCU
      // ──────────────────────────────────────────
      if (!activation_confirmed_) {
        if (current_state_.armed && current_state_.mode == "OFFBOARD") {
          RCLCPP_INFO(this->get_logger(),
            "✅ OFFBOARD+ARM CONFIRMADOS! Iniciando decolagem...");
          activation_confirmed_ = true;
          takeoff_counter_ = 0;
        } else if ((this->now() - activation_time_).seconds() > config_.activation_timeout) {
          // Timeout: tentar novamente
          RCLCPP_WARN(this->get_logger(),
            "⚠️ Timeout ativação OFFBOARD+ARM (%.0f s)! Tentando novamente...",
            config_.activation_timeout);
          RCLCPP_WARN(this->get_logger(),
            "   Estado: armed=%d | mode=%s",
            current_state_.armed,
            current_state_.mode.c_str());
          offboard_activated_ = false;
          activation_confirmed_ = false;
          takeoff_counter_ = 0;
          return;
        } else {
          // Ainda aguardando confirmação
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "⏳ Aguardando OFFBOARD+ARM... | armed=%d | mode=%s",
            current_state_.armed,
            current_state_.mode.c_str());
          takeoff_counter_++;
          return;  // NÃO publica até confirmar
        }
      }

      // ──────────────────────────────────────────
      // CAMADA 3: PUBLICAR SETPOINT DE DECOLAGEM
      // ──────────────────────────────────────────
      // Publica setpoint de decolagem usando Z do último waypoint recebido
      double target_altitude = last_waypoint_goal_.pose.position.z;
      if (using_4d_goal_) {
        publishPositionTargetWithYaw(
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y,
          target_altitude,
          goal_yaw_rad_);
      } else {
        publishPositionTarget(
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y,
          target_altitude,
          0.0, MASK_POS_ONLY);
      }

      takeoff_counter_++;

      if (takeoff_counter_ == 1) {
        RCLCPP_INFO(this->get_logger(), "⬆️ Decolando para %.1f metros...", target_altitude);
        RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.1f",
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y,
          target_altitude);
      }

      // Log de progresso a cada 100 ciclos (1 segundo @ 100 Hz)
      if (takeoff_counter_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "📈 Decolando... Z_alvo=%.1fm | Z_real=%.2fm | Tempo=%.1fs",
          target_altitude,
          current_z_real_,
          (double)takeoff_counter_ / 100.0);
      }

      // Verifica se drone chegou na altitude alvo usando odometria real.
      // Margem de hover_altitude_margin abaixo do alvo para segurança.
      double arrival_threshold = target_altitude - config_.hover_altitude_margin;
      if (current_z_real_ >= arrival_threshold) {
        RCLCPP_INFO(this->get_logger(),
          "✅ Decolagem concluída! Altitude = %.2fm\n", current_z_real_);
        // Confirma comando TAKEOFF
        if (takeoff_cmd_id_) {
          cmd_queue_.confirm(*takeoff_cmd_id_, true);
          RCLCPP_INFO(this->get_logger(),
            "✅ [ID=%lu] TAKEOFF confirmado! Altitude=%.2fm", *takeoff_cmd_id_, current_z_real_);
          takeoff_cmd_id_.reset();
        }
        // Inicia comando HOVER
        hover_cmd_id_ = cmd_queue_.enqueue(
          CommandType::HOVER,
          {{"x", std::to_string(last_waypoint_goal_.pose.position.x)},
           {"y", std::to_string(last_waypoint_goal_.pose.position.y)},
           {"z", std::to_string(target_altitude)}});
        RCLCPP_INFO(this->get_logger(),
          "📋 [ID=%lu] Comando HOVER enfileirado", *hover_cmd_id_);
        state_voo_ = 2;
        takeoff_counter_ = 0;
        return;
      }
    }

    // ==========================================
    // ESTADO 2: HOVER/AGUARDANDO TRAJETÓRIA
    // ==========================================
    else if (state_voo_ == 2) {

      // Publica setpoint em hover usando Z do último waypoint recebido
      if (using_4d_goal_) {
        publishPositionTargetWithYaw(
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y,
          last_waypoint_goal_.pose.position.z,
          goal_yaw_rad_);
      } else {
        publishPositionTarget(
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y,
          last_waypoint_goal_.pose.position.z,
          0.0, MASK_POS_ONLY);
      }

      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "🛸 Em HOVER (%.1fm) | Posição: X=%.2f, Y=%.2f | Aguardando waypoints... Controlador: %s",
          last_waypoint_goal_.pose.position.z,
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y,
          controlador_ativo_ ? "ATIVO" : "INATIVO");
      }

      // Quando recebe waypoints válidos, vai para estado 3
      if (controlador_ativo_) {
        state_voo_ = 3;
        RCLCPP_INFO(this->get_logger(), "✈️ Iniciando execução de trajetória...\n");
      }

      // SE DETECTAR POUSO NESTE ESTADO
      if (pouso_em_andamento_) {
        RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO NO HOVER - DESLIGANDO!");
        state_voo_ = 4;
        return;
      }
    }

    // ==========================================
    // ESTADO 3: EXECUTANDO TRAJETÓRIA
    // ==========================================
    else if (state_voo_ == 3) {

      // NOVO! Detectar pouso durante trajetória (Z real < land_z_threshold)
      if (current_z_real_ < config_.land_z_threshold) {
        RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO DURANTE TRAJETÓRIA! Z = %.2f m", current_z_real_);
        // Marca TRAJECTORY como falha (interrompida por pouso)
        if (trajectory_cmd_id_) {
          cmd_queue_.confirm(*trajectory_cmd_id_, false);
          RCLCPP_WARN(this->get_logger(),
            "⚠️ [ID=%lu] TRAJECTORY interrompida por pouso", *trajectory_cmd_id_);
          trajectory_cmd_id_.reset();
        }
        // Enfileira LAND se não enfileirado
        if (!land_cmd_id_) {
          land_cmd_id_ = cmd_queue_.enqueue(
            CommandType::LAND, {{"z", std::to_string(current_z_real_)}});
          RCLCPP_WARN(this->get_logger(),
            "📋 [ID=%lu] Comando LAND enfileirado (pouso durante trajetória)", *land_cmd_id_);
        }
        pouso_em_andamento_ = true;
        controlador_ativo_ = false;
        state_voo_ = 4;
        return;
      }

      // Detectar pouso automaticamente
      if (pouso_em_andamento_ && !controlador_ativo_) {
        RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO EM TRAJETÓRIA - PARANDO IMEDIATAMENTE!");
        state_voo_ = 4;
        return;
      }

      // INICIALIZAR trajetória na primeira execução
      if (!trajectory_started_) {
        if (trajectory_waypoints_.empty()) {
          RCLCPP_WARN(this->get_logger(), "⚠️ Nenhum waypoint armazenado, voltando para ESTADO 2");
          state_voo_ = 2;
          return;
        }

        trajectory_start_time_ = this->now();
        trajectory_started_ = true;
        current_waypoint_idx_ = 0;

        RCLCPP_INFO(this->get_logger(), "✈️ Trajetória iniciada! %zu waypoints | %.1f segundos cada",
          trajectory_waypoints_.size(), waypoint_duration_);
      }

      // CALCULAR qual waypoint publicar baseado no tempo transcorrido
      double elapsed_time = (this->now() - trajectory_start_time_).seconds();
      int computed_idx = static_cast<int>(elapsed_time / waypoint_duration_);

      // Limitar ao índice máximo (não ultrapassar último waypoint)
      // Boundary-check: trajectory_waypoints_ is guaranteed non-empty here
      // (checked above via trajectory_waypoints_.empty()).
      current_waypoint_idx_ = std::min(
        computed_idx,
        static_cast<int>(trajectory_waypoints_.size()) - 1);

      // Explicit bounds guard to prevent out-of-range access.
      if (current_waypoint_idx_ < 0 ||
          static_cast<size_t>(current_waypoint_idx_) >= trajectory_waypoints_.size()) {
        RCLCPP_ERROR(this->get_logger(),
          "❌ Índice de waypoint inválido: %d (tamanho=%zu)",
          current_waypoint_idx_, trajectory_waypoints_.size());
        state_voo_ = 2;
        return;
      }

      // PUBLICAR waypoint ATUAL
      const auto & current_waypoint = trajectory_waypoints_[current_waypoint_idx_];

      if (trajectory_4d_mode_ &&
          static_cast<size_t>(current_waypoint_idx_) < trajectory_yaws_.size()) {
        publishPositionTargetWithYaw(
          current_waypoint.position.x,
          current_waypoint.position.y,
          current_waypoint.position.z,
          trajectory_yaws_[current_waypoint_idx_]);
      } else {
        publishPositionTarget(
          current_waypoint.position.x,
          current_waypoint.position.y,
          current_waypoint.position.z,
          0.0, MASK_POS_ONLY);
      }

      // LOG: Mostrar progresso da trajetória
      if (cycle_count_ % 500 == 0) {
        double total_time = waypoint_duration_ * (double)trajectory_waypoints_.size();
        double progress_pct = (elapsed_time / total_time) * 100.0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "📡 Trajetória em execução | WP[%d/%zu]: X=%.2fm, Y=%.2fm, Z=%.2fm (Z_real=%.2f) | %.1f%% concluído",
          current_waypoint_idx_,
          trajectory_waypoints_.size() - 1,
          current_waypoint.position.x,
          current_waypoint.position.y,
          current_waypoint.position.z,
          current_z_real_,
          progress_pct);
      }

      // Confirma TRAJECTORY quando todos os waypoints foram visitados
      {
        double total_time = waypoint_duration_ * (double)trajectory_waypoints_.size();
        if (elapsed_time >= total_time && trajectory_cmd_id_) {
          cmd_queue_.confirm(*trajectory_cmd_id_, true);
          RCLCPP_INFO(this->get_logger(),
            "✅ [ID=%lu] TRAJECTORY confirmada - todos os waypoints visitados", *trajectory_cmd_id_);
          trajectory_cmd_id_.reset();
        }
      }
    }

    // ==========================================
    // ESTADO 4: POUSO/PAUSADO - NÃO PUBLICA
    // ==========================================
    else if (state_voo_ == 4) {
      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "🛑 CONTROLADOR PAUSADO | drone_soft_land fazendo pouso...");
      }

      // TIMEOUT: Uma vez que pouso foi detectado, aguarda landing_timeout
      // segundos INDEPENDENTE de last_z_
      if (pouso_em_andamento_) {
        if (!pouso_start_time_set_) {
          pouso_start_time_ = this->now();
          pouso_start_time_set_ = true;
          RCLCPP_INFO(this->get_logger(),
            "⏱️ Iniciando contagem de pouso (%.0f s para confirmar)...", config_.landing_timeout);
        }

        // Se passou landing_timeout segundos desde que pouso foi detectado
        if ((this->now() - pouso_start_time_).seconds() > config_.landing_timeout) {

          if (landing_mode_ == 0) {
            // ── MODO A: Standby no chão ──────────────────────────────────
            // Confirma comando LAND sem desarmar
            if (land_cmd_id_) {
              cmd_queue_.confirm(*land_cmd_id_, true);
              RCLCPP_WARN(this->get_logger(),
                "✅ [ID=%lu] LAND confirmado (Modo A - sem DISARM)", *land_cmd_id_);
              land_cmd_id_.reset();
            }
            // Salva log persistente do histórico de comandos
            cmd_queue_.save_log("/tmp/drone_commands.log");
            RCLCPP_INFO(this->get_logger(),
              "💾 Histórico de comandos salvo em /tmp/drone_commands.log");

            RCLCPP_WARN(this->get_logger(),
              "\n✅ POUSO CONCLUÍDO (MODO A): mantendo OFFBOARD+ARM e segurando no chão.\n"
              "   Drone permanece armado em estado 5 (standby no chão).\n");

            // ground_hold_x_/y_/z_ já foram definidos no callback de detecção de pouso;
            // garantir z mínimo seguro como fallback
            ground_hold_z_ = std::max(0.01, config_.land_z_threshold);

            // Limpa flags de trajetória/execução mas MANTÉM offboard_activated_ e activation_confirmed_
            pouso_em_andamento_ = false;
            controlador_ativo_ = false;
            trajectory_started_ = false;
            pouso_start_time_set_ = false;
            takeoff_counter_ = 0;
            trajectory_waypoints_.clear();
            current_waypoint_idx_ = 0;
            takeoff_cmd_id_.reset();
            hover_cmd_id_.reset();
            trajectory_cmd_id_.reset();

            // Transiciona para estado 5: standby no chão
            state_voo_ = 5;

            RCLCPP_WARN(this->get_logger(), "🔍 DEBUG MODO A (estado 5):");
            RCLCPP_WARN(this->get_logger(), "   offboard_activated_=%d (mantido)", offboard_activated_);
            RCLCPP_WARN(this->get_logger(), "   activation_confirmed_=%d (mantido)", activation_confirmed_);
            RCLCPP_WARN(this->get_logger(), "   ground_hold=(%.2f, %.2f, %.2f)",
              ground_hold_x_, ground_hold_y_, ground_hold_z_);

          } else {
            // ── MODO B: Desligar/Desarmar (comportamento atual) ──────────
            RCLCPP_WARN(this->get_logger(), "🔌 Solicitando DISARM...");

            // DISARM
            request_disarm();

            // Confirma comando LAND
            if (land_cmd_id_) {
              cmd_queue_.confirm(*land_cmd_id_, true);
              RCLCPP_WARN(this->get_logger(),
                "✅ [ID=%lu] LAND confirmado - pouso concluído", *land_cmd_id_);
              land_cmd_id_.reset();
            }
            // Salva log persistente do histórico de comandos
            cmd_queue_.save_log("/tmp/drone_commands.log");
            RCLCPP_INFO(this->get_logger(),
              "💾 Histórico de comandos salvo em /tmp/drone_commands.log");

            RCLCPP_WARN(this->get_logger(),
              "\n✅ POUSO CONCLUÍDO! Aguardando novo comando de waypoint para decolar novamente...\n");

            // Resetar TODAS as flags para estado limpo
            // CRUCIAL: offboard_activated_ DEVE ser false para próxima decolagem!
            state_voo_ = 0;
            pouso_em_andamento_ = false;
            controlador_ativo_ = false;
            trajectory_started_ = false;
            pouso_start_time_set_ = false;
            offboard_activated_ = false;
            activation_confirmed_ = false;
            takeoff_counter_ = 0;
            trajectory_waypoints_.clear();
            current_waypoint_idx_ = 0;
            takeoff_cmd_id_.reset();
            hover_cmd_id_.reset();
            trajectory_cmd_id_.reset();
            land_cmd_id_.reset();

            // Log de verificação
            RCLCPP_WARN(this->get_logger(), "🔍 DEBUG RESET EM ESTADO 4:");
            RCLCPP_WARN(this->get_logger(), "   offboard_activated_=%d (deve ser 0)", offboard_activated_);
            RCLCPP_WARN(this->get_logger(), "   state_voo_=%d (deve ser 0)", state_voo_);
          }

          return;
        }
      }

      // Modo A: publica setpoint de ancoragem para manter OFFBOARD ativo durante pouso
      if (landing_mode_ == 0) {
        publishPositionTarget(ground_hold_x_, ground_hold_y_, ground_hold_z_, 0.0, MASK_POS_ONLY);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
          "🛬 [MODO A] Publicando setpoint de ancoragem durante pouso: X=%.2f, Y=%.2f, Z=%.3f",
          ground_hold_x_, ground_hold_y_, ground_hold_z_);
      }
      // Modo B: não publica nada durante pouso (comportamento original)
      return;
    }

    // ==========================================
    // ESTADO 5: STANDBY NO CHÃO (MODO A)
    // Mantém OFFBOARD+ARM, publica setpoint de posição com Z baixo
    // para o PX4 segurar o drone no solo sem desarmar os motores.
    // Sai deste estado ao receber novo waypoint de voo (Z >= land_z_threshold).
    // ==========================================
    else if (state_voo_ == 5) {
      // Verifica se ainda está armado e em OFFBOARD
      if (!current_state_.armed || current_state_.mode != "OFFBOARD") {
        // Taxa de recuperação: tenta re-solicitar OFFBOARD+ARM a cada activation_timeout segundos
        bool should_request = (this->now() - state5_recovery_time_).seconds() > config_.activation_timeout;
        if (should_request) {
          RCLCPP_WARN(this->get_logger(),
            "⚠️ Estado 5: não está armado/OFFBOARD (armed=%d mode=%s). Tentando reativar...",
            current_state_.armed, current_state_.mode.c_str());
          request_offboard();
          request_arm();
          state5_recovery_time_ = this->now();
        }
        return;
      }

      // Publica setpoint de posição baixa para segurar o drone no solo
      publishPositionTarget(ground_hold_x_, ground_hold_y_, ground_hold_z_, 0.0, MASK_POS_ONLY);

      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "🟢 STANDBY NO CHÃO | Setpoint: X=%.2f, Y=%.2f, Z=%.3f | Aguardando novo waypoint...",
          ground_hold_x_, ground_hold_y_, ground_hold_z_);
      }
    }
  }

  // ==========================================
  // MEMBROS PRIVADOS
  // ==========================================

  // Configuração com todos os parâmetros nomeados (sem magic numbers)
  DroneConfig config_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_pub_;

  // Subscribers
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<drone_control::msg::YawOverride>::SharedPtr yaw_override_sub_;
  rclcpp::Subscription<drone_control::msg::Waypoint4D>::SharedPtr waypoint_goal_4d_sub_;
  rclcpp::Subscription<drone_control::msg::Waypoint4DArray>::SharedPtr waypoints_4d_sub_;

  // Service Clients - ATIVAÇÃO OFFBOARD + ARM
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Estado do drone
  mavros_msgs::msg::State current_state_;

  // Estado do controlador - MÁQUINA DE ESTADOS
  int state_voo_;  // 0=aguardando waypoint, 1=decolagem, 2=hover,
                   // 3=trajetória, 4=pouso, 5=standby no chão (Modo A)
  bool controlador_ativo_;           // Trajetória está ativa?
  bool pouso_em_andamento_;          // Pouso em andamento?
  bool offboard_activated_;          // Ativação OFFBOARD+ARM foi disparada por waypoints?
  bool activation_confirmed_;        // OFFBOARD+ARM confirmados?
  rclcpp::Time activation_time_;     // Timestamp da última solicitação de ativação
  int cycle_count_;                  // Contador de ciclos
  int takeoff_counter_;              // Contador de decolagem

  // Timeout de pouso
  double last_z_;                    // Última posição Z recebida
  rclcpp::Time pouso_start_time_;    // Timestamp quando pouso começou
  bool pouso_start_time_set_;        // Flag para saber se iniciou contagem

  // Rastreamento do último waypoint único recebido
  geometry_msgs::msg::PoseStamped last_waypoint_goal_;
  bool waypoint_goal_received_;      // Recebeu ao menos um waypoint_goal?

  // Setpoints da trajetória (coordenadas relativas ao ponto de hover)
  double trajectory_setpoint_[3];

  // Waypoints da trajetória
  std::vector<geometry_msgs::msg::Pose> trajectory_waypoints_;
  rclcpp::Time trajectory_start_time_;   // Quando trajetória começou
  bool trajectory_started_;              // Trajetória já iniciou?
  int current_waypoint_idx_;             // Qual waypoint estamos
  double waypoint_duration_;             // Tempo em cada waypoint (segundos)

  // Posição Z real do drone (atualizada por odometria)
  double current_z_real_;

  // Posição NED completa do drone (atualizada por odometria)
  double current_x_ned_;
  double current_y_ned_;
  double current_z_ned_;

  // ── Yaw override via tópico /uav1/yaw_override/cmd ────────────────────
  bool yaw_override_enabled_;            ///< true quando override de yaw ativo
  double yaw_rate_cmd_;                  ///< yaw_rate a injetar (rad/s)
  double yaw_override_timeout_s_;        ///< timeout sem cmd para desativar override
  rclcpp::Time last_yaw_cmd_time_;       ///< timestamp do último cmd de yaw recebido
  bool hold_valid_;                      ///< posição de hold capturada e válida?
  double hold_x_ned_;                    ///< X de hold (NED)
  double hold_y_ned_;                    ///< Y de hold (NED)
  double hold_z_ned_;                    ///< Z de hold (NED, valor bruto do odom)

  // ── Suporte a waypoints 4D (posição + yaw absoluto) ───────────────────
  double current_yaw_rad_;              ///< yaw atual do drone (rad, extraído do odom)
  double goal_yaw_rad_;                 ///< yaw alvo do waypoint 4D ativo (rad, [-pi,pi])
  bool using_4d_goal_;                  ///< true quando o último goal veio de /waypoint_goal_4d
  bool trajectory_4d_mode_;             ///< true quando a trajetória ativa veio de /waypoints_4d
  std::vector<double> trajectory_yaws_; ///< yaw por waypoint da trajetória 4D (rad, [-pi,pi])

  // ── Modo de pouso ──────────────────────────────────────────────────────
  /// 0=Modo A (standby no chão, OFFBOARD+ARMED), 1=Modo B (DISARM, padrão)
  int landing_mode_{1};

  // ── Controle de publicação ─────────────────────────────────────────────
  /// Quando false, control_loop() não publica setpoints nem avança a FSM.
  /// Pode ser alterado em runtime via: ros2 param set /drone_controller_completo enabled false
  bool enabled_{true};

  // ── Override externo ──────────────────────────────────────────────────
  /// Quando true, um nó externo (ex: drone_yaw_360) está sinalizando override.
  /// control_loop() congela a FSM mas CONTINUA publicando hold setpoints para
  /// manter OFFBOARD ativo. Alternar via:
  ///   ros2 param set /drone_controller_completo override_active true
  bool override_active_{false};

  // Ponto de ancoragem no solo usado pelo estado 5 (Modo A)
  double ground_hold_x_{0.0};  ///< X a manter enquanto em standby no chão
  double ground_hold_y_{0.0};  ///< Y a manter enquanto em standby no chão
  double ground_hold_z_{0.01}; ///< Z baixo (>= 0.01 m) para segurar no solo

  // Timestamp da última tentativa de reativação OFFBOARD+ARM no estado 5
  rclcpp::Time state5_recovery_time_;

  // Sincronização thread-safe
  std::mutex mutex_;

  // Handle para o callback de parâmetros (deve ser mantido vivo enquanto o nó existir)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ==========================================
  // SISTEMA DE FILA DE COMANDOS
  // ==========================================
  CommandQueue cmd_queue_;              // Fila de comandos thread-safe com histórico

  // Command IDs use std::optional to eliminate the invalid-zero-sentinel
  // antipattern: an empty optional means "no command pending", and the
  // compiler enforces a has_value() check before the ID is used.
  std::optional<uint64_t> offboard_cmd_id_;    ///< ID do último comando SET_MODE_OFFBOARD
  std::optional<uint64_t> arm_cmd_id_;         ///< ID do último comando ARM
  std::optional<uint64_t> disarm_cmd_id_;      ///< ID do último comando DISARM
  std::optional<uint64_t> takeoff_cmd_id_;     ///< ID do comando TAKEOFF atual
  std::optional<uint64_t> hover_cmd_id_;       ///< ID do comando HOVER atual
  std::optional<uint64_t> trajectory_cmd_id_;  ///< ID do comando TRAJECTORY atual
  std::optional<uint64_t> land_cmd_id_;        ///< ID do comando LAND atual
};

}  // namespace drone_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_control::DroneControllerCompleto>();
  
  RCLCPP_INFO(node->get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(node->get_logger(), "║           🚁 CONTROLADOR PRONTO PARA OPERAÇÃO              ║");
  RCLCPP_INFO(node->get_logger(), "║                                                            ║");
  RCLCPP_INFO(node->get_logger(), "║  Pressione Ctrl+C para encerrar                            ║");
  RCLCPP_INFO(node->get_logger(), "╚════════════════════════════════════════════════════════════╝\n");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
