#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp"
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
 * Implements a 5-state flight state machine:
 *  - State 0: Waiting for waypoint goal
 *  - State 1: Takeoff (OFFBOARD + ARM, climb to hover_altitude)
 *  - State 2: Hover (waiting for trajectory)
 *  - State 3: Executing trajectory
 *  - State 4: Landing / paused
 *
 * All drone commands are registered in a CommandQueue so that every
 * operation can be audited and timed out automatically.
 */
class DroneControllerCompleto : public rclcpp::Node {
public:
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
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/uav1/mavros/setpoint_position/local", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /uav1/mavros/setpoint_position/local");

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

    RCLCPP_INFO(this->get_logger(), "✓ Subscribers criados: /uav1/mavros/state, /waypoints, /waypoint_goal e odometria");

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

    RCLCPP_INFO(this->get_logger(), "\n📊 STATUS INICIAL:");
    RCLCPP_INFO(this->get_logger(), "   Estado: %d (aguardando waypoint)", state_voo_);
    RCLCPP_INFO(this->get_logger(), "   Controlador: %s", controlador_ativo_ ? "ATIVO" : "INATIVO");
    RCLCPP_INFO(this->get_logger(), "   Pouso: %s\n", pouso_em_andamento_ ? "SIM" : "NÃO");
  }

private:

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
    current_z_real_ = std::abs(msg->pose.pose.position.z);
  }

  // ==========================================
  // CALLBACK: RECEBE WAYPOINTS
  // ==========================================
  void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

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
      RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z_final = %.2f m", last_z);
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4; // VAI DIRETO PARA ESTADO 4!
      land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(last_z)}});
      RCLCPP_WARN(this->get_logger(),
        "📋 [ID=%lu] Comando LAND enfileirado", *land_cmd_id_);
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

      // Força reativação, independentemente do estado anterior
      RCLCPP_INFO(this->get_logger(), "🔋 Ativando OFFBOARD+ARM para levantamento...\n");

      // Reset explícito ANTES de reativar
      offboard_activated_ = false;
      activation_confirmed_ = false;

      // Solicita OFFBOARD MODE e ARM
      request_offboard();
      request_arm();

      // Enfileira comando de TAKEOFF
      takeoff_cmd_id_ = cmd_queue_.enqueue(
        CommandType::TAKEOFF,
        {{"x", std::to_string(msg->poses[0].position.x)},
         {"y", std::to_string(msg->poses[0].position.y)},
         {"z", std::to_string(config_.hover_altitude)}});
      RCLCPP_INFO(this->get_logger(),
        "📋 [ID=%lu] Comando TAKEOFF enfileirado", *takeoff_cmd_id_);

      // Marca como ativado e aguarda confirmação do FCU
      offboard_activated_ = true;
      activation_time_ = this->now();

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

      // Se pouso em andamento (estado 4), ignora novos waypoints
      // para não corromper last_waypoint_goal_
      if (state_voo_ == 4) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Ignorando waypoints durante pouso (estado 4)");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "🔍 Trajetória (2+ waypoints) recebida");
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d (esperado 2 para ativar)", state_voo_);

      // SEMPRE armazena waypoints (mesmo se ESTADO != 2)
      trajectory_waypoints_ = msg->poses;
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
  // LOOP PRINCIPAL DE CONTROLE
  // ==========================================
  void control_loop() {
    std::lock_guard<std::mutex> lock(mutex_);

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";

    cycle_count_++;

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
      pose_msg.pose.position.x = last_waypoint_goal_.pose.position.x;
      pose_msg.pose.position.y = last_waypoint_goal_.pose.position.y;
      pose_msg.pose.position.z = target_altitude;
      pose_pub_->publish(pose_msg);

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
      pose_msg.pose.position.x = last_waypoint_goal_.pose.position.x;
      pose_msg.pose.position.y = last_waypoint_goal_.pose.position.y;
      pose_msg.pose.position.z = last_waypoint_goal_.pose.position.z;
      pose_pub_->publish(pose_msg);

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

      pose_msg.pose.position.x = current_waypoint.position.x;
      pose_msg.pose.position.y = current_waypoint.position.y;
      pose_msg.pose.position.z = current_waypoint.position.z;
      pose_msg.pose.orientation.w = 1.0;
      pose_pub_->publish(pose_msg);

      // LOG: Mostrar progresso da trajetória
      if (cycle_count_ % 500 == 0) {
        double total_time = waypoint_duration_ * (double)trajectory_waypoints_.size();
        double progress_pct = (elapsed_time / total_time) * 100.0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "📡 Trajetória em execução | WP[%d/%zu]: X=%.2fm, Y=%.2fm, Z=%.2fm (Z_real=%.2f) | %.1f%% concluído",
          current_waypoint_idx_,
          trajectory_waypoints_.size() - 1,
          pose_msg.pose.position.x,
          pose_msg.pose.position.y,
          pose_msg.pose.position.z,
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

          return;
        }
      }

      return; // NÃO publica nada durante pouso
    }
  }

  // ==========================================
  // MEMBROS PRIVADOS
  // ==========================================

  // Configuração com todos os parâmetros nomeados (sem magic numbers)
  DroneConfig config_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Subscribers
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Service Clients - ATIVAÇÃO OFFBOARD + ARM
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Estado do drone
  mavros_msgs::msg::State current_state_;

  // Estado do controlador - MÁQUINA DE ESTADOS
  int state_voo_;                    // 0=aguardando waypoint, 1=decolagem, 2=hover, 3=trajetória, 4=pouso
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

  // Sincronização thread-safe
  std::mutex mutex_;

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
