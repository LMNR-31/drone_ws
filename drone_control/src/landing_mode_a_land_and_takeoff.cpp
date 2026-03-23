#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

// ============================================================
// Nó: landing_mode_a_land_and_takeoff
//
// Configura landing_mode=0 (Modo A: mantém OFFBOARD+ARMED no
// solo após pousar) no /drone_controller_completo, publica um
// waypoint de pouso em /waypoint_goal, aguarda wait_seconds e
// publica um waypoint de decolagem.
//
// AVISO: se o nó drone_soft_land estiver rodando em paralelo
// ele pode desarmar o drone independentemente, interferindo com
// o Modo A. Certifique-se de que drone_soft_land NÃO está ativo
// ao usar este nó.
// ============================================================

class LandingModeALandAndTakeoff : public rclcpp::Node
{
public:
  LandingModeALandAndTakeoff() : Node("landing_mode_a_land_and_takeoff")
  {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🛬🅰️  MODO A: POUSO + STANDBY + DECOLAGEM         ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ==========================================
    // PARÂMETROS
    // ==========================================
    this->declare_parameter<std::string>("controller_node", "/drone_controller_completo");
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("z_land", 0.05);
    this->declare_parameter<double>("z_takeoff", 1.5);
    this->declare_parameter<double>("wait_seconds", 10.0); // mudar o tempo de repouso
    this->declare_parameter<bool>("use_current_xy", true);
    this->declare_parameter<std::string>("odom_topic", "/uav1/mavros/local_position/odom");

    controller_node_ = this->get_parameter("controller_node").as_string();
    x_ = this->get_parameter("x").as_double();
    y_ = this->get_parameter("y").as_double();
    z_land_ = this->get_parameter("z_land").as_double();
    z_takeoff_ = this->get_parameter("z_takeoff").as_double();
    wait_seconds_ = this->get_parameter("wait_seconds").as_double();
    use_current_xy_ = this->get_parameter("use_current_xy").as_bool();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "📋 Parâmetros:");
    RCLCPP_INFO(this->get_logger(), "   controller_node : %s", controller_node_.c_str());
    RCLCPP_INFO(this->get_logger(), "   use_current_xy=%s  (fallback x=%.2f, y=%.2f)",
      use_current_xy_ ? "true" : "false", x_, y_);
    RCLCPP_INFO(this->get_logger(), "   z_land=%.2f, z_takeoff=%.2f", z_land_, z_takeoff_);
    RCLCPP_INFO(this->get_logger(), "   wait_seconds=%.1f", wait_seconds_);

    // ==========================================
    // PUBLISHERS
    // ==========================================
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/waypoint_goal", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /waypoint_goal");

    // ==========================================
    // SUBSCRIBER — odometria para posição atual
    // ==========================================
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        odom_received_ = true;
      });

    RCLCPP_INFO(this->get_logger(), "✓ Subscriber criado: %s", odom_topic.c_str());

    // ==========================================
    // SERVICE CLIENT — set_parameters
    // ==========================================
    std::string service_name = controller_node_ + "/set_parameters";
    set_param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(service_name);

    RCLCPP_INFO(this->get_logger(), "✓ Service client criado: %s", service_name.c_str());

    // ==========================================
    // SERVICE CLIENT — get_parameters (verificação)
    // ==========================================
    std::string get_service_name = controller_node_ + "/get_parameters";
    get_param_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(get_service_name);

    RCLCPP_INFO(this->get_logger(), "✓ Service client criado: %s", get_service_name.c_str());

    // ==========================================
    // TIMER — execução sequencial via máquina de estados
    // ==========================================
    timer_ = this->create_wall_timer(100ms, std::bind(&LandingModeALandAndTakeoff::timerCallback, this));
  }

private:

  static constexpr int kMaxParamRetries = 5;
  static constexpr int kRetryBackoffTicks = 10;    // 10 × 100 ms = 1 s
  static constexpr int kLogThrottleInterval = 20;  // 20 × 100 ms = 2 s

  enum class State {
    WAIT_SERVICE,
    SET_PARAMETER,
    AWAITING_PARAM_ACK,
    VERIFY_PARAM,
    PUBLISH_LAND,
    WAIT_BEFORE_TAKEOFF,
    PUBLISH_TAKEOFF,
    DONE
  };

  void timerCallback()
  {
    switch (state_)
    {
    case State::WAIT_SERVICE:
      if (set_param_client_->service_is_ready()) {
        RCLCPP_INFO(this->get_logger(), "✅ Serviço set_parameters disponível");
        state_ = State::SET_PARAMETER;
      } else {
        if (wait_count_++ % kLogThrottleInterval == 0) {
          RCLCPP_INFO(this->get_logger(), "⏳ Aguardando serviço set_parameters...");
        }
      }
      break;

    case State::SET_PARAMETER:
      RCLCPP_INFO(this->get_logger(),
        "📡 [tentativa %d/%d] Iniciando set de landing_mode=0 em %s/set_parameters ...",
        param_retry_count_ + 1, kMaxParamRetries, controller_node_.c_str());
      param_ok_ = false;
      param_failed_ = false;
      param_retry_wait_ = 0;
      setLandingMode(0);
      state_ = State::AWAITING_PARAM_ACK;
      break;

    case State::AWAITING_PARAM_ACK:
      if (param_ok_) {
        RCLCPP_INFO(this->get_logger(),
          "✅ set_parameters: landing_mode=0 aceito. Verificando com get_parameters...");
        verify_ok_ = false;
        verify_failed_ = false;
        verify_wait_count_ = 0;
        verifyLandingMode(0);
        state_ = State::VERIFY_PARAM;
      } else if (param_failed_ && param_retry_wait_ == 0) {
        param_retry_count_++;
        if (param_retry_count_ >= kMaxParamRetries) {
          RCLCPP_ERROR(this->get_logger(),
            "❌ Falha ao setar landing_mode=0 após %d tentativas. Encerrando nó.", kMaxParamRetries);
          timer_->cancel();
          rclcpp::shutdown();
        } else {
          RCLCPP_WARN(this->get_logger(),
            "⚠️  Tentativa %d/%d falhou. Re-tentando em 1 s ...",
            param_retry_count_, kMaxParamRetries);
          param_failed_ = false;
          // Wait ~1 s (10 × 100 ms ticks) before retrying
          param_retry_wait_ = kRetryBackoffTicks;
        }
      } else if (!param_failed_ && !param_ok_ && param_retry_wait_ == 0) {
        // Waiting for the async response
        if (param_wait_count_++ % kLogThrottleInterval == 0) {
          RCLCPP_INFO(this->get_logger(),
            "⏳ Aguardando resposta do set_parameters...");
        }
      } else if (param_retry_wait_ > 0) {
        // Countdown before next attempt
        if (--param_retry_wait_ == 0) {
          state_ = State::SET_PARAMETER;
        }
      }
      break;

    case State::VERIFY_PARAM:
      if (verify_ok_) {
        RCLCPP_INFO(this->get_logger(),
          "✅ get_parameters confirmou: landing_mode=0 ativo no controlador. Publicando waypoint de pouso.");
        state_ = State::PUBLISH_LAND;
      } else if (verify_failed_) {
        RCLCPP_WARN(this->get_logger(),
          "⚠️  get_parameters: landing_mode não é 0 no controlador. Re-tentando set...");
        param_retry_count_++;
        if (param_retry_count_ >= kMaxParamRetries) {
          RCLCPP_ERROR(this->get_logger(),
            "❌ Falha ao confirmar landing_mode=0 após %d tentativas. Encerrando nó.", kMaxParamRetries);
          timer_->cancel();
          rclcpp::shutdown();
        } else {
          param_ok_ = false;
          param_failed_ = false;
          verify_ok_ = false;
          verify_failed_ = false;
          param_retry_wait_ = kRetryBackoffTicks;   // 1 s backoff (AWAITING_PARAM_ACK counts down)
          state_ = State::AWAITING_PARAM_ACK;
        }
      } else {
        if (verify_wait_count_++ % kLogThrottleInterval == 0) {
          RCLCPP_INFO(this->get_logger(),
            "⏳ Aguardando resposta do get_parameters...");
        }
      }
      break;

    case State::PUBLISH_LAND:
    {
      double pub_x, pub_y;
      bool used_odom = false;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        used_odom = use_current_xy_ && odom_received_;
        pub_x = used_odom ? current_x_ : x_;
        pub_y = used_odom ? current_y_ : y_;
      }

      auto msg = geometry_msgs::msg::PoseStamped();
      msg.header.stamp = this->now();
      msg.header.frame_id = "map";
      msg.pose.position.x = pub_x;
      msg.pose.position.y = pub_y;
      msg.pose.position.z = z_land_;
      msg.pose.orientation.w = 1.0;

      waypoint_pub_->publish(msg);

      RCLCPP_INFO(this->get_logger(),
        "📡 Waypoint de POUSO publicado (%s) → X=%.2f, Y=%.2f, Z=%.2f",
        used_odom ? "XY atual" : "XY parametrizado", pub_x, pub_y, z_land_);

      // Salva XY de pouso para usar na decolagem (mesmo local)
      land_x_ = pub_x;
      land_y_ = pub_y;

      wait_start_ = this->now();
      state_ = State::WAIT_BEFORE_TAKEOFF;
      break;
    }

    case State::WAIT_BEFORE_TAKEOFF:
    {
      double elapsed = (this->now() - wait_start_).seconds();
      if (wait_count_log_++ % kLogThrottleInterval == 0) {
        RCLCPP_INFO(this->get_logger(),
          "⏳ Aguardando %.1f s antes de decolar... (%.1f / %.1f s)",
          wait_seconds_, elapsed, wait_seconds_);
      }
      if (elapsed >= wait_seconds_) {
        state_ = State::PUBLISH_TAKEOFF;
      }
      break;
    }

    case State::PUBLISH_TAKEOFF:
    {
      auto msg = geometry_msgs::msg::PoseStamped();
      msg.header.stamp = this->now();
      msg.header.frame_id = "map";
      msg.pose.position.x = land_x_;
      msg.pose.position.y = land_y_;
      msg.pose.position.z = z_takeoff_;
      msg.pose.orientation.w = 1.0;

      waypoint_pub_->publish(msg);

      RCLCPP_INFO(this->get_logger(),
        "🚀 Waypoint de DECOLAGEM publicado → X=%.2f, Y=%.2f, Z=%.2f",
        land_x_, land_y_, z_takeoff_);

      state_ = State::DONE;
      break;
    }

    case State::DONE:
      RCLCPP_INFO(this->get_logger(), "✅ Sequência concluída. Encerrando nó.");
      timer_->cancel();
      rclcpp::shutdown();
      break;
    }
  }

  void setLandingMode(int mode)
  {
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter param;
    param.name = "landing_mode";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    param.value.integer_value = mode;
    request->parameters.push_back(param);

    set_param_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
        auto response = future.get();
        if (!response->results.empty() && response->results[0].successful) {
          RCLCPP_INFO(this->get_logger(),
            "✅ set_parameters: landing_mode=%d → sucesso", mode);
          param_ok_ = true;
        } else {
          std::string reason = (!response->results.empty())
            ? response->results[0].reason : "sem resposta";
          RCLCPP_WARN(this->get_logger(),
            "⚠️  set_parameters: landing_mode=%d → falha (%s)", mode, reason.c_str());
          param_failed_ = true;
        }
      });
  }

  void verifyLandingMode(int expected_mode)
  {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("landing_mode");

    get_param_client_->async_send_request(
      request,
      [this, expected_mode](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
        auto response = future.get();
        if (!response->values.empty()) {
          int actual = static_cast<int>(response->values[0].integer_value);
          if (actual == expected_mode) {
            RCLCPP_INFO(this->get_logger(),
              "✅ get_parameters: landing_mode=%d confirmado no controlador", actual);
            verify_ok_ = true;
          } else {
            RCLCPP_WARN(this->get_logger(),
              "⚠️  get_parameters: esperado landing_mode=%d, mas controlador reporta %d",
              expected_mode, actual);
            verify_failed_ = true;
          }
        } else {
          RCLCPP_WARN(this->get_logger(),
            "⚠️  get_parameters: resposta vazia para landing_mode");
          verify_failed_ = true;
        }
      });
  }

  // ==========================================
  // PUBLISHERS / CLIENTS / TIMERS
  // ==========================================
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_param_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ==========================================
  // PARÂMETROS
  // ==========================================
  std::string controller_node_;
  double x_{0.0};
  double y_{0.0};
  double z_land_{0.01};
  double z_takeoff_{1.5};
  double wait_seconds_{5.0};
  bool use_current_xy_{true};

  // ==========================================
  // ESTADO INTERNO
  // ==========================================
  State state_{State::WAIT_SERVICE};
  rclcpp::Time wait_start_;
  int wait_count_{0};
  int wait_count_log_{0};

  // set_parameters async tracking
  std::atomic<bool> param_ok_{false};
  std::atomic<bool> param_failed_{false};
  int param_retry_count_{0};
  int param_retry_wait_{0};  // ticks to wait before next retry
  int param_wait_count_{0};  // separate counter for AWAITING_PARAM_ACK log throttle

  // get_parameters verification tracking
  std::atomic<bool> verify_ok_{false};
  std::atomic<bool> verify_failed_{false};
  int verify_wait_count_{0};

  // Posição atual do drone (odometria) — protegida por odom_mutex_
  std::mutex odom_mutex_;
  double current_x_{0.0};
  double current_y_{0.0};
  bool odom_received_{false};

  // XY onde o pouso foi enviado (usado para a decolagem posterior)
  double land_x_{0.0};
  double land_y_{0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingModeALandAndTakeoff>());
  rclcpp::shutdown();
  return 0;
}
