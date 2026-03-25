#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

class DroneActivateAndGoForward : public rclcpp::Node
{
public:
  DroneActivateAndGoForward() : Node("drone_activate_and_go_forward")
  {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🔋⬆️  ATIVADOR + LEVANTADOR UNIFICADO             ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ==========================================
    // PUBLISHERS
    // ==========================================
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/uav1/mavros/setpoint_position/local", 10);

    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/waypoints", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publishers criados");

    // ==========================================
    // SUBSCRIBERS
    // ==========================================
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      std::bind(&DroneActivateAndGoForward::stateCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&DroneActivateAndGoForward::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "✓ Subscriber criado");

    // ==========================================
    // SERVICE CLIENTS
    // ==========================================
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

    RCLCPP_INFO(this->get_logger(), "✓ Service Clients criados");

    // ==========================================
    // TIMER
    // ==========================================
    timer_ = this->create_wall_timer(50ms, std::bind(&DroneActivateAndGoForward::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "✓ Timer criado: 20 Hz (50ms)");

    // ==========================================
    // INICIALIZAÇÃO
    // ==========================================
    setpoint_.pose.position.z = 2.0;
    setpoint_.pose.orientation.w = 1.0;

    state_ = StateMachine::WAIT_FCU;
    state_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "\n🔋 Ativador iniciado - Aguardando conexão FCU");
  }

private:

  enum class StateMachine {
    WAIT_FCU,
    STREAM_SETPOINT,
    REQUEST_OFFBOARD,
    REQUEST_ARM,
    CONFIRM_ACTIVATION,
    ACTIVATED,
    PUBLISH_WAYPOINTS,
    VERIFY_TAKEOFF,
    WAIT_BEFORE_RETRY,
    FINISH
  };

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    odom_received_ = true;

    if (cycle_count_ % 50 == 0) {
      RCLCPP_DEBUG(this->get_logger(), "📍 Posição atual: X=%.2f, Y=%.2f, Z=%.2f",
        current_x_, current_y_, current_z_);
    }
  }

  void timerCallback()
  {
    cycle_count_++;
    setpoint_.header.stamp = this->now();
    setpoint_.header.frame_id = "map";

    // Sempre publica setpoints para manter OFFBOARD ativo
    pose_pub_->publish(setpoint_);

    switch (state_)
    {
    case StateMachine::WAIT_FCU:
      if (current_state_.connected)
      {
        RCLCPP_INFO(this->get_logger(), "✓ FCU conectada!");
        state_ = StateMachine::STREAM_SETPOINT;
        state_time_ = this->now();
      }
      break;

    case StateMachine::STREAM_SETPOINT:
      if ((this->now() - state_time_) > rclcpp::Duration(3s))
      {
        if (!odom_received_) {
          if (cycle_count_ % 20 == 0) {
            RCLCPP_WARN(this->get_logger(), "⏳ Aguardando posição do drone...");
          }
          return;
        }

        RCLCPP_INFO(this->get_logger(), "✓ Setpoints transmitidos. Solicitando OFFBOARD...");
        RCLCPP_INFO(this->get_logger(), "📍 Posição detectada: X=%.2f, Y=%.2f, Z=%.2f",
          current_x_, current_y_, current_z_);
        state_ = StateMachine::REQUEST_OFFBOARD;
      }
      break;

    case StateMachine::REQUEST_OFFBOARD:
      requestOffboard();
      state_ = StateMachine::REQUEST_ARM;
      break;

    case StateMachine::REQUEST_ARM:
      requestArm();
      state_ = StateMachine::CONFIRM_ACTIVATION;
      break;

    case StateMachine::CONFIRM_ACTIVATION:
      if (current_state_.armed && current_state_.mode == "OFFBOARD")
      {
        RCLCPP_INFO(this->get_logger(), "✅ DRONE ATIVADO! (OFFBOARD+ARMED)\n");
        state_ = StateMachine::ACTIVATED;
        activated_time_ = this->now();
      }
      else if ((this->now() - state_time_) > rclcpp::Duration(3s))
      {
        RCLCPP_WARN(this->get_logger(), "⚠️ Ativação pendente, tentando novamente...");
        state_ = StateMachine::REQUEST_OFFBOARD;
      }
      break;

    case StateMachine::ACTIVATED:
      if ((this->now() - activated_time_) > rclcpp::Duration(1s))
      {
        RCLCPP_INFO(this->get_logger(), "⬆️ Publicando waypoints de levantamento...\n");
        publishTakeoffWaypoints();
        state_ = StateMachine::PUBLISH_WAYPOINTS;
        publish_time_ = this->now();
        verify_attempts_ = 0;
      }
      break;

    case StateMachine::PUBLISH_WAYPOINTS:
      if ((this->now() - publish_time_) > rclcpp::Duration(2s))
      {
        RCLCPP_INFO(this->get_logger(), "✅ Waypoints de levantamento enviados. Verificando decolagem...\n");
        state_ = StateMachine::VERIFY_TAKEOFF;
        verify_start_time_ = this->now();
      }
      break;

    case StateMachine::VERIFY_TAKEOFF:
      if (cycle_count_ % 20 == 0)
      {
        RCLCPP_INFO(this->get_logger(), "⬆️ Verificando decolagem... Altitude: %.2fm (threshold: %.2fm)",
          current_z_, altitude_threshold_);
      }
      if ((this->now() - verify_start_time_) > rclcpp::Duration(3s))
      {
        bool altitude_ok = current_z_ > altitude_threshold_;
        bool mode_ok = current_state_.armed && current_state_.mode == "OFFBOARD";

        if (altitude_ok && mode_ok)
        {
          RCLCPP_INFO(this->get_logger(),
            "✅ Decolagem confirmada! Altitude: %.2fm | OFFBOARD+ARMED\n", current_z_);
          state_ = StateMachine::FINISH;
        }
        else
        {
          verify_attempts_++;
          if (!altitude_ok)
          {
            RCLCPP_WARN(this->get_logger(), "⚠️ Altitude insuficiente: %.2fm < %.2fm",
              current_z_, altitude_threshold_);
          }
          if (!mode_ok)
          {
            RCLCPP_WARN(this->get_logger(), "⚠️ Modo incorreto: armed=%d, mode=%s",
              current_state_.armed, current_state_.mode.c_str());
          }

          if (verify_attempts_ < max_verify_attempts_)
          {
            RCLCPP_WARN(this->get_logger(),
              "⚠️ Decolagem inefetiva, aguardando 5s antes de tentar novamente... (%d/%d)",
              verify_attempts_, max_verify_attempts_);
            state_ = StateMachine::WAIT_BEFORE_RETRY;
            wait_retry_start_time_ = this->now();
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(),
              "❌ Falha na verificação de decolagem após %d tentativas. Finalizando.",
              max_verify_attempts_);
            state_ = StateMachine::FINISH;
          }
        }
      }
      break;

    case StateMachine::WAIT_BEFORE_RETRY:
      if (cycle_count_ % 20 == 0)
      {
        auto elapsed = (this->now() - wait_retry_start_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "⏳ Aguardando antes de retry... (%.0fs/5s)", elapsed);
      }
      if ((this->now() - wait_retry_start_time_) > rclcpp::Duration(5s))
      {
        RCLCPP_INFO(this->get_logger(), "📡 Retentando decolagem... (%d/%d)",
          verify_attempts_, max_verify_attempts_);
        publishTakeoffWaypoints();
        publish_time_ = this->now();
        state_ = StateMachine::PUBLISH_WAYPOINTS;
      }
      break;

    case StateMachine::FINISH:
      rclcpp::shutdown();
      break;
    }
  }

  void requestOffboard()
  {
    if (!mode_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";

    mode_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "📡 Solicitando OFFBOARD MODE...");
    state_time_ = this->now();
  }

  void requestArm()
  {
    if (!arming_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    arming_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "🔋 Solicitando ARM...");
    state_time_ = this->now();
  }

  void publishTakeoffWaypoints()
  {
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    // ✅ WAYPOINT 1: Levanta na posição atual
    auto pose1 = geometry_msgs::msg::Pose();
    pose1.position.x = current_x_;
    pose1.position.y = current_y_;
    pose1.position.z = 2.0;
    pose1.orientation.w = 1.57;
    msg.poses.push_back(pose1);

    waypoints_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📡 WAYPOINT DE LEVANTAMENTO PUBLICADO:");
    RCLCPP_INFO(this->get_logger(), "   WP1: X=%.2f, Y=%.2f, Z=2.0 (levanta no ponto atual)",
      current_x_, current_y_);
  }

  // ==========================================
  // PUBLISHERS
  // ==========================================
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;

  // ==========================================
  // SUBSCRIBERS
  // ==========================================
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ==========================================
  // SERVICE CLIENTS
  // ==========================================
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

  // ==========================================
  // TIMER
  // ==========================================
  rclcpp::TimerBase::SharedPtr timer_;

  // ==========================================
  // MENSAGENS E ESTADO
  // ==========================================
  geometry_msgs::msg::PoseStamped setpoint_;
  mavros_msgs::msg::State current_state_;

  StateMachine state_;
  rclcpp::Time state_time_;
  rclcpp::Time activated_time_;
  rclcpp::Time publish_time_;

  // Posição atual do drone
  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  bool odom_received_{false};

  // Contador de ciclos para debug
  int cycle_count_{0};

  // Verificação de decolagem
  static constexpr double altitude_threshold_{0.5};
  static constexpr int max_verify_attempts_{5};
  rclcpp::Time verify_start_time_;
  rclcpp::Time wait_retry_start_time_;
  int verify_attempts_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneActivateAndGoForward>());
  rclcpp::shutdown();
  return 0;
}