#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

class DroneActivator : public rclcpp::Node
{
public:
  DroneActivator() : Node("drone_activator")
  {
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      std::bind(&DroneActivator::stateCallback, this, _1));

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/uav1/mavros/setpoint_position/local", 10);

    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

    setpoint_.pose.position.z = 0.0;
    setpoint_.pose.orientation.w = 1.0;

    timer_ = this->create_wall_timer(50ms, std::bind(&DroneActivator::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "🔋 Ativador iniciado - Aguardando conexão FCU");
  }

private:

  enum class StateMachine
  {
    WAIT_FCU,
    STREAM_SETPOINT,
    REQUEST_OFFBOARD,
    REQUEST_ARM,
    CONFIRM_ACTIVATION,
    ACTIVATED,
    FINISH
  };

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void timerCallback()
  {
    setpoint_.header.stamp = this->now();
    setpoint_pub_->publish(setpoint_);

    switch (state_)
    {
    case StateMachine::WAIT_FCU:
      if (current_state_.connected)
      {
        RCLCPP_INFO(this->get_logger(), "✓ FCU conectada");
        state_ = StateMachine::STREAM_SETPOINT;
        state_time_ = this->now();
      }
      break;

    case StateMachine::STREAM_SETPOINT:
      if ((this->now() - state_time_) > rclcpp::Duration(2s))
      {
        RCLCPP_INFO(this->get_logger(), "✓ Setpoints transmitidos");
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
        RCLCPP_INFO(this->get_logger(), "✅ DRONE ATIVADO! (OFFBOARD+ARMED)");
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
        RCLCPP_INFO(this->get_logger(), "🎯 Finalizando activador...");
        state_ = StateMachine::FINISH;
      }
      break;

    case StateMachine::FINISH:
      rclcpp::shutdown();
      break;
    }
  }

  void requestOffboard()
  {
    if (!mode_client_->wait_for_service(1s))
      return;

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "OFFBOARD";
    mode_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "→ Solicitando OFFBOARD");
    state_time_ = this->now();
  }

  void requestArm()
  {
    if (!arming_client_->wait_for_service(1s))
      return;

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    arming_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "→ Solicitando ARM");
    state_time_ = this->now();
  }

  StateMachine state_{StateMachine::WAIT_FCU};
  mavros_msgs::msg::State current_state_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped setpoint_;
  rclcpp::Time state_time_;
  rclcpp::Time activated_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneActivator>());
  rclcpp::shutdown();
  return 0;
}