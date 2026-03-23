#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

class DroneYaw360 : public rclcpp::Node
{
public:
  DroneYaw360() : Node("drone_yaw_360")
  {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🔄  DRONE YAW 360 - GIRO EM TORNO DO EIXO Z      ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ==========================================
    // PARÂMETROS
    // ==========================================
    this->declare_parameter<std::string>("uav_ns", "/uav1");
    this->declare_parameter<double>("z_hold", 2.0);
    this->declare_parameter<double>("yaw_rate", 0.8);
    this->declare_parameter<double>("angle", 2.0 * M_PI);
    this->declare_parameter<double>("hz", 20.0);

    uav_ns_   = this->get_parameter("uav_ns").as_string();
    z_hold_   = this->get_parameter("z_hold").as_double();
    yaw_rate_ = this->get_parameter("yaw_rate").as_double();
    angle_    = this->get_parameter("angle").as_double();
    hz_       = this->get_parameter("hz").as_double();

    if (hz_ <= 0.0) {
      throw std::runtime_error("Parâmetro 'hz' precisa ser > 0");
    }
    if (yaw_rate_ == 0.0) {
      throw std::runtime_error("Parâmetro 'yaw_rate' não pode ser 0");
    }

    RCLCPP_INFO(this->get_logger(), "Parâmetros: uav_ns=%s z_hold=%.2f yaw_rate=%.2f rad/s angle=%.2f rad hz=%.1f",
      uav_ns_.c_str(), z_hold_, yaw_rate_, angle_, hz_);

    // ==========================================
    // PUBLISHERS
    // ==========================================
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      uav_ns_ + "/mavros/setpoint_position/local", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: %s",
      (uav_ns_ + "/mavros/setpoint_position/local").c_str());

    // ==========================================
    // SUBSCRIBERS
    // ==========================================
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      uav_ns_ + "/mavros/state", 10,
      std::bind(&DroneYaw360::stateCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      uav_ns_ + "/mavros/local_position/odom", 10,
      std::bind(&DroneYaw360::odomCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "✓ Subscribers criados: %s/mavros/state e %s/mavros/local_position/odom",
      uav_ns_.c_str(), uav_ns_.c_str());

    // ==========================================
    // TIMER
    // ==========================================
    state_ = StateMachine::WAIT_FCU;
    state_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz_),
      std::bind(&DroneYaw360::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "✓ Timer criado: %.1f Hz", hz_);
    RCLCPP_INFO(this->get_logger(), "\n🔄 Aguardando conexão FCU...");
  }

private:

  enum class StateMachine {
    WAIT_FCU,
    WAIT_ODOM,
    WAIT_OFFBOARD_AND_ARMED,
    ROTATING,
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
  }

  static geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
  }

  void publishSetpointYaw(double yaw)
  {
    geometry_msgs::msg::PoseStamped sp;
    sp.header.stamp = this->now();
    sp.header.frame_id = "map";
    sp.pose.position.x = current_x_;
    sp.pose.position.y = current_y_;
    sp.pose.position.z = z_hold_;
    sp.pose.orientation = quatFromYaw(yaw);
    setpoint_pub_->publish(sp);
  }

  void timerCallback()
  {
    switch (state_)
    {
      case StateMachine::WAIT_FCU:
      {
        // Publica setpoint neutro para manter stream ativo
        geometry_msgs::msg::PoseStamped sp;
        sp.header.stamp = this->now();
        sp.header.frame_id = "map";
        sp.pose.position.z = z_hold_;
        sp.pose.orientation.w = 1.0;
        setpoint_pub_->publish(sp);

        if (current_state_.connected) {
          RCLCPP_INFO(this->get_logger(), "✓ FCU conectada!");
          state_ = StateMachine::WAIT_ODOM;
          state_time_ = this->now();
        }
        break;
      }

      case StateMachine::WAIT_ODOM:
      {
        geometry_msgs::msg::PoseStamped sp;
        sp.header.stamp = this->now();
        sp.header.frame_id = "map";
        sp.pose.position.z = z_hold_;
        sp.pose.orientation.w = 1.0;
        setpoint_pub_->publish(sp);

        if (odom_received_) {
          RCLCPP_INFO(this->get_logger(), "✓ Odom recebida: X=%.2f Y=%.2f Z=%.2f",
            current_x_, current_y_, current_z_);
          state_ = StateMachine::WAIT_OFFBOARD_AND_ARMED;
          state_time_ = this->now();
        }
        break;
      }

      case StateMachine::WAIT_OFFBOARD_AND_ARMED:
      {
        publishSetpointYaw(0.0);

        if (current_state_.armed && current_state_.mode == "OFFBOARD") {
          RCLCPP_INFO(this->get_logger(), "✅ OFFBOARD + ARMED detectado. Iniciando giro 360.");
          start_yaw_ = 0.0;
          start_time_ = this->now();
          state_ = StateMachine::ROTATING;
        } else if ((this->now() - state_time_).seconds() > 2.0) {
          RCLCPP_WARN(this->get_logger(),
            "⏳ Aguardando OFFBOARD+ARMED... (armed=%d mode=%s)",
            (int)current_state_.armed, current_state_.mode.c_str());
          state_time_ = this->now();
        }
        break;
      }

      case StateMachine::ROTATING:
      {
        const double elapsed = (this->now() - start_time_).seconds();
        const double direction = (angle_ >= 0.0) ? 1.0 : -1.0;
        const double rotated = std::min(std::abs(angle_), std::abs(yaw_rate_) * elapsed);
        const double yaw = start_yaw_ + direction * rotated;

        publishSetpointYaw(yaw);

        if (rotated >= std::abs(angle_) - 1e-3) {
          RCLCPP_INFO(this->get_logger(), "✅ Giro concluído! (%.1f°)",
            angle_ * 180.0 / M_PI);
          finish_yaw_ = yaw;
          state_ = StateMachine::FINISH;
        }
        break;
      }

      case StateMachine::FINISH:
      {
        publishSetpointYaw(finish_yaw_);
        rclcpp::shutdown();
        break;
      }
    }
  }

  // ==========================================
  // PUBLISHERS / SUBSCRIBERS / TIMER
  // ==========================================
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ==========================================
  // ESTADO
  // ==========================================
  StateMachine state_{StateMachine::WAIT_FCU};
  rclcpp::Time state_time_;
  rclcpp::Time start_time_;

  mavros_msgs::msg::State current_state_;
  bool odom_received_{false};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};

  // ==========================================
  // PARÂMETROS
  // ==========================================
  std::string uav_ns_;
  double z_hold_{2.0};
  double yaw_rate_{0.8};
  double angle_{2.0 * M_PI};
  double hz_{20.0};

  // ==========================================
  // ROTAÇÃO
  // ==========================================
  double start_yaw_{0.0};
  double finish_yaw_{0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneYaw360>());
  rclcpp::shutdown();
  return 0;
}
