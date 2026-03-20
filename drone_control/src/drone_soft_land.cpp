#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class DroneSoftLand : public rclcpp::Node
{
public:
  DroneSoftLand() : Node("drone_soft_land")
  {
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      std::bind(&DroneSoftLand::stateCb, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&DroneSoftLand::odomCb, this, std::placeholders::_1));

    setpoint_pub_ =
      this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/uav1/mavros/setpoint_raw/local", 10);

    target_.coordinate_frame =
      mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    // ⭐ controlar POSIÇÃO XYZ
    target_.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_VX |
      mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    timer_ = this->create_wall_timer(
      50ms, std::bind(&DroneSoftLand::timerCb, this));

    RCLCPP_INFO(this->get_logger(),"Soft land POSITION mode");
  }

private:

  void stateCb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    state_ = *msg;
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    z_ = msg->pose.pose.position.z;
  }

  void timerCb()
  {
    target_.header.stamp = this->now();

    // ⭐ Sempre mandar setpoint
    if (!initialized_ && z_ > 0.1)
    {
      target_z_ = z_;
      target_.position.z = z_;
      target_.yaw = 0.0;

      initialized_ = true;

      RCLCPP_INFO(this->get_logger(),
        "Initial altitude locked: %.2f", z_);
    }

    // ⭐ Só desce se estiver em OFFBOARD armado
    if (!state_.armed || state_.mode != "OFFBOARD")
    {
      setpoint_pub_->publish(target_);
      return;
    }

    // ⭐ Pouso progressivo
    if (!landed_)
    {
      target_z_ -= descent_speed_;

      if (target_z_ < 0.05)
        target_z_ = 0.05;

      target_.position.z = target_z_;
      target_.yaw = 0.0;

      if (z_ < 0.07)
      {
        landed_ = true;
        landed_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),"✅ Solo detectado");
      }
    }
    else
    {
      // ⭐ HOLD no solo
      target_.position.z = 0.05;

      double dt = (this->now() - landed_time_).seconds();

      if (dt > 3.0 && !shutdown_)
      {
        shutdown_ = true;

        RCLCPP_INFO(this->get_logger(),
          "🚀 Landing finished — shutdown node");

        rclcpp::shutdown();
      }
    }

    setpoint_pub_->publish(target_);
  }

  mavros_msgs::msg::State state_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::PositionTarget target_;

  double z_{5.0};
  double target_z_{5.0};

  bool initialized_{false};
  bool landed_{false};
  bool shutdown_{false};

  double descent_speed_{0.015};

  rclcpp::Time landed_time_;
};

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<DroneSoftLand>());
  rclcpp::shutdown();
  return 0;
}