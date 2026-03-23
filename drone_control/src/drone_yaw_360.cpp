#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>

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
    // z_hold: altitude de hold durante o giro. -1.0 (default) = usar altitude atual do odometry.
    this->declare_parameter<double>("z_hold", -1.0);
    this->declare_parameter<double>("yaw_rate", 0.8);
    this->declare_parameter<double>("angle", 2.0 * M_PI);
    this->declare_parameter<double>("hz", 20.0);
    // ccw: true = anti-horário (counter-clockwise), false = horário (clockwise)
    this->declare_parameter<bool>("ccw", true);

    uav_ns_   = this->get_parameter("uav_ns").as_string();
    z_hold_   = this->get_parameter("z_hold").as_double();
    yaw_rate_ = this->get_parameter("yaw_rate").as_double();
    angle_    = this->get_parameter("angle").as_double();
    hz_       = this->get_parameter("hz").as_double();
    ccw_      = this->get_parameter("ccw").as_bool();

    if (hz_ <= 0.0) {
      throw std::runtime_error("Parâmetro 'hz' precisa ser > 0");
    }
    if (yaw_rate_ == 0.0) {
      throw std::runtime_error("Parâmetro 'yaw_rate' não pode ser 0");
    }

    // Normaliza: angle é tratado como magnitude; a direção vem de ccw_
    if (angle_ < 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "Parâmetro 'angle' negativo (%.4f rad) — usando valor absoluto. Direção controlada por 'ccw'.", angle_);
      angle_ = std::abs(angle_);
    }

    // yaw_rate com sinal: positivo = CCW, negativo = CW
    yaw_rate_signed_ = ccw_ ? std::abs(yaw_rate_) : -std::abs(yaw_rate_);

    // Duração do giro baseada em tempo: t = angle / |yaw_rate|
    duration_ = angle_ / std::abs(yaw_rate_);

    const char * dir_str = ccw_ ? "anti-horário (CCW)" : "horário (CW)";
    if (z_hold_ < 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "Parâmetros: uav_ns=%s z_hold=<altitude atual> yaw_rate=%.2f rad/s angle=%.2f rad hz=%.1f direção=%s duração=%.1fs",
        uav_ns_.c_str(), yaw_rate_, angle_, hz_, dir_str, duration_);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Parâmetros: uav_ns=%s z_hold=%.2f yaw_rate=%.2f rad/s angle=%.2f rad hz=%.1f direção=%s duração=%.1fs",
        uav_ns_.c_str(), z_hold_, yaw_rate_, angle_, hz_, dir_str, duration_);
    }

    // ==========================================
    // PUBLISHERS
    // ==========================================
    setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
      uav_ns_ + "/mavros/setpoint_raw/local", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: %s",
      (uav_ns_ + "/mavros/setpoint_raw/local").c_str());

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

  // type_mask bits for PositionTarget (1 = ignore that field)
  static constexpr uint16_t IGNORE_VX       = (1 << 3);   // 8
  static constexpr uint16_t IGNORE_VY       = (1 << 4);   // 16
  static constexpr uint16_t IGNORE_VZ       = (1 << 5);   // 32
  static constexpr uint16_t IGNORE_AFX      = (1 << 6);   // 64
  static constexpr uint16_t IGNORE_AFY      = (1 << 7);   // 128
  static constexpr uint16_t IGNORE_AFZ      = (1 << 8);   // 256
  static constexpr uint16_t IGNORE_YAW      = (1 << 10);  // 1024
  static constexpr uint16_t IGNORE_YAW_RATE = (1 << 11);  // 2048

  // Position + yaw_rate; ignore velocity, acceleration, yaw angle
  static constexpr uint16_t MASK_POS_YAWRATE =
    IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW;

  // Position only; ignore velocity, acceleration, yaw angle and yaw_rate
  static constexpr uint16_t MASK_POS_ONLY = MASK_POS_YAWRATE | IGNORE_YAW_RATE;

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

  void publishPositionTarget(double x, double y, double z, double yaw_rate, uint16_t type_mask)
  {
    mavros_msgs::msg::PositionTarget pt;
    pt.header.stamp = this->now();
    pt.header.frame_id = "map";
    pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    pt.type_mask = type_mask;
    pt.position.x = x;
    pt.position.y = y;
    pt.position.z = z;
    pt.yaw_rate = static_cast<float>(yaw_rate);
    setpoint_pub_->publish(pt);
  }

  void timerCallback()
  {
    switch (state_)
    {
      case StateMachine::WAIT_FCU:
      {
        // Publica setpoint neutro para manter stream ativo
        const double z = (z_hold_ >= 0.0) ? z_hold_ : current_z_;
        publishPositionTarget(0.0, 0.0, z, 0.0, MASK_POS_ONLY);

        if (current_state_.connected) {
          RCLCPP_INFO(this->get_logger(), "✓ FCU conectada!");
          state_ = StateMachine::WAIT_ODOM;
          state_time_ = this->now();
        }
        break;
      }

      case StateMachine::WAIT_ODOM:
      {
        const double z = (z_hold_ >= 0.0) ? z_hold_ : current_z_;
        publishPositionTarget(0.0, 0.0, z, 0.0, MASK_POS_ONLY);

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
        const double z = (z_hold_ >= 0.0) ? z_hold_ : current_z_;
        publishPositionTarget(current_x_, current_y_, z, 0.0, MASK_POS_ONLY);

        if (current_state_.armed && current_state_.mode == "OFFBOARD") {
          // Congela posição e altitude no momento de iniciar o giro
          hold_x_ = current_x_;
          hold_y_ = current_y_;
          hold_z_ = (z_hold_ >= 0.0) ? z_hold_ : current_z_;
          RCLCPP_INFO(this->get_logger(), "✅ OFFBOARD + ARMED detectado. Iniciando giro %.1f° %s.",
            angle_ * 180.0 / M_PI, ccw_ ? "anti-horário (CCW)" : "horário (CW)");
          RCLCPP_INFO(this->get_logger(), "   Hold: X=%.2f Y=%.2f Z=%.2f  yaw_rate=%.3f rad/s  duração=%.1fs",
            hold_x_, hold_y_, hold_z_, yaw_rate_signed_, duration_);
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
        publishPositionTarget(hold_x_, hold_y_, hold_z_, yaw_rate_signed_, MASK_POS_YAWRATE);

        const double elapsed = (this->now() - start_time_).seconds();
        if (elapsed >= duration_) {
          RCLCPP_INFO(this->get_logger(), "✅ Giro concluído! (%.1f° %s em %.2fs)",
            angle_ * 180.0 / M_PI, ccw_ ? "CCW" : "CW", elapsed);
          finish_time_ = this->now();
          state_ = StateMachine::FINISH;
        }
        break;
      }

      case StateMachine::FINISH:
      {
        // Segurança: yaw_rate zero, mantém posição por ~1s antes de encerrar
        publishPositionTarget(hold_x_, hold_y_, hold_z_, 0.0, MASK_POS_YAWRATE);
        if ((this->now() - finish_time_).seconds() >= 1.0) {
          RCLCPP_INFO(this->get_logger(), "Encerrando nó.");
          rclcpp::shutdown();
        }
        break;
      }
    }
  }

  // ==========================================
  // PUBLISHERS / SUBSCRIBERS / TIMER
  // ==========================================
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
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
  double z_hold_{-1.0};
  double yaw_rate_{0.8};
  double angle_{2.0 * M_PI};
  double hz_{20.0};
  bool   ccw_{true};

  // yaw_rate com sinal: positivo = CCW, negativo = CW
  double yaw_rate_signed_{0.8};

  // Duração do giro em segundos
  double duration_{0.0};

  // ==========================================
  // ROTAÇÃO
  // ==========================================
  rclcpp::Time finish_time_{0, 0, RCL_ROS_TIME};

  // Posição e altitude congeladas no início do giro
  double hold_x_{0.0};
  double hold_y_{0.0};
  double hold_z_{0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneYaw360>());
  rclcpp::shutdown();
  return 0;
}
