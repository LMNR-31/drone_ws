#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
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

    setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
      "/uav1/mavros/setpoint_raw/local", 10);

    disarm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      "/uav1/mavros/cmd/arming");

    target_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    // Controlar apenas posição XYZ
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

    RCLCPP_INFO(this->get_logger(), "🛬 Soft Land iniciado");
    RCLCPP_INFO(this->get_logger(), "⏳ Aguardando altitude do drone...");
  }

private:

  void stateCb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    state_ = *msg;
    state_received_ = true; // ✅ Flag para confirmar que recebeu estado
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    z_ = msg->pose.pose.position.z;
    odom_received_ = true; // ✅ Flag para confirmar que recebeu odometria
  }

  void timerCb()
  {
    target_.header.stamp = this->now();

    // ✅ Aguarda receber as primeiras mensagens do drone
    if (!state_received_ || !odom_received_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "⏳ Aguardando dados do drone...");
      return;
    }

    // ✅ INICIALIZA altitude alvo (so acontece UMA VEZ)
    if (!initialized_ && z_ > 0.05)
    {
      target_z_ = z_;
      target_.position.z = z_;
      target_.yaw = 0.0;
      initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "✓ Altitude inicial detectada: %.2f m", z_);
      RCLCPP_INFO(this->get_logger(), "🛬 Iniciando descida...");
    }

    if (!initialized_)
    {
      return; // Aguarda inicialização
    }

    // ✅ POUSO PROGRESSIVO - SEMPRE DESCE, NÃO IMPORTA ESTADO DO DRONE
    if (!landed_)
    {
      target_z_ -= descent_speed_; // Desce progressivamente

      if (target_z_ < 0.05)
        target_z_ = 0.05; // Para no solo

      target_.position.z = target_z_;
      target_.yaw = 0.0;

      if (cycle_count_ % 20 == 0)
      {
        RCLCPP_INFO(this->get_logger(), "📍 Descendo... Z_alvo=%.2f | Z_real=%.2f m", 
          target_z_, z_);
      }

      // ✅ Detecta quando chegou no solo (usando odometria real)
      if (z_ < 0.15)
      {
        landed_ = true;
        landed_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "✅ Solo detectado! Z_real = %.2f m", z_);
        RCLCPP_INFO(this->get_logger(), "🛑 Solicitando DISARM...");
        requestDisarm();
      }
    }
    else
    {
      // ✅ MANTER NO SOLO
      target_.position.z = 0.05;
      double dt = (this->now() - landed_time_).seconds();

      if (cycle_count_ % 20 == 0)
      {
        RCLCPP_INFO(this->get_logger(), "⏳ No solo por %.1f segundos | Z=%.2f m", dt, z_);
      }

      // ✅ DESARM após 2 segundos
      if (dt > 2.0 && !disarmed_ && state_received_)
      {
        disarmed_ = true;
        requestDisarm();
      }

      // ✅ FINALIZAR após 3 segundos no solo
      if (dt > 3.0 && !shutdown_)
      {
        shutdown_ = true;
        RCLCPP_INFO(this->get_logger(), "🚀 Pouso concluído - Encerrando nó");
        rclcpp::shutdown();
      }
    }

    setpoint_pub_->publish(target_);
    cycle_count_++;
  }

  void requestDisarm()
  {
    if (!disarm_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "⚠️ Serviço de disarm não disponível");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;
    disarm_client_->async_send_request(req);
  }

  // Subscrições
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr disarm_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Mensagens
  mavros_msgs::msg::State state_;
  mavros_msgs::msg::PositionTarget target_;

  // Estados
  double z_{5.0};
  double target_z_{5.0};
  bool initialized_{false};
  bool landed_{false};
  bool disarmed_{false};
  bool shutdown_{false};
  bool state_received_{false};    // ✅ Flag para confirmar recepção
  bool odom_received_{false};     // ✅ Flag para confirmar recepção
  int cycle_count_{0};

  // Parâmetros
  double descent_speed_{0.03};    // ✅ AUMENTADO: 3cm/ciclo = 1.5m/s (mais rápido)

  rclcpp::Time landed_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneSoftLand>());
  rclcpp::shutdown();
  return 0;
}