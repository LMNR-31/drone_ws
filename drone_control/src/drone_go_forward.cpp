#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <thread>

using namespace std::chrono_literals;

class DroneGoForward : public rclcpp::Node
{
public:
  DroneGoForward() : Node("drone_go_forward")
  {
    // ✅ CORRIGIDO: Usar :: ao invés de /
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      std::bind(&DroneGoForward::stateCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&DroneGoForward::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "⬆️ Go Forward iniciado - Aguardando drone ativado");
  }

private:

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;

    // ✅ Assim que detecta drone OFFBOARD+ARMED, publica waypoints
    if (current_state_.armed && current_state_.mode == "OFFBOARD" && !waypoint_sent_)
    {
      RCLCPP_INFO(this->get_logger(), "✓ Drone detectado em voo (OFFBOARD+ARMED)");
      publishTakeoffWaypoints(); // ✅ PUBLICA 2 WAYPOINTS AGORA
      waypoint_sent_ = true;
    }
  }

  void timerCallback()
  {
    if (waypoint_sent_ && cycle_count_ > 20) // ~2 segundos após envio
    {
      RCLCPP_INFO(this->get_logger(), "✅ Waypoints de levantamento enviados. Finalizando nó.");
      rclcpp::shutdown();
    }
    cycle_count_++;
  }

  void publishTakeoffWaypoints()
  {
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    // ✅ WAYPOINT 1: Posição inicial (ou 2m) - PONTO DE PARTIDA
    auto pose1 = geometry_msgs::msg::Pose();
    pose1.position.x = 0.0;
    pose1.position.y = 0.0;
    pose1.position.z = 2.0; // Começa de 2m (já está aí)
    pose1.orientation.w = 1.0;
    msg.poses.push_back(pose1);

    // ✅ WAYPOINT 2: Posição frente a 2m - PONTO DE CHEGADA
    auto pose2 = geometry_msgs::msg::Pose();
    pose2.position.x = 2.0; // Avança 2m para frente (X)
    pose2.position.y = 0.0;
    pose2.position.z = 2.0; // Mantém altura de 2m
    pose2.orientation.w = 1.0;
    msg.poses.push_back(pose2);

    waypoints_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📡 Waypoints de levantamento PUBLICADOS (2 pontos):");
    RCLCPP_INFO(this->get_logger(), "   WP1: X=0.0, Y=0.0, Z=2.0 (ponto de partida)");
    RCLCPP_INFO(this->get_logger(), "   WP2: X=2.0, Y=0.0, Z=2.0 (avanço frontal)");
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State current_state_;
  bool waypoint_sent_{false};
  int cycle_count_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneGoForward>());
  rclcpp::shutdown();
  return 0;
}