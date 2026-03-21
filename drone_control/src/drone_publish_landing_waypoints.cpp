#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class DronePublishLandingWaypoints : public rclcpp::Node
{
public:
  DronePublishLandingWaypoints() : Node("drone_publish_landing_waypoints")
  {
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

    RCLCPP_INFO(this->get_logger(), "📡 Nó de publicação de waypoints de pouso iniciado");

    // Publica waypoints de pouso imediatamente
    publishLandingWaypoints();

    RCLCPP_INFO(this->get_logger(), "✅ Waypoints de pouso publicados! Finalizando nó");
    
    // Aguarda um pouco para garantir que foi publicado
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
  }

private:

  void publishLandingWaypoints()
  {
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    // ✅ WAYPOINT 1: Posição inicial em 2m (onde o drone está)
    auto pose1 = geometry_msgs::msg::Pose();
    pose1.position.x = 0.0;
    pose1.position.y = 0.0;
    pose1.position.z = 2.0; // Começa de 2m (posição atual do drone)
    pose1.orientation.w = 1.0;
    msg.poses.push_back(pose1);

    // ✅ WAYPOINT 2: Pouso no solo (Z = 0.05m)
    auto pose2 = geometry_msgs::msg::Pose();
    pose2.position.x = 0.0;
    pose2.position.y = 0.0;
    pose2.position.z = 0.05; // Solo!
    pose2.orientation.w = 1.0;
    msg.poses.push_back(pose2);

    waypoints_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📍 WAYPOINTS DE POUSO PUBLICADOS:");
    RCLCPP_INFO(this->get_logger(), "   WP1: X=0.0, Y=0.0, Z=2.0 (posição atual)");
    RCLCPP_INFO(this->get_logger(), "   WP2: X=0.0, Y=0.0, Z=0.05 (SOLO!)");
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DronePublishLandingWaypoints>());
  rclcpp::shutdown();
  return 0;
}