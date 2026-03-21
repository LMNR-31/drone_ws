#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class DronePublishLandingWaypoints : public rclcpp::Node
{
public:
  DronePublishLandingWaypoints() : Node("drone_publish_landing_waypoints")
  {
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/uav1/mavros/local_position/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        pose_received_ = true;
      });

    RCLCPP_INFO(this->get_logger(), "📡 Nó de publicação de waypoints de pouso iniciado");
    RCLCPP_INFO(this->get_logger(), "⏳ Aguardando posição do drone...");

    rclcpp::Rate loop_rate(10);
    int wait_count = 0;
    while (!pose_received_ && wait_count < 50) {
      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
      wait_count++;
    }

    if (!pose_received_) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Timeout aguardando pose. Mantendo última posição conhecida...");
      // NÃO reseta X e Y - mantém o que foi recebido (ou 0,0 se nada foi recebido)
      // Apenas ajusta Z se estiver abaixo de um valor seguro
      if (current_pose_.pose.position.z < 0.5) {
        current_pose_.pose.position.z = 2.0;  // Altura segura
      }
      RCLCPP_INFO(this->get_logger(), "✅ Usando última pose: X=%.2f, Y=%.2f, Z=%.2f",
        current_pose_.pose.position.x,
        current_pose_.pose.position.y,
        current_pose_.pose.position.z);
    } else {
      RCLCPP_INFO(this->get_logger(), "✅ Pose recebida! X=%.2f, Y=%.2f, Z=%.2f",
        current_pose_.pose.position.x,
        current_pose_.pose.position.y,
        current_pose_.pose.position.z);
    }

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

    // ✅ WP1: Posição ATUAL do drone (X, Y, Z atuais)
    auto pose1 = geometry_msgs::msg::Pose();
    pose1.position.x = current_pose_.pose.position.x;
    pose1.position.y = current_pose_.pose.position.y;
    pose1.position.z = current_pose_.pose.position.z;
    pose1.orientation.w = 1.0;
    msg.poses.push_back(pose1);

    // ✅ WP2: Pouso no MESMO X, Y mas Z=0.05 (solo)
    auto pose2 = geometry_msgs::msg::Pose();
    pose2.position.x = current_pose_.pose.position.x;
    pose2.position.y = current_pose_.pose.position.y;
    pose2.position.z = 0.05;
    pose2.orientation.w = 1.0;
    msg.poses.push_back(pose2);

    waypoints_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📍 WAYPOINTS DE POUSO PUBLICADOS:");
    RCLCPP_INFO(this->get_logger(),
      "   WP1: X=%.2f, Y=%.2f, Z=%.2f (posição atual)",
      current_pose_.pose.position.x,
      current_pose_.pose.position.y,
      current_pose_.pose.position.z);
    RCLCPP_INFO(this->get_logger(),
      "   WP2: X=%.2f, Y=%.2f, Z=0.05 (POUSO NO PONTO!)",
      current_pose_.pose.position.x,
      current_pose_.pose.position.y);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped current_pose_ = []() {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 2.0;  // Altura segura como fallback inicial
    p.pose.orientation.w = 1.0;
    return p;
  }();
  bool pose_received_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DronePublishLandingWaypoints>());
  rclcpp::shutdown();
  return 0;
}