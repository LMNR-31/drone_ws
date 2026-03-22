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
      if (current_pose_.pose.position.z < 0.1) {
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

    RCLCPP_INFO(this->get_logger(), "✅ Waypoints de pouso publicados! Aguardando pouso...");

    // Aguarda o drone pousar de fato (Z < 0.1 m) antes de encerrar
    // Só aguarda se temos dados reais de pose
    if (pose_received_) {
      rclcpp::Rate wait_rate(10);
      int land_timeout = 0;
      const int max_wait_cycles = 300; // 30 segundos máximo
      while (current_pose_.pose.position.z > 0.5 && land_timeout < max_wait_cycles) {
        rclcpp::spin_some(this->get_node_base_interface());
        wait_rate.sleep();
        land_timeout++;
        if (land_timeout % 20 == 0) {
          RCLCPP_INFO(this->get_logger(), "  📍 Pousando... Z atual: %.2f m", current_pose_.pose.position.z);
        }
      }

      if (current_pose_.pose.position.z <= 0.5) {
        RCLCPP_INFO(this->get_logger(), "✅ Pouso confirmado! Z=%.2f m", current_pose_.pose.position.z);
      } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Timeout aguardando pouso! Z=%.2f m", current_pose_.pose.position.z);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️ Sem dados de pose - não é possível confirmar pouso");
    }

    RCLCPP_INFO(this->get_logger(), "✅ Finalizando nó drone_publish_landing_waypoints");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
  }

private:

  void publishLandingWaypoints()
  {
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    double current_z = current_pose_.pose.position.z;
    double landing_z = 0.05;
    double delta_z = current_z - landing_z;

    // ✅ Descida em 2 waypoints (início e solo)
    for (int i = 0; i <= 1; i++) {
      auto pose = geometry_msgs::msg::Pose();
      pose.position.x = current_pose_.pose.position.x;
      pose.position.y = current_pose_.pose.position.y;

      // Descida linear: começa em current_z, termina em 0.05
      double progress = static_cast<double>(i) / 1.0;  // 0.0 → 1.0
      pose.position.z = current_z - (delta_z * progress);

      pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    waypoints_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📍 WAYPOINTS DE POUSO COM DESCIDA PUBLICADOS:");
    RCLCPP_INFO(this->get_logger(),
      "   Posição XY: (%.2f, %.2f)",
      current_pose_.pose.position.x,
      current_pose_.pose.position.y);
    RCLCPP_INFO(this->get_logger(),
      "   Descida: %.2f m → 0.05 m (2 waypoints)",
      current_z);

    for (size_t i = 0; i < msg.poses.size(); i++) {
      RCLCPP_INFO(this->get_logger(),
        "   WP[%zu]: Z=%.2f",
        i,
        msg.poses[i].position.z);
    }
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