#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;

class OffboardTakeoff : public rclcpp::Node {
public:
    OffboardTakeoff() : Node("offboard_takeoff_node") {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/uav1/mavros/setpoint_position/local", 10);

        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardTakeoff::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Aguardando comando para subir 2 metros...");
    }

private:
    void timer_callback() {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "uav1/map";
        pose_msg.pose.position.x = 0.0;
        pose_msg.pose.position.y = 0.0;
        pose_msg.pose.position.z = 2.0; // ALVO: 2 METROS
        pose_pub_->publish(pose_msg);

        static int retry_count = 0;
        if (retry_count < 10) {
            if (arm_client_->service_is_ready() && mode_client_->service_is_ready()) {
                auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_req->value = true;
                arm_client_->async_send_request(arm_req);

                auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                mode_req->custom_mode = "OFFBOARD";
                mode_client_->async_send_request(mode_req);
                retry_count++;
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardTakeoff>());
    rclcpp::shutdown();
    return 0;
}