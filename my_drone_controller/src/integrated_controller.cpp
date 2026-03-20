#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;

class IntegratedController : public rclcpp::Node {
public:
    IntegratedController() : Node("integrated_controller_node") {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/uav1/mavros/setpoint_velocity/cmd_vel", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

        timer_ = this->create_wall_timer(50ms, std::bind(&IntegratedController::control_loop, this));
        
        cycle_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Missao Quadrado Iniciada (20Hz)");
    }

private:
    void control_loop() {
        cycle_count_++;
        auto now = this->get_clock()->now();
        auto vel_msg = geometry_msgs::msg::TwistStamped();
        vel_msg.header.stamp = now;
        vel_msg.header.frame_id = "uav1/fcu"; // Movimento relativo ao corpo do drone

        // --- FASE 1: DECOLAGEM (0 a 15s) ---
        if (cycle_count_ <= 300) {
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = now;
            pose_msg.header.frame_id = "uav1/map";
            pose_msg.pose.position.z = 5.0; 
            pose_pub_->publish(pose_msg);

            if (cycle_count_ > 20 && cycle_count_ < 100) {
                auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                arm_req->value = true;
                arm_client_->async_send_request(arm_req);

                auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                mode_req->custom_mode = "OFFBOARD";
                mode_client_->async_send_request(mode_req);
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "STATUS: Decolando... Ciclo: %d", cycle_count_);
        }
        // --- FASE 2: FRENTE (+X) ---
        else if (cycle_count_ <= 360) {
            vel_msg.twist.linear.x = 0.8;
            vel_pub_->publish(vel_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "QUADRADO: Frente...");
        }
        // --- FASE 3: DIREITA (-Y) ---
        else if (cycle_count_ <= 420) {
            vel_msg.twist.linear.y = -0.8; 
            vel_pub_->publish(vel_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "QUADRADO: Direita...");
        }
        // --- FASE 4: TRÁS (-X) ---
        else if (cycle_count_ <= 480) {
            vel_msg.twist.linear.x = -0.8;
            vel_pub_->publish(vel_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "QUADRADO: Tras...");
        }
        // --- FASE 5: ESQUERDA (+Y) ---
        else if (cycle_count_ <= 540) {
            vel_msg.twist.linear.y = 0.8;
            vel_pub_->publish(vel_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "QUADRADO: Esquerda (Voltando ao inicio)...");
        }
        // --- FASE 6: HOVER ---
        else {
            vel_msg.twist.linear.x = 0.0;
            vel_msg.twist.linear.y = 0.0;
            vel_pub_->publish(vel_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "STATUS: Quadrado completo. Em Hover.");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int cycle_count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntegratedController>());
    rclcpp::shutdown();
    return 0;
}