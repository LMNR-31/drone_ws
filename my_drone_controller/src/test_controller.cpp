// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"
// #include "mavros_msgs/srv/set_mode.hpp"
// #include "mavros_msgs/srv/command_bool.hpp"

// using namespace std::chrono_literals;

// class DroneController : public rclcpp::Node {
// public:
//     DroneController() : Node("drone_controller_cpp") {
//         // Publisher para velocidade
//         vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
//             "/uav1/mavros/setpoint_velocity/cmd_vel", 10);

//         // Clientes para serviços do MAVROS
//         mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
//         arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

//         // Loop de controle a 10Hz
//         timer_ = this->create_wall_timer(100ms, std::bind(&DroneController::control_loop, this));
        
//         RCLCPP_INFO(this->get_logger(), "No de controle iniciado!");
//     }

// private:
//     void control_loop() {
//         // 1. Publica velocidade constantemente (obrigatório para manter o OFFBOARD ativo)
//         auto vel_msg = geometry_msgs::msg::TwistStamped();
//         vel_msg.header.stamp = this->get_clock()->now();
//         vel_msg.header.frame_id = "uav1/fcu";
//         vel_msg.twist.linear.x = 1.0; 
//         vel_msg.twist.linear.y = 0.0;
//         vel_msg.twist.linear.z = 0.0;
//         vel_pub_->publish(vel_msg);

//         // 2. Lógica de Ativação (Tenta armar e colocar em OFFBOARD nas primeiras iterações)
//         static int retry_count = 0;
//         if (retry_count < 10) { 
//             if (arm_client_->service_is_ready() && mode_client_->service_is_ready()) {
//                 arm_drone(true);
//                 set_offboard_mode();
//                 retry_count++;
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Aguardando serviços do MAVROS ficarem prontos...");
//             }
//         }
//     }

//     void set_offboard_mode() {
//         auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
//         request->custom_mode = "OFFBOARD";
//         mode_client_->async_send_request(request);
//         RCLCPP_INFO(this->get_logger(), "Solicitando modo OFFBOARD...");
//     }

//     void arm_drone(bool arm) {
//         auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//         request->value = arm;
//         arm_client_->async_send_request(request);
//         RCLCPP_INFO(this->get_logger(), arm ? "Solicitando ARMING..." : "Solicitando DISARMING...");
//     }

//     // Declaração dos membros da classe
//     rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
//     rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
//     rclcpp::Client<mavros_msgs::srv::CommandBool>::