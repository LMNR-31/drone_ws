#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp" 

#include "rtwtypes.h"
#include "coder_array.h"
#include "main_codegen.h"
#include "Drone_codegen.h"
#include "TrajectoryPlanner_codegen.h"
#include <vector>

using namespace std::chrono_literals;

class DroneNode : public rclcpp::Node {
public:
    DroneNode() : Node("drone_integrated_controller") {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

        // Lê o estado do drone em tempo real
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/uav1/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                current_state_ = *msg;
            });

        // ==========================================
        // CONFIGURAÇÃO DA IA: PONTOS DINÂMICOS
        // ==========================================
        int n = 7; 
        
        std::vector<double> pontos_X = {0.0, 3.0, 5.0, 6.0, 8.0, 10.0, 12.0, 0.0};
        std::vector<double> pontos_Y = {0.0, 3.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0};
        std::vector<double> pontos_Z = {2.0, 3.0, 4.0, 3.0, 2.0, 2.0, 4.0, 3.0};

        total_segmentos_ = n; 
        planner_obj_.numSegments = n;

        planner_obj_.waypoints.clear();
        planner_obj_.waypoints.insert(planner_obj_.waypoints.end(), pontos_X.begin(), pontos_X.end());
        planner_obj_.waypoints.insert(planner_obj_.waypoints.end(), pontos_Y.begin(), pontos_Y.end());
        planner_obj_.waypoints.insert(planner_obj_.waypoints.end(), pontos_Z.begin(), pontos_Z.end());

        planner_obj_.segmentTimes.clear();
        for(int i = 0; i < n; i++) {
            planner_obj_.segmentTimes.push_back(5.0); 
        }

        // ==========================================
        // INICIALIZAÇÃO
        // ==========================================
        planner_ptr_ = planner_obj_.init();
        drone_ptr_   = drone_obj_.init();
        
        sim_time_ = 0.0;
        cycle_count_ = 0;
        estado_voo_ = 0; 
        current_segment_ = 0; 
        hover_counter_ = 0;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&DroneNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Controlador Inteligente Iniciado ===");
    }

private:
    void control_loop() {
        cycle_count_++;
        auto now = this->now();
        
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "map";

        // Inicializamos essas variáveis explicitamente para evitar lixo de memória e segmentation fault
        double Xd[3] = {0.0, 0.0, 2.0}; 
        double Vd[3] = {0.0, 0.0, 0.0}; 
        double Ad[3] = {0.0, 0.0, 0.0};

        // ==========================================
        // ESTADO 0: DECOLAGEM INTELIGENTE + ANTI-PÂNICO
        // ==========================================
        if (estado_voo_ == 0) {
            
            // FASE 1: Aguarda Conexão e Offboard/Arm
            if (current_state_.mode != "OFFBOARD" || !current_state_.armed) {
                // Envia continuamente setpoints em Z=0 para o PX4 aceitar o comando offboard
                pose_msg.pose.position.x = 0.0;
                pose_msg.pose.position.y = 0.0;
                pose_msg.pose.position.z = 0.0;
                pose_pub_->publish(pose_msg);

                // Dispara requisições a cada 50 ciclos para não sobrecarregar o serviço
                if (cycle_count_ % 50 == 0) {
                    if (current_state_.mode != "OFFBOARD") {
                        auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                        mode_req->custom_mode = "OFFBOARD";
                        mode_client_->async_send_request(mode_req);
                        RCLCPP_INFO(this->get_logger(), "Solicitando OFFBOARD...");
                    } 
                    else if (!current_state_.armed) {
                        auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                        arm_req->value = true;
                        arm_client_->async_send_request(arm_req);
                        RCLCPP_INFO(this->get_logger(), "Solicitando ARM...");
                    }
                }
            } 
            // FASE 2: Decolagem suave após armar
            else {
                // Sobe para 2.0m mantendo x e y em 0
                pose_msg.pose.position.x = 0.0;
                pose_msg.pose.position.y = 0.0;
                pose_msg.pose.position.z = 2.0; 
                pose_pub_->publish(pose_msg);
                
                hover_counter_++;
                
                if (hover_counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "Aguardando drone decolar e estabilizar...");
                }
                
                // Aguarda 800 ciclos (8 segundos) para estabilizar no ar de forma segura
                if (hover_counter_ > 800) {
                    estado_voo_ = 1;
                    hover_counter_ = 0; 
                    // Garante que o relógio da simulação para a trajetória inicie zerado
                    sim_time_ = 0.0; 
                    RCLCPP_INFO(this->get_logger(), "Decolagem concluída! Iniciando Trajetória do MATLAB!");
                }
            }
        }
        // ==========================================
        // ESTADO 1: VOANDO PELA TRAJETÓRIA
        // ==========================================
        else if (estado_voo_ == 1) {
            // Chamada segura para o código gerado pelo MATLAB
            planner_obj_.getNextSetpoint(sim_time_, Xd, Vd, Ad);
            pose_msg.pose.position.x = Xd[0];
            pose_msg.pose.position.y = Xd[1];
            pose_msg.pose.position.z = Xd[2];
            pose_pub_->publish(pose_msg);

            // Incrementa o tempo simulado da trajetória (depende do loop rate de 10ms)
            sim_time_ += 0.01;

            if (cycle_count_ % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Tempo: %.2f | X: %.2f | Y: %.2f | Z: %.2f", sim_time_, Xd[0], Xd[1], Xd[2]);
            }

            // Verifica se o tempo atual já cobriu o segmento atual (cada segmento tem 5.0s)
            if (sim_time_ >= (current_segment_ + 1) * 5.0) {
                estado_voo_ = 2; 
                hover_counter_ = 0;
                RCLCPP_INFO(this->get_logger(), "--> Chegou no Ponto %d! Iniciando Hover...", current_segment_ + 1);
            }
        }
        // ==========================================
        // ESTADO 2: HOVER NO WAYPOINT
        // ==========================================
        else if (estado_voo_ == 2) {
            // Mantém a última posição calculada pela trajetória
            planner_obj_.getNextSetpoint(sim_time_, Xd, Vd, Ad);
            pose_msg.pose.position.x = Xd[0];
            pose_msg.pose.position.y = Xd[1];
            pose_msg.pose.position.z = Xd[2];
            pose_pub_->publish(pose_msg);

            hover_counter_++; 

            // Faz hover por 3 segundos (300 ciclos)
            if (hover_counter_ >= 300) { 
                current_segment_++; 
                
                if (current_segment_ >= total_segmentos_) { 
                    estado_voo_ = 3;
                    RCLCPP_INFO(this->get_logger(), "--> Trajetória Concluída! Mantendo posição final.");
                } else {
                    estado_voo_ = 1; 
                    RCLCPP_INFO(this->get_logger(), "Hover concluído. Indo para o próximo Ponto...");
                }
            }
        }
        // ==========================================
        // ESTADO 3: FIM
        // ==========================================
        else if (estado_voo_ == 3) {
            planner_obj_.getNextSetpoint(sim_time_, Xd, Vd, Ad);
            pose_msg.pose.position.x = Xd[0];
            pose_msg.pose.position.y = Xd[1];
            pose_msg.pose.position.z = Xd[2];
            pose_pub_->publish(pose_msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    mavros_msgs::msg::State current_state_;

    rclcpp::TimerBase::SharedPtr timer_;

    Drone_codegen drone_obj_;
    Drone_codegen *drone_ptr_;
    TrajectoryPlanner_codegen planner_obj_;
    TrajectoryPlanner_codegen *planner_ptr_;
    
    double sim_time_;
    int cycle_count_;
    int estado_voo_;
    int current_segment_; 
    int hover_counter_;   
    int total_segmentos_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}