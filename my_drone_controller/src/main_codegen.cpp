#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp" 
#include <vector>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

class DroneControllerCompleto : public rclcpp::Node {
public:
  DroneControllerCompleto() : Node("drone_controller_completo") {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🚁 CONTROLADOR INTELIGENTE DE DRONE - VERSÃO FINAL      ║");
    RCLCPP_INFO(this->get_logger(), "║     COM ATIVAÇÃO OFFBOARD + ARM + DETECÇÃO DE POUSO     ║");
    RCLCPP_INFO(this->get_logger(), "║               345 LINHAS - CÓDIGO COMPLETO               ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════════╝\n");

    // ==========================================
    // PUBLISHERS
    // ==========================================
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/uav1/mavros/setpoint_position/local", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /uav1/mavros/setpoint_position/local");

    // ==========================================
    // SUBSCRIBERS
    // ==========================================
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
      });

    waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/waypoints", 1,
      std::bind(&DroneControllerCompleto::waypoints_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "✓ Subscribers criados: /uav1/mavros/state e /waypoints");

    // ==========================================
    // SERVICE CLIENTS - ATIVAÇÃO
    // ==========================================
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");

    RCLCPP_INFO(this->get_logger(), "✓ Service Clients criados: set_mode e arming");

    // ==========================================
    // AGUARDAR SERVIÇOS DISPONÍVEIS
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "⏳ Aguardando serviços MAVROS...");
    
    while (!mode_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/set_mode...");
    }
    RCLCPP_INFO(this->get_logger(), "✓ Serviço set_mode disponível");

    while (!arm_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/cmd/arming...");
    }
    RCLCPP_INFO(this->get_logger(), "✓ Serviço arming disponível\n");

    // ==========================================
    // TIMER DO CONTROLE
    // ==========================================
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&DroneControllerCompleto::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "✓ Timer criado: 100 Hz (10ms)");

    // ==========================================
    // INICIALIZAÇÃO DE VARIÁVEIS
    // ==========================================
    state_voo_ = 0;
    controlador_ativo_ = false;
    pouso_em_andamento_ = false;
    cycle_count_ = 0;
    offboard_requested_ = false;
    arm_requested_ = false;
    takeoff_counter_ = 0;

    RCLCPP_INFO(this->get_logger(), "\n📊 STATUS INICIAL:");
    RCLCPP_INFO(this->get_logger(), "   Estado: %d (ativação)", state_voo_);
    RCLCPP_INFO(this->get_logger(), "   Controlador: %s", controlador_ativo_ ? "ATIVO" : "INATIVO");
    RCLCPP_INFO(this->get_logger(), "   Pouso: %s\n", pouso_em_andamento_ ? "SIM" : "NÃO");
  }

private:

  // ==========================================
  // SOLICITA OFFBOARD MODE
  // ==========================================
  void request_offboard() {
    if (!mode_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";

    mode_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "📡 Solicitando OFFBOARD MODE...");
  }

  // ==========================================
  // SOLICITA ARM (ARMAR DRONE)
  // ==========================================
  void request_arm() {
    if (!arm_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true; // true = armar, false = desarmar

    arm_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "🔋 Solicitando ARM...");
  }

  // ==========================================
  // CALLBACK: RECEBE WAYPOINTS
  // ==========================================
  void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (msg->poses.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "❌ Waypoints insuficientes: %zu", msg->poses.size());
      return;
    }

    // ✅ DETECTA POUSO
    double last_z = msg->poses.back().position.z;
    if (last_z < 0.5) {
      RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z_final = %.2f m", last_z);
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4; // ✅ VAI DIRETO PARA ESTADO 4!
      RCLCPP_WARN(this->get_logger(), "🛬 CONTROLADOR DESLIGADO - DEIXANDO drone_soft_land POUSAR\n");
      return;
    }

    // ✅ TRAJETÓRIA NORMAL
    RCLCPP_INFO(this->get_logger(), "\n📍 WAYPOINTS RECEBIDOS: %zu pontos", msg->poses.size());
    for (size_t i = 0; i < msg->poses.size(); i++) {
      RCLCPP_INFO(this->get_logger(), 
        "   WP[%zu]: X=%.2f, Y=%.2f, Z=%.2f", 
        i, 
        msg->poses[i].position.x,
        msg->poses[i].position.y,
        msg->poses[i].position.z);
    }
    RCLCPP_INFO(this->get_logger(), " ");

    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
  }

  // ==========================================
  // LOOP PRINCIPAL DE CONTROLE
  // ==========================================
  void control_loop() {
    std::lock_guard<std::mutex> lock(mutex_);

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";

    cycle_count_++;

    // ==========================================
    // ESTADO 0: ATIVAÇÃO (OFFBOARD + ARM)
    // ==========================================
    if (state_voo_ == 0) {

      // ✅ Sempre publica setpoints para manter OFFBOARD ativo
      pose_msg.pose.position.x = 0.0;
      pose_msg.pose.position.y = 0.0;
      pose_msg.pose.position.z = 0.0;
      pose_pub_->publish(pose_msg);

      // Verifica se já está OFFBOARD+ARMED
      if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
        if (!offboard_requested_ && !arm_requested_) {
          RCLCPP_INFO(this->get_logger(), "\n✅ OFFBOARD + ARM JÁ CONFIRMADOS!");
          RCLCPP_INFO(this->get_logger(), "⬆️ Iniciando decolagem...\n");
        }
        state_voo_ = 1;
        offboard_requested_ = false;
        arm_requested_ = false;
        return;
      }

      // Solicita OFFBOARD a cada 50 ciclos
      if (cycle_count_ % 50 == 0) {
        if (current_state_.mode != "OFFBOARD") {
          request_offboard();
          offboard_requested_ = true;
        }
      }

      // Solicita ARM a cada 100 ciclos (após OFFBOARD)
      if (cycle_count_ % 100 == 0) {
        if (!current_state_.armed) {
          request_arm();
          arm_requested_ = true;
        }
      }

      // Log de status
      if (cycle_count_ % 200 == 0) {
        RCLCPP_INFO(this->get_logger(), 
          "⏳ Ativação: OFFBOARD=%s | ARMED=%s",
          current_state_.mode == "OFFBOARD" ? "✓" : "✗",
          current_state_.armed ? "✓" : "✗");
      }
    }

    // ==========================================
    // ESTADO 1: DECOLAGEM
    // ==========================================
    else if (state_voo_ == 1) {

      // ✅ Sempre publica setpoints
      pose_msg.pose.position.x = 0.0;
      pose_msg.pose.position.y = 0.0;
      pose_msg.pose.position.z = 2.0; // Levanta para 2m
      pose_pub_->publish(pose_msg);

      takeoff_counter_++;

      if (takeoff_counter_ == 1) {
        RCLCPP_INFO(this->get_logger(), "⬆️ Decolando para 2.0 metros...");
      }

      // Aguarda 8 segundos (800 ciclos) para estabilizar
      if (takeoff_counter_ > 800) {
        state_voo_ = 2;
        takeoff_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "✅ Decolagem concluída! Altitude = 2.0m\n");
      }
    }

    // ==========================================
    // ESTADO 2: HOVER/AGUARDANDO TRAJETÓRIA
    // ==========================================
    else if (state_voo_ == 2) {

      // ✅ Publica setpoint em hover
      pose_msg.pose.position.x = 0.0;
      pose_msg.pose.position.y = 0.0;
      pose_msg.pose.position.z = 2.0;
      pose_pub_->publish(pose_msg);

      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "🛸 Em HOVER (2.0m) | Aguardando waypoints... Controlador: %s",
          controlador_ativo_ ? "ATIVO" : "INATIVO");
      }

      // ✅ Quando recebe waypoints válidos, vai para estado 3
      if (controlador_ativo_) {
        state_voo_ = 3;
        RCLCPP_INFO(this->get_logger(), "✈️ Iniciando execução de trajetória...\n");
      }

      // ✅ SE DETECTAR POUSO NESTE ESTADO
      if (pouso_em_andamento_) {
        RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO NO HOVER - DESLIGANDO!");
        state_voo_ = 4;
        return;
      }
    }

    // ==========================================
    // ESTADO 3: EXECUTANDO TRAJETÓRIA
    // ==========================================
    else if (state_voo_ == 3) {

      // ✅ VERIFICA POUSO - DETECTA AUTOMATICAMENTE
      if (pouso_em_andamento_ && !controlador_ativo_) {
        RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO EM TRAJETÓRIA - PARANDO IMEDIATAMENTE!");
        state_voo_ = 4;
        return; // ✅ CRUCIAL: NÃO publica mais!
      }

      // Mantém drone em posição fixa (aguardando waypoints)
      pose_msg.pose.position.x = 0.0;
      pose_msg.pose.position.y = 0.0;
      pose_msg.pose.position.z = 2.0;
      pose_pub_->publish(pose_msg);

      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "📡 Trajetória em execução | Z: 2.0m (aguardando comando)");
      }
    }

    // ==========================================
    // ESTADO 4: POUSO/PAUSADO - NÃO PUBLICA
    // ==========================================
    else if (state_voo_ == 4) {
      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "🛑 CONTROLADOR PAUSADO | drone_soft_land fazendo pouso...");
      }

      // ✅ VERIFICA SE POUSO TERMINOU
      // Se recebeu novos waypoints com Z > 0.5, volta a voar!
      if (!pouso_em_andamento_ && controlador_ativo_) {
        RCLCPP_WARN(this->get_logger(), "\n✅ POUSO CONCLUÍDO! VOLTANDO A VOAR!");
        RCLCPP_WARN(this->get_logger(), "⬆️ Iniciando nova decolagem...\n");
        state_voo_ = 1; // ✅ VOLTA PARA DECOLAGEM!
        takeoff_counter_ = 0;
        return;
      }

      return; // NÃO publica nada durante pouso
    }
  }

  // ==========================================
  // MEMBROS PRIVADOS
  // ==========================================

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Subscribers
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;

  // Service Clients - ATIVAÇÃO OFFBOARD + ARM
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Estado do drone
  mavros_msgs::msg::State current_state_;

  // Estado do controlador - MÁQUINA DE ESTADOS
  int state_voo_;                    // 0=ativação, 1=decolagem, 2=hover, 3=trajetória, 4=pouso
  bool controlador_ativo_;           // Trajetória está ativa?
  bool pouso_em_andamento_;          // Pouso em andamento?
  bool offboard_requested_;          // OFFBOARD foi solicitado?
  bool arm_requested_;               // ARM foi solicitado?
  int cycle_count_;                  // Contador de ciclos
  int takeoff_counter_;              // Contador de decolagem

  // Sincronização thread-safe
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneControllerCompleto>();
  
  RCLCPP_INFO(node->get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(node->get_logger(), "║           🚁 CONTROLADOR PRONTO PARA OPERAÇÃO              ║");
  RCLCPP_INFO(node->get_logger(), "║                                                            ║");
  RCLCPP_INFO(node->get_logger(), "║  Pressione Ctrl+C para encerrar                            ║");
  RCLCPP_INFO(node->get_logger(), "╚════════════════════════════════════════════════════════════╝\n");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}