#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp" 
#include <vector>
#include <cmath>
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

    waypoint_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/waypoint_goal", 1,
      std::bind(&DroneControllerCompleto::waypoint_goal_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "✓ Subscribers criados: /uav1/mavros/state, /waypoints e /waypoint_goal");

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
    offboard_activated_ = false;
    activation_confirmed_ = false;
    takeoff_counter_ = 0;
    waypoint_goal_received_ = false;
    last_waypoint_goal_.pose.position.x = 0.0;
    last_waypoint_goal_.pose.position.y = 0.0;
    last_waypoint_goal_.pose.position.z = 2.0; // Hover de segurança
    trajectory_setpoint_[0] = 0.0;
    trajectory_setpoint_[1] = 0.0;
    trajectory_setpoint_[2] = 2.0;

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
  // SOLICITA DISARM (DESARMAR DRONE)
  // ==========================================
  void request_disarm() {
    if (!arm_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false; // false = desarmar

    arm_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "🔴 Solicitando DISARM...");
  }

  // ==========================================
  // CALLBACK: RECEBE WAYPOINTS
  // ==========================================
  void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // ✅ VALIDAÇÃO: Mínimo 1 waypoint
    if (msg->poses.size() < 1) {
      RCLCPP_WARN(this->get_logger(), "❌ Waypoints insuficientes: %zu", msg->poses.size());
      return;
    }

    double last_z = msg->poses.back().position.z;

    // ==========================================
    // DETECTA POUSO (Z < 0.5)
    // ==========================================
    if (last_z < 0.5) {
      RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z_final = %.2f m", last_z);
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4; // ✅ VAI DIRETO PARA ESTADO 4!
      RCLCPP_WARN(this->get_logger(), "🛬 CONTROLADOR DESLIGADO - DEIXANDO drone_soft_land POUSAR\n");
      return;
    }

    // ==========================================
    // ESTRATÉGIA 1: 1 WAYPOINT = LEVANTAMENTO
    // ==========================================
    if (msg->poses.size() == 1 && last_z >= 0.5) {
      RCLCPP_INFO(this->get_logger(), "\n⬆️ WAYPOINT DE LEVANTAMENTO recebido:");
      RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.2f",
        msg->poses[0].position.x,
        msg->poses[0].position.y,
        msg->poses[0].position.z);

      last_waypoint_goal_.pose = msg->poses[0];

      // ✅ RESET: Limpa flags do ciclo anterior (CRUCIAL!)
      pouso_em_andamento_ = false;  // ✅ Drone NÃO está mais pousando
      controlador_ativo_ = false;   // ✅ Reseta controlador para novo ciclo
      state_voo_ = 0;               // ✅ Volta para ESTADO 0 (ativação)

      if (!offboard_activated_) {
        RCLCPP_INFO(this->get_logger(), "🔋 Ativando OFFBOARD+ARM para levantamento...\n");
        request_offboard();
        request_arm();
        offboard_activated_ = true;
        activation_time_ = this->now();
      }

      // ✅ NÃO define state_voo_ = 1 aqui!
      // Deixa ESTADO 0 (control_loop) confirmar OFFBOARD+ARMED e então transicionar para ESTADO 1

      return;
    }

    // ==========================================
    // ESTRATÉGIA 2: 2+ WAYPOINTS = TRAJETÓRIA
    // ==========================================
    if (msg->poses.size() >= 2 && last_z >= 0.5) {
      RCLCPP_INFO(this->get_logger(), "\n✈️ WAYPOINTS DE TRAJETÓRIA recebidos: %zu pontos", msg->poses.size());
      for (size_t i = 0; i < msg->poses.size(); i++) {
        RCLCPP_INFO(this->get_logger(),
          "   WP[%zu]: X=%.2f, Y=%.2f, Z=%.2f",
          i,
          msg->poses[i].position.x,
          msg->poses[i].position.y,
          msg->poses[i].position.z);
      }
      RCLCPP_INFO(this->get_logger(), " ");

      // ✅ Primeira posição = posição de levantamento (para OFFSET)
      last_waypoint_goal_.pose = msg->poses[0];

      // ✅ Processa trajetória e ativa controlador
      controlador_ativo_ = true;
      pouso_em_andamento_ = false;
      state_voo_ = 3; // ✅ ESTADO 3: TRAJETÓRIA

      RCLCPP_INFO(this->get_logger(), "✅ Trajetória ativada - Entrando em ESTADO 3\n");
      return;
    }
  }

  // ==========================================
  // CALLBACK: RECEBE WAYPOINT ÚNICO (PoseStamped)
  // ==========================================
  void waypoint_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // ✅ DETECTA POUSO
    if (z < 0.5) {
      RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z = %.2f m", z);
      pouso_em_andamento_ = true;
      controlador_ativo_ = false;
      state_voo_ = 4;
      RCLCPP_WARN(this->get_logger(), "🛬 CONTROLADOR DESLIGADO - DEIXANDO drone_soft_land POUSAR\n");
      return;
    }

    // ✅ DETECTA DISARM em ESTADO 4
    // Quando drone é desarmado após pouso, reseta flags para novo ciclo
    if (state_voo_ == 4 && !current_state_.armed) {
      RCLCPP_INFO(this->get_logger(), "✅ DRONE DESARMADO! Pronto para novo ciclo");

      // ✅ RESETAR FLAGS PARA NOVO CICLO
      offboard_activated_ = false;
      activation_confirmed_ = false;
      state_voo_ = 0;
      takeoff_counter_ = 0;
      pouso_em_andamento_ = false;
      return;
    }

    // ✅ Em ESTADO 3 (trajetória), armazena setpoints relativos em trajectory_setpoint_
    // para que last_waypoint_goal_ permaneça como posição de hover (offset)
    if (state_voo_ == 3) {
      trajectory_setpoint_[0] = x;  // X relativo da trajetória
      trajectory_setpoint_[1] = y;  // Y relativo da trajetória
      trajectory_setpoint_[2] = z;  // Z absoluto da trajetória
      controlador_ativo_ = true;
      pouso_em_andamento_ = false;
      return;
    }

    // ✅ IGNORA DUPLICATAS
    const double eps = 1e-9;
    if (waypoint_goal_received_ &&
        std::abs(x - last_waypoint_goal_.pose.position.x) < eps &&
        std::abs(y - last_waypoint_goal_.pose.position.y) < eps &&
        std::abs(z - last_waypoint_goal_.pose.position.z) < eps) {
      RCLCPP_DEBUG(this->get_logger(), "⚠️ Waypoint duplicado ignorado: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);
      return;
    }

    // ✅ NOVO WAYPOINT RECEBIDO
    RCLCPP_INFO(this->get_logger(), "\n📍 NOVO WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

    last_waypoint_goal_ = *msg;
    waypoint_goal_received_ = true;

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
    // ESTADO 0: AGUARDANDO ATIVAÇÃO OFFBOARD+ARM
    // ==========================================
    if (state_voo_ == 0) {

      // ✅ Sempre publica setpoints para manter OFFBOARD ativo
      pose_msg.pose.position.x = last_waypoint_goal_.pose.position.x;
      pose_msg.pose.position.y = last_waypoint_goal_.pose.position.y;
      pose_msg.pose.position.z = last_waypoint_goal_.pose.position.z;
      pose_pub_->publish(pose_msg);

      // ✅ Verifica se OFFBOARD+ARM foram confirmados
      if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
        if (!activation_confirmed_) {
          RCLCPP_INFO(this->get_logger(), "✅ DRONE ATIVADO! (OFFBOARD+ARMED)");
          RCLCPP_INFO(this->get_logger(), "⬆️ Iniciando decolagem para 2.0m...\n");
          activation_confirmed_ = true;
        }
        state_voo_ = 1;
        takeoff_counter_ = 0;
        return;
      }

      // ⏳ Timeout: se não ativar em 5 segundos, tenta novamente
      if (offboard_activated_ && (this->now() - activation_time_).seconds() > 5.0) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Ativação pendente (OFFBOARD=%s | ARMED=%s), tentando novamente...",
          current_state_.mode == "OFFBOARD" ? "✓" : "✗",
          current_state_.armed ? "✓" : "✗");
        request_offboard();
        request_arm();
        activation_time_ = this->now();
      }

      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "⏳ Aguardando waypoints de levantamento (Z > 0.5)...");
      }
    }

    // ==========================================
    // ESTADO 1: DECOLAGEM
    // ==========================================
    else if (state_voo_ == 1) {

      // ✅ Sempre publica setpoints
      pose_msg.pose.position.x = last_waypoint_goal_.pose.position.x;
      pose_msg.pose.position.y = last_waypoint_goal_.pose.position.y;
      pose_msg.pose.position.z = 2.0; // Levanta para 2m
      pose_pub_->publish(pose_msg);

      takeoff_counter_++;

      if (takeoff_counter_ == 1) {
        RCLCPP_INFO(this->get_logger(), "⬆️ Decolando para 2.0 metros...");
        RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=2.0",
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y);
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
      pose_msg.pose.position.x = last_waypoint_goal_.pose.position.x;
      pose_msg.pose.position.y = last_waypoint_goal_.pose.position.y;
      pose_msg.pose.position.z = 2.0;
      pose_pub_->publish(pose_msg);

      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "🛸 Em HOVER (2.0m) | Posição: X=%.2f, Y=%.2f | Aguardando waypoints... Controlador: %s",
          last_waypoint_goal_.pose.position.x,
          last_waypoint_goal_.pose.position.y,
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

      // ✅ OFFSET: Trajetória começa da posição de levantamento (hover)
      double offset_x = last_waypoint_goal_.pose.position.x;
      double offset_y = last_waypoint_goal_.pose.position.y;

      // ✅ Setpoint relativo (da trajetória) + offset = setpoint absoluto
      pose_msg.pose.position.x = offset_x + trajectory_setpoint_[0];
      pose_msg.pose.position.y = offset_y + trajectory_setpoint_[1];
      pose_msg.pose.position.z = trajectory_setpoint_[2];
      pose_pub_->publish(pose_msg);

      if (cycle_count_ % 500 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "📡 Trajetória em execução | X: %.2fm, Y: %.2fm, Z: %.2fm (offset: %.2f, %.2f)",
          pose_msg.pose.position.x,
          pose_msg.pose.position.y,
          pose_msg.pose.position.z,
          offset_x,
          offset_y);
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
      // Se recebeu novos waypoints com Z > 0.5, solicita DISARM e aguarda confirmação
      if (!pouso_em_andamento_ && controlador_ativo_ && current_state_.armed) {
        RCLCPP_WARN(this->get_logger(), "\n✅ POUSO CONCLUÍDO! VOLTANDO A VOAR!");
        RCLCPP_WARN(this->get_logger(), "⬆️ Iniciando nova decolagem...\n");

        // ✅ DISARM ANTES DE NOVO CICLO (CRUCIAL!)
        // Flags serão resetadas em waypoint_goal_callback() quando DISARM for confirmado
        request_disarm();
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_goal_sub_;

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
  bool offboard_activated_;          // Ativação OFFBOARD+ARM foi disparada por waypoints?
  bool activation_confirmed_;        // OFFBOARD+ARM confirmados?
  rclcpp::Time activation_time_;     // Timestamp da última solicitação de ativação
  int cycle_count_;                  // Contador de ciclos
  int takeoff_counter_;              // Contador de decolagem

  // Rastreamento do último waypoint único recebido (para ignorar duplicatas)
  geometry_msgs::msg::PoseStamped last_waypoint_goal_;
  bool waypoint_goal_received_;      // Recebeu ao menos um waypoint_goal?

  // Setpoints da trajetória (coordenadas relativas ao ponto de hover)
  double trajectory_setpoint_[3];

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