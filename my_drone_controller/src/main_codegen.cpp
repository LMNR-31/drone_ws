#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&DroneControllerCompleto::odometry_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "✓ Subscribers criados: /uav1/mavros/state, /waypoints, /waypoint_goal e odometria");

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
    last_z_ = 0.0;
    pouso_start_time_set_ = false;
    pouso_start_time_ = this->now();
    last_waypoint_goal_.pose.position.x = 0.0;
    last_waypoint_goal_.pose.position.y = 0.0;
    last_waypoint_goal_.pose.position.z = 2.0; // Hover de segurança
    trajectory_setpoint_[0] = 0.0;
    trajectory_setpoint_[1] = 0.0;
    trajectory_setpoint_[2] = 2.0;
    trajectory_waypoints_.clear();
    trajectory_started_ = false;
    current_waypoint_idx_ = 0;
    waypoint_duration_ = 4.0;
    current_z_real_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "\n📊 STATUS INICIAL:");
    RCLCPP_INFO(this->get_logger(), "   Estado: %d (aguardando waypoint)", state_voo_);
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
  // CALLBACK: ODOMETRIA (Z REAL DO DRONE)
  // ==========================================
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // ✅ CORRIGIR: /uav1/mavros/local_position/odom usa NED (North-East-Down)
    // Z negativo em NED = altura (Z=-2.0 significa 2 metros de altura)
    // Usamos MÓDULO (valor absoluto) para ter altura sempre positiva
    // |Z| = altura em metros (0 no solo, 2.0 em hover)
    current_z_real_ = std::abs(msg->pose.pose.position.z);
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
    if (msg->poses.size() == 1 && last_z < 0.5) {
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

      // ✅ NOVO! Reset adicional de trajetória
      trajectory_started_ = false;
      trajectory_waypoints_.clear();
      current_waypoint_idx_ = 0;

      // ✅ Log de debug ANTES da verificação
      RCLCPP_INFO(this->get_logger(), "🔍 DEBUG FLAGS ANTES:");
      RCLCPP_INFO(this->get_logger(), "   offboard_activated_=%d", offboard_activated_);
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d", state_voo_);
      RCLCPP_INFO(this->get_logger(), "   activation_confirmed_=%d", activation_confirmed_);

      // ✅ SEMPRE força reativação, independentemente do estado anterior
      RCLCPP_INFO(this->get_logger(), "🔋 Ativando OFFBOARD+ARM para levantamento...\n");

      // ✅ Reset explícito ANTES de reativar
      offboard_activated_ = false;
      activation_confirmed_ = false;

      // ✅ Solicita OFFBOARD MODE e ARM
      request_offboard();
      request_arm();

      // ✅ Marca como ativado e aguarda confirmação do FCU
      offboard_activated_ = true;
      activation_time_ = this->now();

      // ✅ Vai direto para ESTADO 1 (decolagem) aguardando OFFBOARD+ARM confirmados
      state_voo_ = 1;
      takeoff_counter_ = 0;

      // ✅ Log de debug DEPOIS da verificação
      RCLCPP_INFO(this->get_logger(), "🔍 DEBUG FLAGS DEPOIS:");
      RCLCPP_INFO(this->get_logger(), "   offboard_activated_=%d", offboard_activated_);
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d\n", state_voo_);

      return;
    }

    // ==========================================
    // ESTRATÉGIA 2: 2+ WAYPOINTS = TRAJETÓRIA
    // (COM ou SEM descida de pouso)
    // ==========================================
    if (msg->poses.size() >= 2) {

      RCLCPP_INFO(this->get_logger(), "🔍 Trajetória (2+ waypoints) recebida");
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d (esperado 2 para ativar)", state_voo_);

      // ✅ SEMPRE armazena waypoints (mesmo se ESTADO != 2)
      trajectory_waypoints_ = msg->poses;
      current_waypoint_idx_ = 0;
      trajectory_started_ = false;
      last_waypoint_goal_.pose = msg->poses[0];

      RCLCPP_INFO(this->get_logger(), "✈️ WAYPOINTS DE TRAJETÓRIA armazenados: %zu pontos", msg->poses.size());
      for (size_t i = 0; i < msg->poses.size(); i++) {
        RCLCPP_INFO(this->get_logger(),
          "   WP[%zu]: X=%.2f, Y=%.2f, Z=%.2f",
          i,
          msg->poses[i].position.x,
          msg->poses[i].position.y,
          msg->poses[i].position.z);
      }
      RCLCPP_INFO(this->get_logger(), " ");

      // ✅ Se NÃO em HOVER (ESTADO 2), apenas armazena e aguarda
      if (state_voo_ != 2) {
        RCLCPP_INFO(this->get_logger(),
          "⏸️ Trajetória armazenada - Será ativada quando drone chegar em HOVER (ESTADO 2)");
        controlador_ativo_ = false;  // ← Ainda NÃO ativa!
        pouso_em_andamento_ = false;
        return;  // ← Retorna mas WAYPOINTS estão armazenados!
      }

      // ✅ Se EM HOVER (ESTADO 2), ativa trajetória IMEDIATAMENTE
      RCLCPP_INFO(this->get_logger(), "\n✅ TRAJETÓRIA ACEITA E ATIVADA! Drone em HOVER pronto!\n");

      controlador_ativo_ = true;   // ← Ativa agora!
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

    last_z_ = z;

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
    // ESTADO 0: AGUARDANDO NOVO WAYPOINT
    // ==========================================
    if (state_voo_ == 0) {

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "⏳ Aguardando novo comando de waypoint para decolar...");
      // ✅ Não faz nada! Apenas aguarda novo waypoint em waypoints_callback()
    }

    // ==========================================
    // ESTADO 1: DECOLAGEM
    // ==========================================
    else if (state_voo_ == 1) {

      // ✅ VERIFICAÇÃO DE SEGURANÇA: Se chegou em ESTADO 1 mas OFFBOARD não foi ativado, ativar!
      if (!offboard_activated_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ ESTADO 1 SEM OFFBOARD ATIVADO! Forçando ativação...");
        request_offboard();
        request_arm();
        offboard_activated_ = true;
        activation_time_ = this->now();
      }

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

      // ✅ Verifica se drone chegou em Z ~= 2.0m usando odometria real
      // Margem de 0.05m abaixo do alvo (1.95m) para ter segurança
      if (current_z_real_ >= 1.95) {
        RCLCPP_INFO(this->get_logger(), "✅ Decolagem concluída! Altitude = %.2fm\n", current_z_real_);
        state_voo_ = 2;
        takeoff_counter_ = 0;
        return;
      }

      // ✅ Log de debug a cada 100 ciclos (1 segundo @ 100Hz)
      if (takeoff_counter_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "📈 Decolando... Z_alvo=2.0m | Z_real=%.2fm | Tempo=%.1fs",
          current_z_real_,
          (double)takeoff_counter_ / 100.0);
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

      // ✅ NOVO! Detectar pouso durante trajetória (Z real < 0.5)
      if (current_z_real_ < 0.5) {
        RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO DURANTE TRAJETÓRIA! Z = %.2f m", current_z_real_);
        pouso_em_andamento_ = true;
        controlador_ativo_ = false;
        state_voo_ = 4;
        return;
      }

      // ✅ Detectar pouso automaticamente
      if (pouso_em_andamento_ && !controlador_ativo_) {
        RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO EM TRAJETÓRIA - PARANDO IMEDIATAMENTE!");
        state_voo_ = 4;
        return;
      }

      // ✅ INICIALIZAR trajetória na primeira execução
      if (!trajectory_started_) {
        if (trajectory_waypoints_.empty()) {
          RCLCPP_WARN(this->get_logger(), "⚠️ Nenhum waypoint armazenado, voltando para ESTADO 2");
          state_voo_ = 2;
          return;
        }

        trajectory_start_time_ = this->now();
        trajectory_started_ = true;
        current_waypoint_idx_ = 0;

        RCLCPP_INFO(this->get_logger(), "✈️ Trajetória iniciada! %zu waypoints | %.1f segundos cada",
          trajectory_waypoints_.size(), waypoint_duration_);
      }

      // ✅ CALCULAR qual waypoint publicar baseado no tempo transcorrido
      double elapsed_time = (this->now() - trajectory_start_time_).seconds();
      int computed_idx = static_cast<int>(elapsed_time / waypoint_duration_);

      // ✅ Limitar ao índice máximo (não ultrapassar último waypoint)
      current_waypoint_idx_ = std::min(computed_idx, static_cast<int>(trajectory_waypoints_.size()) - 1);

      // ✅ PUBLICAR waypoint ATUAL
      auto current_waypoint = trajectory_waypoints_[current_waypoint_idx_];

      pose_msg.pose.position.x = current_waypoint.position.x;
      pose_msg.pose.position.y = current_waypoint.position.y;
      pose_msg.pose.position.z = current_waypoint.position.z;
      pose_msg.pose.orientation.w = 1.0;
      pose_pub_->publish(pose_msg);

      // ✅ LOG: Mostrar progresso da trajetória
      if (cycle_count_ % 500 == 0) {
        double total_time = waypoint_duration_ * (double)trajectory_waypoints_.size();
        double progress_pct = (elapsed_time / total_time) * 100.0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "📡 Trajetória em execução | WP[%d/%zu]: X=%.2fm, Y=%.2fm, Z=%.2fm (Z_real=%.2f) | %.1f%% concluído",
          current_waypoint_idx_,
          trajectory_waypoints_.size() - 1,
          pose_msg.pose.position.x,
          pose_msg.pose.position.y,
          pose_msg.pose.position.z,
          current_z_real_,
          progress_pct);
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

      // ✅ TIMEOUT: Uma vez que pouso foi detectado, aguarda 3 segundos INDEPENDENTE de last_z_
      if (pouso_em_andamento_) {
        if (!pouso_start_time_set_) {
          pouso_start_time_ = this->now();
          pouso_start_time_set_ = true;
          RCLCPP_INFO(this->get_logger(), "⏱️ Iniciando contagem de pouso (3s para confirmar)...");
        }

        // ✅ Se passou 3 segundos desde que pouso foi detectado, pouso foi concluído!
        if ((this->now() - pouso_start_time_).seconds() > 3.0) {
          RCLCPP_WARN(this->get_logger(), "🔌 Solicitando DISARM...");

          // ✅ DISARM
          request_disarm();

          RCLCPP_WARN(this->get_logger(),
            "\n✅ POUSO CONCLUÍDO! Aguardando novo comando de waypoint para decolar novamente...\n");

          // ✅ Resetar TODAS as flags para estado limpo
          // ✅ CRUCIAL: offboard_activated_ DEVE ser false para próxima decolagem!
          state_voo_ = 0;
          pouso_em_andamento_ = false;
          controlador_ativo_ = false;
          trajectory_started_ = false;
          pouso_start_time_set_ = false;
          offboard_activated_ = false;
          activation_confirmed_ = false;
          takeoff_counter_ = 0;
          trajectory_waypoints_.clear();
          current_waypoint_idx_ = 0;

          // ✅ Log de verificação
          RCLCPP_WARN(this->get_logger(), "🔍 DEBUG RESET EM ESTADO 4:");
          RCLCPP_WARN(this->get_logger(), "   offboard_activated_=%d (deve ser 0)", offboard_activated_);
          RCLCPP_WARN(this->get_logger(), "   state_voo_=%d (deve ser 0)", state_voo_);

          return;
        }
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Service Clients - ATIVAÇÃO OFFBOARD + ARM
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Estado do drone
  mavros_msgs::msg::State current_state_;

  // Estado do controlador - MÁQUINA DE ESTADOS
  int state_voo_;                    // 0=aguardando waypoint, 1=decolagem, 2=hover, 3=trajetória, 4=pouso
  bool controlador_ativo_;           // Trajetória está ativa?
  bool pouso_em_andamento_;          // Pouso em andamento?
  bool offboard_activated_;          // Ativação OFFBOARD+ARM foi disparada por waypoints?
  bool activation_confirmed_;        // OFFBOARD+ARM confirmados?
  rclcpp::Time activation_time_;     // Timestamp da última solicitação de ativação
  int cycle_count_;                  // Contador de ciclos
  int takeoff_counter_;              // Contador de decolagem

  // Timeout de pouso
  double last_z_;                    // Última posição Z recebida (para usar em control_loop)
  rclcpp::Time pouso_start_time_;    // Timestamp quando pouso começou
  bool pouso_start_time_set_;        // Flag para saber se iniciou contagem

  // Rastreamento do último waypoint único recebido (para ignorar duplicatas)
  geometry_msgs::msg::PoseStamped last_waypoint_goal_;
  bool waypoint_goal_received_;      // Recebeu ao menos um waypoint_goal?

  // Setpoints da trajetória (coordenadas relativas ao ponto de hover)
  double trajectory_setpoint_[3];

  // Waypoints da trajetória (5 pontos de descida)
  std::vector<geometry_msgs::msg::Pose> trajectory_waypoints_;
  rclcpp::Time trajectory_start_time_;   // Quando trajetória começou
  bool trajectory_started_;              // Trajetória já iniciou?
  int current_waypoint_idx_;             // Qual waypoint estamos (0-4)
  double waypoint_duration_;             // Tempo em cada waypoint (segundos)

  // Posição Z real do drone (atualizada por odometria)
  double current_z_real_;

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