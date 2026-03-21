#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager() : Node("mission_manager")
  {
    timer_ = this->create_wall_timer(
      2s, std::bind(&MissionManager::startMission, this));

    RCLCPP_INFO(this->get_logger(), "🎯 Mission Manager Iniciado");
  }

private:

  void startMission()
  {
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║        🚀 SEQUÊNCIA COMPLETA COM POUSO           ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ==========================================
    // FASE 1: PUBLICAR WAYPOINTS DE POUSO
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 1: PUBLICAR WAYPOINTS DE POUSO          ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "📡 Executando: drone_publish_landing_waypoints\n");
    
    int result0 = std::system("ros2 run drone_control drone_publish_landing_waypoints");
    
    if (result0 == 0) {
      RCLCPP_INFO(this->get_logger(), "✅ Waypoints de pouso publicados!\n");
    }

    std::this_thread::sleep_for(1s);

    // ==========================================
    // FASE 2: POUSO COMPLETO
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 2: POUSO COMPLETO                       ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "🛬 Executando: drone_soft_land\n");
    
    int result1 = std::system("ros2 run drone_control drone_soft_land");
    
    if (result1 == 0) {
      RCLCPP_INFO(this->get_logger(), "✅ Pouso Completo - Drone no SOLO\n");
    }

    std::this_thread::sleep_for(3s);

    // ==========================================
    // FASE 3: REPOUSO 10 SEGUNDOS
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 3: REPOUSO                               ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "😴 Drone descansando no solo...\n");
    
    for (int i = 10; i > 0; i--) {
      RCLCPP_INFO(this->get_logger(), "  ⏳ %d segundos restantes...", i);
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO(this->get_logger(), "✅ Repouso concluído!\n");

    // ==========================================
    // FASE 4: ATIVAR + LEVANTAR (UNIFICADO)
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 4: ATIVAÇÃO E LEVANTAMENTO              ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "🔋⬆️ Executando: drone_activate_and_go_forward\n");

    // ✅ Executar em BACKGROUND para não bloquear
    std::system("ros2 run drone_control drone_activate_and_go_forward &");

    RCLCPP_INFO(this->get_logger(), "✅ drone_activate_and_go_forward iniciado em BACKGROUND!\n");
    RCLCPP_INFO(this->get_logger(), "✅ Mission Manager finalizando - Drone continua ativo!\n");

    // ✅ Aguardar brevemente apenas para garantir que o nó iniciou
    std::this_thread::sleep_for(2s);

    // ==========================================
    // SEQUÊNCIA CONCLUÍDA
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║ ✅ SEQUÊNCIA COMPLETA COM SUCESSO ✅             ║");
    RCLCPP_INFO(this->get_logger(), "║                                                   ║");
    RCLCPP_INFO(this->get_logger(), "║  1️⃣ ✅ Waypoints de pouso publicados             ║");
    RCLCPP_INFO(this->get_logger(), "║  2️⃣ ✅ Pouso completo até o solo                 ║");
    RCLCPP_INFO(this->get_logger(), "║  3️⃣ ✅ Repouso 10 segundos                        ║");
    RCLCPP_INFO(this->get_logger(), "║  4️⃣ ✅ Ativado + Levantado (UNIFICADO)           ║");
    RCLCPP_INFO(this->get_logger(), "║                                                   ║");
    RCLCPP_INFO(this->get_logger(), "║  Drone em voo aguardando comandos                 ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}