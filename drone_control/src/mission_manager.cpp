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
    RCLCPP_INFO(this->get_logger(), "║   🚀 SEQUÊNCIA DE NÓS: MISSÃO COMPLETA            ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ==========================================
    // FASE 1: PUBLICAR WAYPOINTS DE POUSO
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 1: PUBLICAR WAYPOINTS DE POUSO           ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    RCLCPP_INFO(this->get_logger(), "🔄 Executando: ros2 run drone_control drone_publish_landing_waypoints\n");

    int result1 = std::system("ros2 run drone_control drone_publish_landing_waypoints");

    if (result1 == 0) {
      RCLCPP_INFO(this->get_logger(), "✅ Waypoints de pouso publicados!\n");
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️ Erro ao publicar waypoints (código: %d)\n", result1);
    }

    // ==========================================
    // REPOUSO: 5 SEGUNDOS
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  ⏳ FASE 2: REPOUSO 10 SEGUNDOS                     ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    for (int i = 10; i > 0; i--) {
      RCLCPP_INFO(this->get_logger(), "  ⏳ %d segundo(s) restante(s)...", i);
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO(this->get_logger(), "✅ Repouso concluído!\n");

    // ==========================================
    // FASE 3: ATIVAR DRONE E EXECUTAR
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🚁 FASE 3: ATIVAR DRONE E EXECUTAR                ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    RCLCPP_INFO(this->get_logger(), "🔄 Executando: ros2 run drone_control drone_activate_and_go_forward\n");

    int result2 = std::system("ros2 run drone_control drone_activate_and_go_forward");

    if (result2 == 0) {
      RCLCPP_INFO(this->get_logger(), "✅ Drone ativado e missão concluída!\n");
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️ Erro ao executar drone (código: %d)\n", result2);
    }

    // ==========================================
    // MISSÃO FINALIZADA
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║ ✅ MISSÃO COMPLETA COM SUCESSO ✅                  ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    RCLCPP_INFO(this->get_logger(), "📋 Sequência executada:");
    RCLCPP_INFO(this->get_logger(), "   1️⃣ %s drone_publish_landing_waypoints", result1 == 0 ? "✅" : "⚠️");
    RCLCPP_INFO(this->get_logger(), "   2️⃣ ✅ Repouso 5 segundos");
    RCLCPP_INFO(this->get_logger(), "   3️⃣ %s drone_activate_and_go_forward\n", result2 == 0 ? "✅" : "⚠️");

    RCLCPP_INFO(this->get_logger(), "🛑 Encerrando Mission Manager...");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ✅ ENCERRA O NÓ
    rclcpp::shutdown();
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