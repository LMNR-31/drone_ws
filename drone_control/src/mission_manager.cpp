#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager() : Node("mission_manager")
  {
    // Inicia a sequência após 2 segundos
    timer_ = this->create_wall_timer(
      2s, std::bind(&MissionManager::startMission, this));

    RCLCPP_INFO(this->get_logger(), "🎯 Mission Manager Iniciado");
    RCLCPP_INFO(this->get_logger(), "📡 Aguardando início da sequência em 2s...");
  }

private:

  void startMission()
  {
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🚀 INICIANDO SEQUÊNCIA DE MISSÕES 🚀  ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝\n");

    // ==========================================
    // FASE 1: POUSO COMPLETO (SoftLand)
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 1: POUSO COMPLETO            ║");
    RCLCPP_INFO(this->get_logger(), "╚═══════════════════���════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "🛬 Executando: drone_soft_land");
    RCLCPP_INFO(this->get_logger(), "⬇️ Descendo até o SOLO...\n");
    
    // ✅ Executar drone_soft_land de forma BLOQUEANTE (aguarda conclusão)
    int result = std::system("ros2 run drone_control drone_soft_land");
    
    if (result == 0)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Pouso Concluído - Drone no SOLO com Motores Desligados\n");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "⚠️ drone_soft_land pode não ter executado corretamente\n");
    }

    // ==========================================
    // FASE 2: REPOUSO 10 SEGUNDOS
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  ⏱️ FASE 2: REPOUSO                    ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "😴 Drone descansando no solo...\n");
    
    for (int i = 10; i > 0; i--)
    {
      RCLCPP_INFO(this->get_logger(), "  ⏳ %d segundos restantes...", i);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "✅ Tempo de repouso concluído!\n");

    // ==========================================
    // FASE 3: ATIVAR DRONE
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 3: ATIVAÇÃO DO DRONE         ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "🔋 Executando: drone_activator");
    RCLCPP_INFO(this->get_logger(), "🔓 Solicitando OFFBOARD + ARM...\n");
    
    int result3 = std::system("ros2 run drone_control drone_activator");
    
    if (result3 == 0)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Drone Ativado (OFFBOARD + ARMED)\n");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Erro ao ativar drone!\n");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "⏳ Aguardando 2 segundos antes de levantar...");
    std::this_thread::sleep_for(2s);

    // ==========================================
    // FASE 4: LEVANTAR 2 METROS
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  📍 FASE 4: LEVANTAMENTO              ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "⬆️ Executando: drone_go_forward");
    RCLCPP_INFO(this->get_logger(), "🚁 Levantando até 2 metros...\n");

    int result4 = std::system("ros2 run drone_control drone_go_forward");
    
    if (result4 == 0)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Drone levantou para 2 metros!\n");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "⚠️ Aviso ao levantar\n");
    }

    RCLCPP_INFO(this->get_logger(), "⏳ Monitorando voo...");
    std::this_thread::sleep_for(5s);

    // ==========================================
    // SEQUÊNCIA CONCLUÍDA
    // ==========================================
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║ ✅ SEQUÊNCIA CONCLUÍDA COM SUCESSO ✅  ║");
    RCLCPP_INFO(this->get_logger(), "║                                        ║");
    RCLCPP_INFO(this->get_logger(), "║  1️⃣ ✅ Pouso completo                  ║");
    RCLCPP_INFO(this->get_logger(), "║  2️⃣ ✅ Repouso 10 segundos             ║");
    RCLCPP_INFO(this->get_logger(), "║  3️⃣ ✅ Drone ativado                   ║");
    RCLCPP_INFO(this->get_logger(), "║  4️⃣ ✅ Levantado para 2 metros        ║");
    RCLCPP_INFO(this->get_logger(), "║                                        ║");
    RCLCPP_INFO(this->get_logger(), "║  Drone em voo aguardando comandos...   ║");
    RCLCPP_INFO(this->get_logger(), "║  Pressione Ctrl+C para finalizar       ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝\n");

    // Continua rodando para monitorar (não finaliza)
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