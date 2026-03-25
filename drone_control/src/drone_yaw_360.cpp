#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include "drone_control/msg/yaw_override.hpp"

#include <chrono>
#include <cmath>
#include <future>

using namespace std::chrono_literals;
using std::placeholders::_1;

class DroneYaw360 : public rclcpp::Node
{
public:
  DroneYaw360() : Node("drone_yaw_360")
  {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🔄  DRONE YAW 360 - GIRO EM TORNO DO EIXO Z      ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ==========================================
    // PARÂMETROS
    // ==========================================
    this->declare_parameter<std::string>("uav_ns", "/uav1");
    // z_hold: altitude de hold durante o giro. -1.0 (default) = usar altitude atual do odometry.
    this->declare_parameter<double>("z_hold", -1.0);
    this->declare_parameter<double>("yaw_rate", 0.8);
    this->declare_parameter<double>("angle", 2.0 * M_PI);
    this->declare_parameter<double>("hz", 20.0);
    // ccw: true = anti-horário (counter-clockwise), false = horário (clockwise)
    this->declare_parameter<bool>("ccw", true);
    // controller_node: nome do nó controlador (my_drone_controller / drone_controller_completo)
    // a pausar/retomar durante o giro
    this->declare_parameter<std::string>("controller_node", "drone_controller_completo");
    // auto_disable_controller: se true, seta override_active=true no my_drone_controller antes
    // de girar e override_active=false ao terminar, congelando a FSM durante o giro.
    this->declare_parameter<bool>("auto_disable_controller", true);
    // set_enabled_param: se true (padrão), também seta enabled=false no drone_controller_completo
    // durante o giro (impede publicação de hold setpoints) e restaura enabled=true ao final.
    // Requer auto_disable_controller=true para ter efeito.
    this->declare_parameter<bool>("set_enabled_param", true);
    // exit_on_finish: se true (padrão), chama rclcpp::shutdown() após o giro.
    // Se false, retorna a WAIT_OFFBOARD_AND_ARMED para permitir novos giros.
    this->declare_parameter<bool>("exit_on_finish", true);

    uav_ns_                  = this->get_parameter("uav_ns").as_string();
    z_hold_                  = this->get_parameter("z_hold").as_double();
    yaw_rate_                = this->get_parameter("yaw_rate").as_double();
    angle_                   = this->get_parameter("angle").as_double();
    hz_                      = this->get_parameter("hz").as_double();
    ccw_                     = this->get_parameter("ccw").as_bool();
    controller_node_         = this->get_parameter("controller_node").as_string();
    auto_disable_controller_ = this->get_parameter("auto_disable_controller").as_bool();
    set_enabled_param_       = this->get_parameter("set_enabled_param").as_bool();
    exit_on_finish_          = this->get_parameter("exit_on_finish").as_bool();

    if (hz_ <= 0.0) {
      throw std::runtime_error("Parâmetro 'hz' precisa ser > 0");
    }
    if (yaw_rate_ == 0.0) {
      throw std::runtime_error("Parâmetro 'yaw_rate' não pode ser 0");
    }

    // Normaliza: angle é tratado como magnitude; a direção vem de ccw_
    if (angle_ < 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "Parâmetro 'angle' negativo (%.4f rad) — usando valor absoluto. Direção controlada por 'ccw'.", angle_);
      angle_ = std::abs(angle_);
    }

    // yaw_rate com sinal: positivo = CCW, negativo = CW
    yaw_rate_signed_ = ccw_ ? std::abs(yaw_rate_) : -std::abs(yaw_rate_);

    // Duração do giro baseada em tempo: t = angle / |yaw_rate|
    duration_ = angle_ / std::abs(yaw_rate_);

    const char * dir_str = ccw_ ? "anti-horário (CCW)" : "horário (CW)";
    if (z_hold_ < 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "Parâmetros: uav_ns=%s z_hold=<altitude atual> yaw_rate=%.2f rad/s angle=%.2f rad hz=%.1f direção=%s duração=%.1fs",
        uav_ns_.c_str(), yaw_rate_, angle_, hz_, dir_str, duration_);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Parâmetros: uav_ns=%s z_hold=%.2f yaw_rate=%.2f rad/s angle=%.2f rad hz=%.1f direção=%s duração=%.1fs",
        uav_ns_.c_str(), z_hold_, yaw_rate_, angle_, hz_, dir_str, duration_);
    }
    RCLCPP_INFO(this->get_logger(),
      "Parâmetros: controller_node=%s auto_disable_controller=%s (override_active) set_enabled_param=%s (enabled) exit_on_finish=%s",
      controller_node_.c_str(), auto_disable_controller_ ? "true" : "false",
      set_enabled_param_ ? "true" : "false",
      exit_on_finish_ ? "true" : "false");

    // ==========================================
    // PUBLISHERS
    // ==========================================
    yaw_override_pub_ = this->create_publisher<drone_control::msg::YawOverride>(
      uav_ns_ + "/yaw_override/cmd", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: %s",
      (uav_ns_ + "/yaw_override/cmd").c_str());

    // ==========================================
    // SUBSCRIBERS
    // ==========================================
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      uav_ns_ + "/mavros/state", 10,
      std::bind(&DroneYaw360::stateCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      uav_ns_ + "/mavros/local_position/odom", 10,
      std::bind(&DroneYaw360::odomCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "✓ Subscribers criados: %s/mavros/state e %s/mavros/local_position/odom",
      uav_ns_.c_str(), uav_ns_.c_str());

    // ==========================================
    // CLIENTE DE PARÂMETROS (para ativar/desativar override e enabled no my_drone_controller)
    //
    // Integração com my_drone_controller (drone_controller_completo):
    //   - override_active=true  → congela FSM (continua publicando hold setpoints)
    //   - enabled=false         → para completamente a publicação de setpoints do controller,
    //                             dando controle exclusivo ao yaw_override durante o giro.
    // ==========================================
    if (auto_disable_controller_) {
      param_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, controller_node_, rmw_qos_profile_parameters, param_cb_group_);
      RCLCPP_INFO(this->get_logger(),
        "✓ AsyncParametersClient criado para nó: %s", controller_node_.c_str());
    }

    // ==========================================
    // TIMER
    // ==========================================
    state_ = StateMachine::WAIT_FCU;
    state_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz_),
      std::bind(&DroneYaw360::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "✓ Timer criado: %.1f Hz", hz_);
    RCLCPP_INFO(this->get_logger(), "\n🔄 Aguardando conexão FCU...");
  }

  ~DroneYaw360()
  {
    // Tenta desativar o override no controller mesmo em caso de shutdown inesperado (best-effort).
    // Restaura override_active=false e enabled=true no my_drone_controller (drone_controller_completo).
    if (controller_disabled_) {
      // Publica mensagem de disable do yaw override
      if (yaw_override_pub_) {
        drone_control::msg::YawOverride msg;
        msg.enable = false;
        msg.yaw_rate = 0.0f;
        msg.timeout = 0.0f;
        yaw_override_pub_->publish(msg);
      }
    }
    if (auto_disable_controller_ && controller_disabled_) {
      if (param_client_ && param_client_->service_is_ready()) {
        RCLCPP_INFO(this->get_logger(),
          "🔓 Destrutor: restaurando override_active=false e enabled=true em '%s' (best-effort)...",
          controller_node_.c_str());
        // Restaura override_active=false para retomar FSM do my_drone_controller
        param_client_->set_parameters({rclcpp::Parameter("override_active", false)});
        // Restaura enabled=true para retomar publicação de setpoints do my_drone_controller
        if (set_enabled_param_) {
          param_client_->set_parameters({rclcpp::Parameter("enabled", true)});
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
          "⚠️  Destrutor: serviço de parâmetros de '%s' indisponível — override_active/enabled não restaurados.",
          controller_node_.c_str());
      }
    }
  }

private:

  enum class StateMachine {
    WAIT_FCU,
    WAIT_ODOM,
    WAIT_OFFBOARD_AND_ARMED,
    ROTATING,
    FINISH
  };

  /**
   * @brief Publish a YawOverride command to the controller.
   *
   * When enable=true, the controller will hold current position and apply yaw_rate.
   * When enable=false, the controller resumes normal FSM operation.
   */
  void publishYawOverride(bool enable, double yaw_rate, float timeout = 0.3f)
  {
    drone_control::msg::YawOverride msg;
    msg.enable = enable;
    msg.yaw_rate = static_cast<float>(yaw_rate);
    msg.timeout = timeout;
    yaw_override_pub_->publish(msg);
  }

  /**
   * @brief Set the `override_active` parameter on the remote controller node (non-blocking).
   *
   * Integração com my_drone_controller: override_active=true congela a FSM do
   * drone_controller_completo mas mantém a publicação de hold setpoints para
   * preservar o modo OFFBOARD. override_active=false retoma a FSM normal.
   *
   * Fire-and-forget via AsyncParametersClient — não bloqueia o timer callback.
   * Best-effort: loga warning se o serviço estiver indisponível.
   *
   * @param value  New value for `override_active` (true = freeze FSM, false = resume).
   */
  void setControllerOverride(bool value)
  {
    if (!param_client_) {
      return;
    }
    if (!param_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  Serviço de parâmetros de '%s' não disponível — continuando sem %s o override_active.",
        controller_node_.c_str(), value ? "ativar" : "desativar");
      return;
    }

    // Fire-and-forget: não bloqueamos o timer callback aguardando a resposta.
    param_client_->set_parameters({rclcpp::Parameter("override_active", value)});
    RCLCPP_INFO(this->get_logger(),
      "%s override_active=%s em '%s' (my_drone_controller, best-effort).",
      value ? "🔒 Ativando" : "🔓 Desativando",
      value ? "true" : "false",
      controller_node_.c_str());
  }

  /**
   * @brief Set the `enabled` parameter on the remote controller node (non-blocking).
   *
   * Integração com my_drone_controller: enabled=false para completamente a publicação
   * de setpoints do drone_controller_completo, dando controle exclusivo ao yaw_override
   * durante o giro. enabled=true retoma a publicação normal de setpoints.
   *
   * Usado somente quando set_enabled_param_=true (parâmetro configurável).
   * Fire-and-forget via AsyncParametersClient — não bloqueia o timer callback.
   * Best-effort: loga warning se o serviço estiver indisponível.
   *
   * @param value  New value for `enabled` (false = stop setpoints, true = resume).
   */
  void setControllerEnabled(bool value)
  {
    if (!set_enabled_param_) {
      return;
    }
    if (!param_client_) {
      return;
    }
    if (!param_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  Serviço de parâmetros de '%s' não disponível — continuando sem %s o enabled.",
        controller_node_.c_str(), value ? "restaurar" : "desativar");
      return;
    }

    // Fire-and-forget: não bloqueamos o timer callback aguardando a resposta.
    param_client_->set_parameters({rclcpp::Parameter("enabled", value)});
    RCLCPP_INFO(this->get_logger(),
      "%s enabled=%s em '%s' (my_drone_controller, best-effort).",
      value ? "🔓 Restaurando" : "🔒 Desativando",
      value ? "true" : "false",
      controller_node_.c_str());
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void timerCallback()
  {
    switch (state_)
    {
      case StateMachine::WAIT_FCU:
      {
        if (current_state_.connected) {
          RCLCPP_INFO(this->get_logger(), "✓ FCU conectada!");
          state_ = StateMachine::WAIT_ODOM;
          state_time_ = this->now();
        } else {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "⏳ Aguardando conexão FCU... (connected=%d)", (int)current_state_.connected);
        }
        break;
      }

      case StateMachine::WAIT_ODOM:
      {
        if (odom_received_) {
          RCLCPP_INFO(this->get_logger(), "✓ Odom recebida: X=%.2f Y=%.2f Z=%.2f",
            current_x_, current_y_, current_z_);
          state_ = StateMachine::WAIT_OFFBOARD_AND_ARMED;
          state_time_ = this->now();
        } else {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "⏳ Aguardando odom... (odom_received=%d)", (int)odom_received_);
        }
        break;
      }

      case StateMachine::WAIT_OFFBOARD_AND_ARMED:
      {
        if (!(current_state_.armed && current_state_.mode == "OFFBOARD")) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "⚠️ Não está em OFFBOARD+ARMED (armed=%d mode=%s) — iniciando rotação mesmo assim (best-effort).",
            (int)current_state_.armed, current_state_.mode.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "🔄 Iniciando giro %.1f° %s (best-effort).",
          angle_ * 180.0 / M_PI, ccw_ ? "anti-horário (CCW)" : "horário (CW)");
        RCLCPP_INFO(this->get_logger(), "   yaw_rate=%.3f rad/s  duração=%.1fs",
          yaw_rate_signed_, duration_);

        // Sinaliza ao my_drone_controller (drone_controller_completo) para congelar FSM
        // (override_active=true) e parar publicação de setpoints (enabled=false),
        // dando controle exclusivo ao yaw_override durante o giro.
        if (auto_disable_controller_) {
          RCLCPP_INFO(this->get_logger(),
            "🔒 Integrando com my_drone_controller '%s': ativando override_active=true e %s...",
            controller_node_.c_str(),
            set_enabled_param_ ? "enabled=false" : "(set_enabled_param=false, enabled não alterado)");
          setControllerOverride(true);
          setControllerEnabled(false);
          controller_disabled_ = true;
        }

        start_time_ = this->now();
        state_ = StateMachine::ROTATING;
        break;
      }

      case StateMachine::ROTATING:
      {
        // Publica override de yaw para o controller (ele mantém a posição + aplica yaw_rate)
        publishYawOverride(true, yaw_rate_signed_, 0.3f);

        const double elapsed = (this->now() - start_time_).seconds();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "🔄 Girando... %.1fs / %.1fs (%.0f%%) armed=%d mode=%s",
          elapsed, duration_, std::min(elapsed / duration_ * 100.0, 100.0),
          (int)current_state_.armed, current_state_.mode.c_str());
        if (elapsed >= duration_) {
          RCLCPP_INFO(this->get_logger(), "✅ Giro concluído! (%.1f° %s em %.2fs)",
            angle_ * 180.0 / M_PI, ccw_ ? "CCW" : "CW", elapsed);
          finish_time_ = this->now();
          state_ = StateMachine::FINISH;
        }
        break;
      }

      case StateMachine::FINISH:
      {
        // Desativa override de yaw: controller retoma FSM normal
        publishYawOverride(false, 0.0, 0.0f);
        if ((this->now() - finish_time_).seconds() >= 1.0) {
          // Restaura parâmetros do my_drone_controller (drone_controller_completo):
          //   enabled=true          → retoma publicação de setpoints
          //   override_active=false → retoma FSM normal
          if (auto_disable_controller_ && controller_disabled_) {
            RCLCPP_INFO(this->get_logger(),
              "🔓 Restaurando my_drone_controller '%s': override_active=false e %s...",
              controller_node_.c_str(),
              set_enabled_param_ ? "enabled=true" : "(set_enabled_param=false, enabled não alterado)");
            setControllerEnabled(true);
            setControllerOverride(false);
            controller_disabled_ = false;
          }
          if (exit_on_finish_) {
            RCLCPP_INFO(this->get_logger(), "Encerrando nó.");
            rclcpp::shutdown();
          } else {
            RCLCPP_INFO(this->get_logger(),
              "Giro concluído. Retornando a WAIT_OFFBOARD_AND_ARMED (exit_on_finish=false).");
            state_ = StateMachine::WAIT_OFFBOARD_AND_ARMED;
            state_time_ = this->now();
          }
        }
        break;
      }
    }
  }

  // ==========================================
  // PUBLISHERS / SUBSCRIBERS / TIMER
  // ==========================================
  rclcpp::Publisher<drone_control::msg::YawOverride>::SharedPtr yaw_override_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback group dedicado para o param_client_ (evita deadlock com MultiThreadedExecutor)
  rclcpp::CallbackGroup::SharedPtr param_cb_group_;
  // Cliente assíncrono de parâmetros para pausar/retomar o drone_controller_completo
  rclcpp::AsyncParametersClient::SharedPtr param_client_;

  // ==========================================
  // ESTADO
  // ==========================================
  StateMachine state_{StateMachine::WAIT_FCU};
  rclcpp::Time state_time_;
  rclcpp::Time start_time_;

  mavros_msgs::msg::State current_state_;
  bool odom_received_{false};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};

  // ==========================================
  // PARÂMETROS
  // ==========================================
  std::string uav_ns_;
  double z_hold_{-1.0};
  double yaw_rate_{0.8};
  double angle_{2.0 * M_PI};
  double hz_{20.0};
  bool   ccw_{true};

  // Nome do nó controlador (my_drone_controller / drone_controller_completo)
  // a ter override_active e enabled alterados durante o giro
  std::string controller_node_;
  // Se true, seta override_active=true/false no drone_controller_completo durante/após o giro
  bool auto_disable_controller_{true};
  // Se true, também seta enabled=false/true no drone_controller_completo durante/após o giro
  // (evita competição de setpoints; requer auto_disable_controller=true para ter efeito)
  bool set_enabled_param_{true};
  // Flag para rastrear se o override foi ativado por este nó (para desativar no destrutor)
  bool controller_disabled_{false};
  // Se true, chama rclcpp::shutdown() após o giro; se false, retorna a WAIT_OFFBOARD_AND_ARMED
  bool exit_on_finish_{true};

  // yaw_rate com sinal: positivo = CCW, negativo = CW
  double yaw_rate_signed_{0.8};

  // Duração do giro em segundos
  double duration_{0.0};

  // ==========================================
  // ROTAÇÃO
  // ==========================================
  rclcpp::Time finish_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor: permite que a resposta do serviço de parâmetros
  // seja processada num thread separado (embora as chamadas sejam fire-and-forget).
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(std::make_shared<DroneYaw360>());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
