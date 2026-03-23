#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

// ============================================================
// Nó: landing_mode_b_land_and_disarm
//
// Configura landing_mode=1 (Modo B: DISARM ao concluir pouso)
// no /drone_controller_completo e publica um waypoint de pouso
// em /waypoint_goal.
// ============================================================

class LandingModeBLandAndDisarm : public rclcpp::Node
{
public:
  LandingModeBLandAndDisarm() : Node("landing_mode_b_land_and_disarm")
  {
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║  🛬🅱️  MODO B: POUSO + DISARM                      ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝\n");

    // ==========================================
    // PARÂMETROS
    // ==========================================
    this->declare_parameter<std::string>("controller_node", "/drone_controller_completo");
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("z_land", 0.05);
    this->declare_parameter<bool>("use_current_xy", true);
    this->declare_parameter<std::string>("odom_topic", "/uav1/mavros/local_position/odom");

    controller_node_ = this->get_parameter("controller_node").as_string();
    x_ = this->get_parameter("x").as_double();
    y_ = this->get_parameter("y").as_double();
    z_land_ = this->get_parameter("z_land").as_double();
    use_current_xy_ = this->get_parameter("use_current_xy").as_bool();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "📋 Parâmetros:");
    RCLCPP_INFO(this->get_logger(), "   controller_node : %s", controller_node_.c_str());
    RCLCPP_INFO(this->get_logger(), "   use_current_xy=%s  (fallback x=%.2f, y=%.2f, z_land=%.2f)",
      use_current_xy_ ? "true" : "false", x_, y_, z_land_);

    // ==========================================
    // PUBLISHERS
    // ==========================================
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/waypoint_goal", 10);

    RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /waypoint_goal");

    // ==========================================
    // SUBSCRIBER — odometria para posição atual
    // ==========================================
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        odom_received_ = true;
      });

    RCLCPP_INFO(this->get_logger(), "✓ Subscriber criado: %s", odom_topic.c_str());

    // ==========================================
    // SERVICE CLIENT — set_parameters
    // ==========================================
    std::string service_name = controller_node_ + "/set_parameters";
    set_param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(service_name);

    RCLCPP_INFO(this->get_logger(), "✓ Service client criado: %s", service_name.c_str());

    // ==========================================
    // TIMER — execução sequencial via máquina de estados
    // ==========================================
    timer_ = this->create_wall_timer(100ms, std::bind(&LandingModeBLandAndDisarm::timerCallback, this));
  }

private:

  enum class State {
    WAIT_SERVICE,
    SET_PARAMETER,
    PUBLISH_LAND,
    DONE
  };

  void timerCallback()
  {
    switch (state_)
    {
    case State::WAIT_SERVICE:
      if (set_param_client_->service_is_ready()) {
        RCLCPP_INFO(this->get_logger(), "✅ Serviço set_parameters disponível");
        state_ = State::SET_PARAMETER;
      } else {
        if (wait_count_++ % 20 == 0) {
          RCLCPP_INFO(this->get_logger(), "⏳ Aguardando serviço set_parameters...");
        }
      }
      break;

    case State::SET_PARAMETER:
      setLandingMode(1);
      state_ = State::PUBLISH_LAND;
      break;

    case State::PUBLISH_LAND:
    {
      double pub_x, pub_y;
      bool used_odom = false;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        used_odom = use_current_xy_ && odom_received_;
        pub_x = used_odom ? current_x_ : x_;
        pub_y = used_odom ? current_y_ : y_;
      }

      auto msg = geometry_msgs::msg::PoseStamped();
      msg.header.stamp = this->now();
      msg.header.frame_id = "map";
      msg.pose.position.x = pub_x;
      msg.pose.position.y = pub_y;
      msg.pose.position.z = z_land_;
      msg.pose.orientation.w = 1.0;

      waypoint_pub_->publish(msg);

      RCLCPP_INFO(this->get_logger(),
        "📡 Waypoint de POUSO publicado (%s) → X=%.2f, Y=%.2f, Z=%.2f",
        used_odom ? "XY atual" : "XY parametrizado", pub_x, pub_y, z_land_);

      state_ = State::DONE;
      break;
    }

    case State::DONE:
      RCLCPP_INFO(this->get_logger(), "✅ Waypoint de pouso enviado. Encerrando nó.");
      timer_->cancel();
      rclcpp::shutdown();
      break;
    }
  }

  void setLandingMode(int mode)
  {
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter param;
    param.name = "landing_mode";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    param.value.integer_value = mode;
    request->parameters.push_back(param);

    set_param_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
        auto response = future.get();
        if (!response->results.empty() && response->results[0].successful) {
          RCLCPP_INFO(this->get_logger(),
            "✅ Parâmetro landing_mode=%d configurado com sucesso no controlador", mode);
        } else {
          std::string reason = (!response->results.empty()) ? response->results[0].reason : "sem resposta";
          RCLCPP_WARN(this->get_logger(),
            "⚠️ Falha ao configurar landing_mode=%d: %s", mode, reason.c_str());
        }
      });

    RCLCPP_INFO(this->get_logger(),
      "📡 Solicitando landing_mode=%d em %s/set_parameters ...", mode, controller_node_.c_str());
  }

  // ==========================================
  // PUBLISHERS / CLIENTS / TIMERS
  // ==========================================
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ==========================================
  // PARÂMETROS
  // ==========================================
  std::string controller_node_;
  double x_{0.0};
  double y_{0.0};
  double z_land_{0.05};
  bool use_current_xy_{true};

  // ==========================================
  // ESTADO INTERNO
  // ==========================================
  State state_{State::WAIT_SERVICE};
  int wait_count_{0};

  // Posição atual do drone (odometria) — protegida por odom_mutex_
  std::mutex odom_mutex_;
  double current_x_{0.0};
  double current_y_{0.0};
  bool odom_received_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingModeBLandAndDisarm>());
  rclcpp::shutdown();
  return 0;
}
