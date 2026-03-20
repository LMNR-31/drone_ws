#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>

using namespace std::chrono_literals;

class DroneUpAndForward : public rclcpp::Node
{
public:
    DroneUpAndForward() : Node("drone_up_and_forward"), ready_(false)
    {
        // Publisher
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/uav1/mavros/setpoint_raw/local", 10);

        // Subscriber de estado
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/uav1/mavros/state", 10,
            std::bind(&DroneUpAndForward::stateCallback, this, std::placeholders::_1));

        // Timer ~20 Hz
        timer_ = this->create_wall_timer(
            50ms, std::bind(&DroneUpAndForward::timerCallback, this));

        // Configuração do setpoint (quase tudo ignorado exceto posição z e talvez x)
        target_.header.frame_id = "local_origin";
        target_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // Máscara: ignora px, py, afx, afy, afz, yaw
        // Mantém apenas position.z + possivelmente velocity.x se quiser velocidade
        target_.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_PX |
            mavros_msgs::msg::PositionTarget::IGNORE_PY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;

        // Valores desejados
        target_.position.z = 2.0;     // altura desejada (m)
        target_.position.x = 2.0;     // posição x desejada (m)  ← se quiser usar posição
        // target_.velocity.x = 0.5;  // ← descomente se preferir controlar por VELOCIDADE

        target_.velocity.y = 0.0;
        target_.velocity.z = 0.0;

        target_.yaw = 0.0;
        target_.yaw_rate = 0.0;

        RCLCPP_INFO(this->get_logger(), "Nó iniciado. Aguardando drone em OFFBOARD + armed + connected...");
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;

        bool was_not_ready = !ready_;
        ready_ = (msg->connected &&
                  msg->armed &&
                  msg->mode == "OFFBOARD");

        if (ready_ && was_not_ready)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Drone PRONTO! (connected + armed + OFFBOARD) → enviando setpoints");
        }
    }

    void timerCallback()
    {
        if (!ready_)
        {
            return;
        }

        target_.header.stamp = this->now();
        setpoint_pub_->publish(target_);

        // Descomente para debug mais intenso
        // RCLCPP_DEBUG(this->get_logger(), "Setpoint publicado: z = %.2f", target_.position.z);
    }

    // Membros
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::PositionTarget target_;
    mavros_msgs::msg::State current_state_;
    bool ready_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneUpAndForward>();

    RCLCPP_INFO(node->get_logger(), "Iniciando drone_up_and_forward (C++)");

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exceção: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}