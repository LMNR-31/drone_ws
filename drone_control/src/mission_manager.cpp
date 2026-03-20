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
  }

private:

  void startMission()
  {
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(),"🚀 Starting SoftLand node");

    std::system("ros2 run drone_control drone_soft_land &");

    std::this_thread::sleep_for(std::chrono::seconds(20));

    RCLCPP_INFO(this->get_logger(),"⏱ Waiting extra 10 seconds");
    std::this_thread::sleep_for(std::chrono::seconds(10));

    RCLCPP_INFO(this->get_logger(),"🚀 Starting Activator + Forward");

    std::system("ros2 run drone_control drone_activator &");
    std::system("ros2 run drone_control drone_go_forward &");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}