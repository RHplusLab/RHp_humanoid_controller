#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

class HumanoidController : public rclcpp::Node {
public:
  HumanoidController() : Node("humanoid_manual_node") {
    // ros2_control이 명령을 받을 수 있도록 토픽 발행
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);
    // 1초마다 명령 실행 (임의의 시간/각도)
    timer_ = this->create_wall_timer(1s, std::bind(&HumanoidController::send_command, this));
  }

private:
  void send_command() {
    auto message = trajectory_msgs::msg::JointTrajectory();
    message.joint_names = {"joint_1", "joint_2"}; // 모터 2개 이름

    trajectory_msgs::msg::JointTrajectoryPoint point;
    // 임의의 각도: joint1은 0.5rad, joint2는 -0.3rad
    point.positions = {0.5, -0.3};
    // 0.5초 동안 해당 위치로 이동
    point.time_from_start = rclcpp::Duration(500ms);

    message.points.push_back(point);
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "모터 2개에 각도 지령 전송 완료");
  }
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanoidController>());
  rclcpp::shutdown();
  return 0;
}
