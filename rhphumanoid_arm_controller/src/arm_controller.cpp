#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// 방금 만든 헤더 파일 포함
#include "rhphumanoid_arm_controller/arm_definitions.hpp"

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  ArmController() : Node("arm_controller")
  {
    // 액션 클라이언트 (arm_controller)
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/arm_controller/follow_joint_trajectory");

    // 관절 이름 (10개)
    joint_names_ = {
      "l_sho_pitch", "l_sho_roll", "l_el", "l_wst", "l_grp",
      "r_sho_pitch", "r_sho_roll", "r_el", "r_wst", "r_grp"
    };
  }

  // --- API Functions ---
  
  void raise_arm() {
    RCLCPP_INFO(this->get_logger(), "Command: Raise Arm");
    execute_sequence(rhp_arm_motions::SEQ_RAISE_ARM);
  }

  void wave_arm(int times) {
    RCLCPP_INFO(this->get_logger(), "Command: Wave Arm %d times", times);
    for(int i=0; i<times; i++) execute_sequence(rhp_arm_motions::SEQ_WAVE_ARM);
  }

  void lower_arm() {
    RCLCPP_INFO(this->get_logger(), "Command: Lower Arm");
    execute_sequence(rhp_arm_motions::SEQ_LOWER_ARM);
  }

private:
  void execute_sequence(const std::vector<rhp_arm_motions::MotionStep>& sequence) {
    for (const auto& step : sequence) {
      if(!rclcpp::ok()) break;
      send_goal(step.positions, step.move_time, step.stop_time);
    }
  }

  void send_goal(const std::vector<double>& pos, double move_t, double stop_t) {
    if (!client_ptr_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Server not available");
      return;
    }
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = pos;
    p.time_from_start = rclcpp::Duration::from_seconds(move_t);
    goal.trajectory.points.push_back(p);

    client_ptr_->async_send_goal(goal);
    
    // Blocking Wait
    std::this_thread::sleep_for(std::chrono::duration<double>(move_t));
    if(stop_t > 0) std::this_thread::sleep_for(std::chrono::duration<double>(stop_t));
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmController>();

  std::thread logic([node](){
    std::this_thread::sleep_for(2s);

    // [시나리오 실행]
    node->raise_arm();  // 오른팔 들기
    node->wave_arm(2);  // 오른팔 흔들기
    node->lower_arm();  // 오른팔 내리기

  });

  rclcpp::spin(node);
  if(logic.joinable()) logic.join();
  rclcpp::shutdown();
  return 0;
}