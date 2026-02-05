#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// 위에서 만든 헤더 포함 (경로 주의)
#include "rhphumanoid_walking_pattern/motion_definitions.hpp"

using namespace std::chrono_literals;

class WalkingController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  WalkingController() : Node("walking_controller")
  {
    // 액션 클라이언트 설정
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/leg_controller/follow_joint_trajectory");

    // 관절 이름 설정
    joint_names_ = {
      "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
      "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll"
    };
  }

  // [핵심] 시퀀스를 통째로 받아서 순차 실행하는 함수
  void execute_sequence(const std::vector<rhp_motions::MotionStep>& sequence)
  {
    for (const auto& step : sequence) {
      if (!rclcpp::ok()) break;

      // hpp에 적힌 시간대로 실행
      send_single_step(step.positions, step.move_time, step.stop_time);
    }
  }

private:
  void send_single_step(const std::vector<double>& positions, double move_t, double stop_t)
  {
    if (!client_ptr_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Server not available");
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(move_t);
    goal_msg.trajectory.points.push_back(point);

    auto future = client_ptr_->async_send_goal(goal_msg);

    // 결과 대기 (Blocking)
    // 실제 이동 시간 + 약간의 여유 시간만큼 대기
    std::this_thread::sleep_for(std::chrono::duration<double>(move_t));

    // 목표 도달 후 정지 시간 대기
    if (stop_t > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(stop_t));
    }
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WalkingController>();

  std::thread logic_thread([node](){
    std::this_thread::sleep_for(2s); // 초기화 대기

    // 1. 초기 자세 잡기
    RCLCPP_INFO(node->get_logger(), ">>> Init Stand");
    node->execute_sequence(rhp_motions::SEQ_INIT_STAND);

    // 2. 보행 루프
    while(rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), ">>> Walking Pattern Start");

        // hpp에 정의된 '걷기 패턴'을 한 번에 실행!
        node->execute_sequence(rhp_motions::SEQ_WALK_IN_PLACE);
    }
  });

  rclcpp::spin(node);
  if(logic_thread.joinable()) logic_thread.join();
  rclcpp::shutdown();
  return 0;
}
