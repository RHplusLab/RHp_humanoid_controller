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

// [중요] 위에서 만든 헤더 파일을 포함합니다.
#include "rhphumanoid_walking_pattern/motion_definitions.hpp"

using namespace std::chrono_literals;

class WalkingController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  WalkingController() : Node("walking_controller")
  {
    // 1. 액션 클라이언트 생성
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/leg_controller/follow_joint_trajectory");

    // 2. 관절 이름 설정 (12개)
    joint_names_ = {
      "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
      "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll"
    };

    RCLCPP_INFO(this->get_logger(), "Walking Controller Ready.");
  }

  // ========================================================================
  // [High-Level Control API] 사용자가 호출할 함수들
  // ========================================================================

  // 1. 앞으로 걷기 (steps 횟수만큼 반복)
  void go_forward(int steps) {
    RCLCPP_INFO(this->get_logger(), ">>> Command: Go Forward (%d steps)", steps);
    for(int i=0; i<steps; i++) {
        // 헤더에 정의된 SEQ_WALK_FORWARD 실행
        execute_sequence(rhp_motions::SEQ_WALK_FORWARD);
    }
  }

  // 2. 뒤로 걷기
  void go_backward(int steps) {
    RCLCPP_INFO(this->get_logger(), ">>> Command: Go Backward (%d steps)", steps);
    for(int i=0; i<steps; i++) {
        execute_sequence(rhp_motions::SEQ_WALK_BACKWARD);
    }
  }

  // 3. 왼쪽으로 돌기
  void turn_left(int count) {
    RCLCPP_INFO(this->get_logger(), ">>> Command: Turn Left (%d times)", count);
    for(int i=0; i<count; i++) {
        execute_sequence(rhp_motions::SEQ_TURN_LEFT);
    }
  }

  // 4. 정지 및 차렷
  void stop_and_stand() {
    RCLCPP_INFO(this->get_logger(), ">>> Command: Stop & Stand");
    execute_sequence(rhp_motions::SEQ_INIT_STAND);
  }


private:
  // [Low-Level] 시퀀스를 받아서 순차적으로 실행하는 내부 함수
  void execute_sequence(const std::vector<rhp_motions::MotionStep>& sequence)
  {
    for (const auto& step : sequence) {
      if (!rclcpp::ok()) break;
      send_single_step(step.positions, step.move_time, step.stop_time);
    }
  }

  // [Low-Level] 단일 스텝 전송 및 대기 함수
  void send_single_step(const std::vector<double>& positions, double move_t, double stop_t)
  {
    if (!client_ptr_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(move_t);
    goal_msg.trajectory.points.push_back(point);

    // 비동기 전송
    auto future = client_ptr_->async_send_goal(goal_msg);

    // [동기화] 로봇이 움직이는 시간만큼 쓰레드 대기 (Blocking)
    // 실제 이동 시간 + 여유 시간
    std::this_thread::sleep_for(std::chrono::duration<double>(move_t));

    // 도착 후 정지 시간 대기
    if (stop_t > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(stop_t));
    }
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  std::vector<std::string> joint_names_;
};


// ========================================================================
// [Main Loop] 시나리오 작성
// ========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WalkingController>();

  // 로직을 별도 스레드에서 실행 (ROS 통신을 방해하지 않기 위해)
  std::thread logic_thread([node](){

    // 1. 초기 안정화 (2초 대기)
    std::this_thread::sleep_for(2s);
    node->stop_and_stand(); // 차렷 자세

    RCLCPP_INFO(node->get_logger(), "=== SCENARIO START ===");

    if (rclcpp::ok()) {

        // [시나리오 1] 앞으로 2걸음
        node->go_forward(2);
        std::this_thread::sleep_for(1s); // 동작 사이 잠깐 쉬기

        // [시나리오 2] 왼쪽으로 3번 돌기
        node->turn_left(3);
        std::this_thread::sleep_for(1s);

        // [시나리오 3] 뒤로 1걸음
        node->go_backward(1);
        std::this_thread::sleep_for(1s);

        // [종료] 차렷
        node->stop_and_stand();
    }

    RCLCPP_INFO(node->get_logger(), "=== SCENARIO FINISHED ===");

  });

  // 메인 스레드는 ROS 콜백 처리
  rclcpp::spin(node);

  if(logic_thread.joinable()) logic_thread.join();
  rclcpp::shutdown();
  return 0;
}
