#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "rhphumanoid_head_controller/head_definitions.hpp"

using namespace std::chrono_literals;

class HeadController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  HeadController() : Node("head_controller_node")
  {
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/head_controller/follow_joint_trajectory");

    // Joint order must match URDF ros2_control declaration
    joint_names_ = {"head_pan", "head_tilt"};
  }

  // --- API Functions ---

  void look_at(double pan_rad, double tilt_rad)
  {
    RCLCPP_INFO(this->get_logger(), "Command: look_at(pan=%.3f, tilt=%.3f)", pan_rad, tilt_rad);
    send_goal({pan_rad, tilt_rad}, 0.8, 0.2);
  }

  void center()
  {
    RCLCPP_INFO(this->get_logger(), "Command: center");
    execute_sequence(rhp_head_motions::SEQ_CENTER);
  }

  void scan_horizontal()
  {
    RCLCPP_INFO(this->get_logger(), "Command: scan_horizontal");
    execute_sequence(rhp_head_motions::SEQ_SCAN_HORIZONTAL);
  }

  void scan_vertical()
  {
    RCLCPP_INFO(this->get_logger(), "Command: scan_vertical");
    execute_sequence(rhp_head_motions::SEQ_SCAN_VERTICAL);
  }

private:
  void execute_sequence(const std::vector<rhp_head_motions::MotionStep>& sequence)
  {
    for (const auto& step : sequence) {
      if (!rclcpp::ok()) break;
      send_goal(step.positions, step.move_time, step.stop_time);
    }
  }

  void send_goal(const std::vector<double>& pos, double move_t, double stop_t)
  {
    if (!client_ptr_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "head_controller action server not available");
      return;
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = pos;
    p.time_from_start = rclcpp::Duration::from_seconds(move_t);
    goal.trajectory.points.push_back(p);

    client_ptr_->async_send_goal(goal);

    // Blocking wait matching the arm_controller pattern
    std::this_thread::sleep_for(std::chrono::duration<double>(move_t));
    if (stop_t > 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(stop_t));
    }
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadController>();

  std::thread logic([node]() {
    std::this_thread::sleep_for(2s);

    // [Default scenario]
    node->center();
    node->scan_horizontal();
    node->center();
  });

  rclcpp::spin(node);
  if (logic.joinable()) logic.join();
  rclcpp::shutdown();
  return 0;
}
