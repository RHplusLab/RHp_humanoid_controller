#ifndef RHPHUMANOID_WALKING_MODULE_H_
#define RHPHUMANOID_WALKING_MODULE_H_

#include <cmath>
#include <map>
#include <string>

#include <eigen3/Eigen/Eigen>

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "rhphumanoid_walking_parameter.h"
#include "rhphumanoid_kinematics_dynamics.h"

namespace rhp
{

typedef struct { double x, y, z; } Position3D;
typedef struct { double x, y, z, roll, pitch, yaw; } Pose3D;

class WalkingModule : public rclcpp::Node
{
public:
  enum { PHASE0 = 0, PHASE1 = 1, PHASE2 = 2, PHASE3 = 3 };

  explicit WalkingModule(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~WalkingModule();

  int    getCurrentPhase() { return phase_; }
  double getBodySwingY()   { return body_swing_y_; }
  double getBodySwingZ()   { return body_swing_z_; }

private:
  enum WalkingState { WalkingDisable = 0, WalkingEnable = 1, WalkingReady = 2 };

  // ROS2 callbacks
  void walkingCommandCallback(const std_msgs::msg::String::SharedPtr msg);

  // Walking logic
  void   processPhase(double time_unit);
  bool   computeLegAngle(double *leg_angle);
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  void   updateTimeParam();
  void   updateMovementParam();
  void   updatePoseParam();
  void   startWalking();
  void   stop();
  void   loadWalkingParam(const std::string &path);
  void   startGaitCycle();

  // Kinematics
  RHpKinematicsDynamics *kd_;

  // ROS2
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr start_timer_;

  void sendTrajectory(const std::vector<double> &positions, double move_time);

  // joint name -> index in angle array
  // index 0~5: R leg, 6~11: L leg
  std::map<std::string, int> joint_table_;
  std::vector<std::string>   joint_names_;   // ordered by index

  // State
  WalkingState walking_state_;
  bool ctrl_running_;
  bool real_running_;
  bool is_starting_;
  double time_;
  int phase_;

  // Walking parameters
  WalkingParam walking_param_;
  double previous_x_move_amplitude_;

  // Body swing
  double body_swing_y_;
  double body_swing_z_;

  // Time params
  double period_time_;
  double dsp_ratio_;
  double ssp_ratio_;
  double x_swap_period_time_, x_move_period_time_;
  double y_swap_period_time_, y_move_period_time_;
  double z_swap_period_time_, z_move_period_time_;
  double a_move_period_time_;
  double ssp_time_;
  double l_ssp_start_time_, l_ssp_end_time_;
  double r_ssp_start_time_, r_ssp_end_time_;
  double phase1_time_, phase2_time_, phase3_time_;

  // Pose offset params
  double x_offset_, y_offset_, z_offset_;
  double r_offset_, p_offset_, a_offset_;
  double hit_pitch_offset_;

  // Swing amplitude params
  double x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_;
  double x_move_phase_shift_, x_move_amplitude_, x_move_amplitude_shift_;
  double y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_;
  double y_move_phase_shift_, y_move_amplitude_, y_move_amplitude_shift_;
  double z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_;
  double z_move_phase_shift_, z_move_amplitude_, z_move_amplitude_shift_;
  double a_move_phase_shift_, a_move_amplitude_, a_move_amplitude_shift_;

  double pelvis_offset_;
  double pelvis_swing_;

  // Control cycle
  int control_cycle_ms_;

  static constexpr int JOINT_NUM = 12;  // 6 R leg + 6 L leg
};

} // namespace rhp

#endif /* RHPHUMANOID_WALKING_MODULE_H_ */
