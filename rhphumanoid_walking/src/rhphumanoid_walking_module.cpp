#include <fstream>
#include <yaml-cpp/yaml.h>

#include "rhphumanoid_walking/rhphumanoid_walking_module.h"

namespace rhp
{

static constexpr double DEG2RAD = M_PI / 180.0;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
WalkingModule::WalkingModule(const rclcpp::NodeOptions &options)
: Node("rhphumanoid_walking", options),
  walking_state_(WalkingReady),
  ctrl_running_(false),
  real_running_(false),
  is_starting_(false),
  time_(0.0),
  phase_(PHASE0),
  previous_x_move_amplitude_(0.0),
  body_swing_y_(0.0),
  body_swing_z_(0.0),
  control_cycle_ms_(20)   // 50 Hz
{
  kd_ = new RHpKinematicsDynamics();

  // joint index table: 0~5 = R leg, 6~11 = L leg
  joint_table_["r_hip_yaw"]   = 0;
  joint_table_["r_hip_roll"]  = 1;
  joint_table_["r_hip_pitch"] = 2;
  joint_table_["r_knee"]      = 3;
  joint_table_["r_ank_pitch"] = 4;
  joint_table_["r_ank_roll"]  = 5;
  joint_table_["l_hip_yaw"]   = 6;
  joint_table_["l_hip_roll"]  = 7;
  joint_table_["l_hip_pitch"] = 8;
  joint_table_["l_knee"]      = 9;
  joint_table_["l_ank_pitch"] = 10;
  joint_table_["l_ank_roll"]  = 11;

  // ordered list (must match joint_table_ index)
  joint_names_ = {
    "r_hip_yaw","r_hip_roll","r_hip_pitch","r_knee","r_ank_pitch","r_ank_roll",
    "l_hip_yaw","l_hip_roll","l_hip_pitch","l_knee","l_ank_pitch","l_ank_roll"
  };

  // Default walking parameters (tuned for RHp: smaller robot)
  walking_param_.init_x_offset       = -0.010;
  walking_param_.init_y_offset        =  0.005;
  walking_param_.init_z_offset        =  0.010;
  walking_param_.init_roll_offset     =  0.0;
  walking_param_.init_pitch_offset    =  0.0;
  walking_param_.init_yaw_offset      =  0.0;
  walking_param_.hip_pitch_offset     =  10.0 * DEG2RAD;
  walking_param_.period_time          =  0.600;   // 600 ms
  walking_param_.dsp_ratio            =  0.2;
  walking_param_.step_fb_ratio        =  0.28;
  walking_param_.x_move_amplitude     =  0.0;
  walking_param_.y_move_amplitude     =  0.0;
  walking_param_.z_move_amplitude     =  0.020;   // foot lift height
  walking_param_.angle_move_amplitude =  0.0;
  walking_param_.move_aim_on          =  false;
  walking_param_.y_swap_amplitude     =  0.018;
  walking_param_.z_swap_amplitude     =  0.005;
  walking_param_.pelvis_offset        =  3.0 * DEG2RAD;

  // Phase shifts (same as OP3)
  x_swap_phase_shift_    = M_PI;
  x_swap_amplitude_shift_= 0.0;
  x_move_phase_shift_    = M_PI / 2.0;
  x_move_amplitude_shift_= 0.0;
  y_swap_phase_shift_    = 0.0;
  y_swap_amplitude_shift_= 0.0;
  y_move_phase_shift_    = M_PI / 2.0;
  z_swap_phase_shift_    = M_PI * 3.0 / 2.0;
  z_move_phase_shift_    = M_PI / 2.0;
  a_move_phase_shift_    = M_PI / 2.0;

  // Load yaml if present
  this->declare_parameter<std::string>("walking_param_path", "");
  std::string param_path = this->get_parameter("walking_param_path").as_string();
  if (!param_path.empty())
    loadWalkingParam(param_path);

  updateTimeParam();
  updateMovementParam();

  // Action client
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/leg_controller/follow_joint_trajectory");

  // Subscriber
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/walking/command", 10,
    std::bind(&WalkingModule::walkingCommandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "WalkingModule initialized (kinematic-only, no IMU).");
}

WalkingModule::~WalkingModule()
{
  delete kd_;
}

// ---------------------------------------------------------------------------
// Command callback
// ---------------------------------------------------------------------------
void WalkingModule::walkingCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "start")
    startWalking();
  else if (msg->data == "stop")
    stop();
  else
    RCLCPP_WARN(this->get_logger(), "Unknown command: %s", msg->data.c_str());
}

// ---------------------------------------------------------------------------
// startGaitCycle — 한 보행 사이클(30포인트)을 계산해 action으로 전송
// ---------------------------------------------------------------------------
void WalkingModule::startGaitCycle()
{
  if (!real_running_) return;

  if (!action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not ready!");
    real_running_ = false;
    return;
  }

  const double time_unit = control_cycle_ms_ * 0.001;
  const int n_points = static_cast<int>(std::round(period_time_ / time_unit));

  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = joint_names_;

  for (int i = 0; i < n_points; i++) {
    processPhase(time_unit);

    double angle[JOINT_NUM] = {};
    if (!computeLegAngle(angle)) {
      RCLCPP_WARN(this->get_logger(), "IK failed at step %d/%d", i, n_points);
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = std::vector<double>(angle, angle + JOINT_NUM);
    pt.time_from_start = rclcpp::Duration::from_seconds((i + 1) * time_unit);
    goal_msg.trajectory.points.push_back(pt);

    time_ += time_unit;
    if (time_ >= period_time_) {
      time_ = 0.0;
      previous_x_move_amplitude_ = walking_param_.x_move_amplitude * 0.5;
    }
  }

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult &) {
      if (ctrl_running_) {
        startGaitCycle();
      } else {
        real_running_ = false;
        RCLCPP_INFO(this->get_logger(), "Walking stopped.");
      }
    };

  action_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Gait cycle sent (%d points, %.1fms).", n_points, period_time_ * 1000.0);
}

// ---------------------------------------------------------------------------
// wSin
// ---------------------------------------------------------------------------
double WalkingModule::wSin(double time, double period, double period_shift,
                           double mag, double mag_shift)
{
  return mag * std::sin(2.0 * M_PI / period * time - period_shift) + mag_shift;
}

// ---------------------------------------------------------------------------
// updateTimeParam
// ---------------------------------------------------------------------------
void WalkingModule::updateTimeParam()
{
  period_time_  = walking_param_.period_time;
  dsp_ratio_    = walking_param_.dsp_ratio;
  ssp_ratio_    = 1.0 - dsp_ratio_;

  x_swap_period_time_ = period_time_ / 2.0;
  x_move_period_time_ = period_time_ * ssp_ratio_;
  y_swap_period_time_ = period_time_;
  y_move_period_time_ = period_time_ * ssp_ratio_;
  z_swap_period_time_ = period_time_ / 2.0;
  z_move_period_time_ = period_time_ * ssp_ratio_ / 2.0;
  a_move_period_time_ = period_time_ * ssp_ratio_;

  ssp_time_         = period_time_ * ssp_ratio_;
  l_ssp_start_time_ = (1.0 - ssp_ratio_) * period_time_ / 4.0;
  l_ssp_end_time_   = (1.0 + ssp_ratio_) * period_time_ / 4.0;
  r_ssp_start_time_ = (3.0 - ssp_ratio_) * period_time_ / 4.0;
  r_ssp_end_time_   = (3.0 + ssp_ratio_) * period_time_ / 4.0;

  phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2.0;
  phase2_time_ = (l_ssp_end_time_   + r_ssp_start_time_) / 2.0;
  phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_)  / 2.0;

  pelvis_offset_ = walking_param_.pelvis_offset;
  pelvis_swing_  = pelvis_offset_ * 0.35;
}

// ---------------------------------------------------------------------------
// updateMovementParam
// ---------------------------------------------------------------------------
void WalkingModule::updateMovementParam()
{
  x_move_amplitude_ = walking_param_.x_move_amplitude;
  x_swap_amplitude_ = walking_param_.x_move_amplitude * walking_param_.step_fb_ratio;

  if (previous_x_move_amplitude_ == 0.0)
  {
    x_move_amplitude_ *= 0.5;
    x_swap_amplitude_ *= 0.5;
  }

  y_move_amplitude_ = walking_param_.y_move_amplitude / 2.0;
  if (y_move_amplitude_ > 0)
    y_move_amplitude_shift_ = y_move_amplitude_;
  else
    y_move_amplitude_shift_ = -y_move_amplitude_;
  y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;

  z_move_amplitude_       = walking_param_.z_move_amplitude / 2.0;
  z_move_amplitude_shift_ = z_move_amplitude_ / 2.0;
  z_swap_amplitude_       = walking_param_.z_swap_amplitude;
  z_swap_amplitude_shift_ = z_swap_amplitude_;

  if (!walking_param_.move_aim_on)
  {
    a_move_amplitude_ = walking_param_.angle_move_amplitude / 2.0;
    a_move_amplitude_shift_ = (a_move_amplitude_ > 0) ? a_move_amplitude_ : -a_move_amplitude_;
  }
  else
  {
    a_move_amplitude_ = -walking_param_.angle_move_amplitude / 2.0;
    a_move_amplitude_shift_ = (a_move_amplitude_ > 0) ? -a_move_amplitude_ : a_move_amplitude_;
  }
}

// ---------------------------------------------------------------------------
// updatePoseParam
// ---------------------------------------------------------------------------
void WalkingModule::updatePoseParam()
{
  x_offset_        = walking_param_.init_x_offset;
  y_offset_        = walking_param_.init_y_offset;
  z_offset_        = walking_param_.init_z_offset;
  r_offset_        = walking_param_.init_roll_offset;
  p_offset_        = walking_param_.init_pitch_offset;
  a_offset_        = walking_param_.init_yaw_offset;
  hit_pitch_offset_= walking_param_.hip_pitch_offset;
}

// ---------------------------------------------------------------------------
// startWalking / stop
// ---------------------------------------------------------------------------
void WalkingModule::startWalking()
{
  if (is_starting_ || real_running_)
  {
    RCLCPP_WARN(this->get_logger(), "Already starting or walking. Ignoring command.");
    return;
  }

  is_starting_ = true;

  // 1. 차렷 자세로 이동 (2초)
  RCLCPP_INFO(this->get_logger(), "Moving to stand pose (2s)...");
  std::vector<double> stand_pose(JOINT_NUM, 0.0);
  sendTrajectory(stand_pose, 2.0);

  // 2. 2.2초 뒤 첫 사이클 시작
  start_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(2200),
    [this]() {
      start_timer_->cancel();
      updateMovementParam();
      ctrl_running_  = true;
      real_running_  = true;
      time_          = 0.0;
      is_starting_   = false;
      RCLCPP_INFO(this->get_logger(), "Walking started.");
      startGaitCycle();
    });
}

void WalkingModule::stop()
{
  ctrl_running_ = false;
  RCLCPP_INFO(this->get_logger(), "Walking stop requested.");
}

// ---------------------------------------------------------------------------
// processPhase
// ---------------------------------------------------------------------------
void WalkingModule::processPhase(double time_unit)
{
  if (time_ == 0.0)
  {
    updateTimeParam();
    phase_ = PHASE0;

    if (!ctrl_running_)
    {
      if (x_move_amplitude_ == 0.0 && y_move_amplitude_ == 0.0 && a_move_amplitude_ == 0.0)
        real_running_ = false;
      else
      {
        walking_param_.x_move_amplitude     = 0.0;
        walking_param_.y_move_amplitude     = 0.0;
        walking_param_.angle_move_amplitude = 0.0;
        previous_x_move_amplitude_          = 0.0;
      }
    }
  }
  else if (time_ >= (phase1_time_ - time_unit / 2.0) && time_ < (phase1_time_ + time_unit / 2.0))
  {
    updateMovementParam();
    updateTimeParam();
    time_  = phase1_time_;
    phase_ = PHASE1;
  }
  else if (time_ >= (phase2_time_ - time_unit / 2.0) && time_ < (phase2_time_ + time_unit / 2.0))
  {
    updateTimeParam();
    time_  = phase2_time_;
    phase_ = PHASE2;

    if (!ctrl_running_)
    {
      if (x_move_amplitude_ == 0.0 && y_move_amplitude_ == 0.0 && a_move_amplitude_ == 0.0)
        real_running_ = false;
      else
      {
        walking_param_.x_move_amplitude     = previous_x_move_amplitude_;
        walking_param_.y_move_amplitude     = 0.0;
        walking_param_.angle_move_amplitude = 0.0;
      }
    }
  }
  else if (time_ >= (phase3_time_ - time_unit / 2.0) && time_ < (phase3_time_ + time_unit / 2.0))
  {
    updateMovementParam();
    updateTimeParam();
    time_  = phase3_time_;
    phase_ = PHASE3;
  }
}

// ---------------------------------------------------------------------------
// computeLegAngle
// ---------------------------------------------------------------------------
bool WalkingModule::computeLegAngle(double *leg_angle)
{
  Pose3D swap, right_leg_move, left_leg_move;
  double pelvis_offset_r, pelvis_offset_l;
  double ep[12];

  updatePoseParam();

  // Body swap (common oscillation)
  swap.x   = wSin(time_, x_swap_period_time_, x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_);
  swap.y   = wSin(time_, y_swap_period_time_, y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_);
  swap.z   = wSin(time_, z_swap_period_time_, z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_);
  swap.roll = swap.pitch = swap.yaw = 0.0;

  // Leg trajectories — same 5-region logic as OP3
  if (time_ <= l_ssp_start_time_)  // double support before L swing
  {
    left_leg_move.x   = wSin(l_ssp_start_time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*l_ssp_start_time_, x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y   = wSin(l_ssp_start_time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*l_ssp_start_time_, y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z   = wSin(l_ssp_start_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*l_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*l_ssp_start_time_, a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x  = wSin(l_ssp_start_time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*l_ssp_start_time_, -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y  = wSin(l_ssp_start_time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*l_ssp_start_time_, -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z  = wSin(r_ssp_start_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*r_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw= wSin(l_ssp_start_time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*l_ssp_start_time_, -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0.0; pelvis_offset_r = 0.0;
  }
  else if (time_ <= l_ssp_end_time_)  // L single support
  {
    left_leg_move.x   = wSin(time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*l_ssp_start_time_, x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y   = wSin(time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*l_ssp_start_time_, y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z   = wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*l_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*l_ssp_start_time_, a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x  = wSin(time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*l_ssp_start_time_, -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y  = wSin(time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*l_ssp_start_time_, -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z  = wSin(r_ssp_start_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*r_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw= wSin(time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*l_ssp_start_time_, -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*l_ssp_start_time_, pelvis_swing_/2.0, pelvis_swing_/2.0);
    pelvis_offset_r = wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*l_ssp_start_time_, -pelvis_offset_/2.0, -pelvis_offset_/2.0);
  }
  else if (time_ <= r_ssp_start_time_)  // double support between L and R swing
  {
    left_leg_move.x   = wSin(l_ssp_end_time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*l_ssp_start_time_, x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y   = wSin(l_ssp_end_time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*l_ssp_start_time_, y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z   = wSin(l_ssp_end_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*l_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*l_ssp_start_time_, a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x  = wSin(l_ssp_end_time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*l_ssp_start_time_, -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y  = wSin(l_ssp_end_time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*l_ssp_start_time_, -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z  = wSin(r_ssp_start_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*r_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw= wSin(l_ssp_end_time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*l_ssp_start_time_, -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0.0; pelvis_offset_r = 0.0;
  }
  else if (time_ <= r_ssp_end_time_)  // R single support
  {
    left_leg_move.x   = wSin(time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*r_ssp_start_time_+M_PI, x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y   = wSin(time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*r_ssp_start_time_+M_PI, y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z   = wSin(l_ssp_end_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*l_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*r_ssp_start_time_+M_PI, a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x  = wSin(time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*r_ssp_start_time_+M_PI, -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y  = wSin(time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*r_ssp_start_time_+M_PI, -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z  = wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*r_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw= wSin(time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*r_ssp_start_time_+M_PI, -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*r_ssp_start_time_, pelvis_offset_/2.0, pelvis_offset_/2.0);
    pelvis_offset_r = wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*r_ssp_start_time_, -pelvis_swing_/2.0, -pelvis_swing_/2.0);
  }
  else  // double support after R swing
  {
    left_leg_move.x   = wSin(r_ssp_end_time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*r_ssp_start_time_+M_PI, x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y   = wSin(r_ssp_end_time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*r_ssp_start_time_+M_PI, y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z   = wSin(l_ssp_end_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*l_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*r_ssp_start_time_+M_PI, a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x  = wSin(r_ssp_end_time_, x_move_period_time_, x_move_phase_shift_ + 2*M_PI/x_move_period_time_*r_ssp_start_time_+M_PI, -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y  = wSin(r_ssp_end_time_, y_move_period_time_, y_move_phase_shift_ + 2*M_PI/y_move_period_time_*r_ssp_start_time_+M_PI, -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z  = wSin(r_ssp_end_time_, z_move_period_time_, z_move_phase_shift_ + 2*M_PI/z_move_period_time_*r_ssp_start_time_, z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw= wSin(r_ssp_end_time_, a_move_period_time_, a_move_phase_shift_ + 2*M_PI/a_move_period_time_*r_ssp_start_time_+M_PI, -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0.0; pelvis_offset_r = 0.0;
  }

  left_leg_move.roll  = 0.0; left_leg_move.pitch  = 0.0;
  right_leg_move.roll = 0.0; right_leg_move.pitch = 0.0;

  double leg_length = kd_->thigh_length_m_ + kd_->calf_length_m_ + kd_->ankle_length_m_;

  // Endpoint positions (in pelvis frame)
  ep[0]  = swap.x + right_leg_move.x + x_offset_;
  ep[1]  = swap.y + right_leg_move.y - y_offset_ / 2.0;
  ep[2]  = swap.z + right_leg_move.z + z_offset_ - leg_length;
  ep[3]  = swap.roll  + right_leg_move.roll  - r_offset_ / 2.0;
  ep[4]  = swap.pitch + right_leg_move.pitch + p_offset_;
  ep[5]  = swap.yaw   + right_leg_move.yaw   - a_offset_ / 2.0;
  ep[6]  = swap.x + left_leg_move.x + x_offset_;
  ep[7]  = swap.y + left_leg_move.y + y_offset_ / 2.0;
  ep[8]  = swap.z + left_leg_move.z + z_offset_ - leg_length;
  ep[9]  = swap.roll  + left_leg_move.roll   + r_offset_ / 2.0;
  ep[10] = swap.pitch + left_leg_move.pitch  + p_offset_;
  ep[11] = swap.yaw   + left_leg_move.yaw    + a_offset_ / 2.0;

  // Body swing
  if (time_ <= l_ssp_end_time_) { body_swing_y_ = -ep[7]; body_swing_z_ = ep[8]; }
  else                          { body_swing_y_ = -ep[1]; body_swing_z_ = ep[2]; }
  body_swing_z_ -= leg_length;

  // Solve IK
  if (!kd_->calcInverseKinematicsForRightLeg(&leg_angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]))
  {
    RCLCPP_WARN(this->get_logger(), "IK failed R: %.3f %.3f %.3f", ep[0], ep[1], ep[2]);
    return false;
  }
  if (!kd_->calcInverseKinematicsForLeftLeg(&leg_angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]))
  {
    RCLCPP_WARN(this->get_logger(), "IK failed L: %.3f %.3f %.3f", ep[6], ep[7], ep[8]);
    return false;
  }

  // Pelvis offset correction (hip roll)
  leg_angle[joint_table_["r_hip_roll"]] += kd_->getJointDirection("r_hip_roll") * pelvis_offset_r;
  leg_angle[joint_table_["l_hip_roll"]] += kd_->getJointDirection("l_hip_roll") * pelvis_offset_l;

  // Hip pitch offset
  leg_angle[joint_table_["r_hip_pitch"]] -= kd_->getJointDirection("r_hip_pitch") * hit_pitch_offset_;
  leg_angle[joint_table_["l_hip_pitch"]] -= kd_->getJointDirection("l_hip_pitch") * hit_pitch_offset_;

  return true;
}

// ---------------------------------------------------------------------------
// loadWalkingParam
// ---------------------------------------------------------------------------
void WalkingModule::loadWalkingParam(const std::string &path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load param file: %s", e.what());
    return;
  }

  walking_param_.init_x_offset       = doc["x_offset"].as<double>();
  walking_param_.init_y_offset        = doc["y_offset"].as<double>();
  walking_param_.init_z_offset        = doc["z_offset"].as<double>();
  walking_param_.init_roll_offset     = doc["roll_offset"].as<double>()  * DEG2RAD;
  walking_param_.init_pitch_offset    = doc["pitch_offset"].as<double>() * DEG2RAD;
  walking_param_.init_yaw_offset      = doc["yaw_offset"].as<double>()   * DEG2RAD;
  walking_param_.hip_pitch_offset     = doc["hip_pitch_offset"].as<double>() * DEG2RAD;
  walking_param_.period_time          = doc["period_time"].as<double>() * 0.001;
  walking_param_.dsp_ratio            = doc["dsp_ratio"].as<double>();
  walking_param_.step_fb_ratio        = doc["step_forward_back_ratio"].as<double>();
  walking_param_.x_move_amplitude     = doc["x_move_amplitude"].as<double>();
  walking_param_.z_move_amplitude     = doc["foot_height"].as<double>();
  walking_param_.y_swap_amplitude     = doc["swing_right_left"].as<double>();
  walking_param_.z_swap_amplitude     = doc["swing_top_down"].as<double>();
  walking_param_.pelvis_offset        = doc["pelvis_offset"].as<double>() * DEG2RAD;

  RCLCPP_INFO(this->get_logger(), "Walking param loaded from: %s", path.c_str());
}

// ---------------------------------------------------------------------------
// sendTrajectory  — action 방식 (rhphumanoid_walking_pattern과 동일)
// ---------------------------------------------------------------------------
void WalkingModule::sendTrajectory(const std::vector<double> &positions, double move_time)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after 10s! Skipping stand pose.");
    return;
  }

  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions       = positions;
  pt.time_from_start = rclcpp::Duration::from_seconds(move_time);
  goal_msg.trajectory.points.push_back(pt);

  action_client_->async_send_goal(goal_msg);
}

} // namespace rhp
