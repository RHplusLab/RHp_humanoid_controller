#ifndef RHPHUMANOID_WALKING_PARAMETER_H_
#define RHPHUMANOID_WALKING_PARAMETER_H_

struct WalkingParam
{
  // init pose offsets (m, rad)
  double init_x_offset;
  double init_y_offset;
  double init_z_offset;
  double init_roll_offset;
  double init_pitch_offset;
  double init_yaw_offset;
  double hip_pitch_offset;

  // time (s)
  double period_time;
  double dsp_ratio;
  double step_fb_ratio;

  // movement (m, rad)
  double x_move_amplitude;
  double y_move_amplitude;
  double z_move_amplitude;      // foot height
  double angle_move_amplitude;
  bool   move_aim_on;

  // swing
  double y_swap_amplitude;
  double z_swap_amplitude;
  double pelvis_offset;
};

#endif /* RHPHUMANOID_WALKING_PARAMETER_H_ */
