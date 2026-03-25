#ifndef RHPHUMANOID_KINEMATICS_DYNAMICS_H_
#define RHPHUMANOID_KINEMATICS_DYNAMICS_H_

#include <eigen3/Eigen/Eigen>
#include "rhphumanoid_kinematics_dynamics_define.h"
#include "link_data.h"

namespace rhp
{

class RHpKinematicsDynamics
{
public:
  RHpKinematicsDynamics();
  ~RHpKinematicsDynamics();

  void calcForwardKinematics(int joint_id);

  bool calcInverseKinematicsForLeg(double *out,
    double x, double y, double z,
    double roll, double pitch, double yaw);

  bool calcInverseKinematicsForRightLeg(double *out,
    double x, double y, double z,
    double roll, double pitch, double yaw);

  bool calcInverseKinematicsForLeftLeg(double *out,
    double x, double y, double z,
    double roll, double pitch, double yaw);

  double getJointDirection(const std::string &link_name);
  double getJointDirection(int link_id);

  LinkData *link_data_[ALL_JOINT_ID + 1];

  // Leg geometry (m)
  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double hip_pitch_offset_m_;
  double hip_offset_angle_rad_;
  double leg_side_offset_m_;

private:
  void initLinkData();
};

} // namespace rhp

#endif /* RHPHUMANOID_KINEMATICS_DYNAMICS_H_ */
