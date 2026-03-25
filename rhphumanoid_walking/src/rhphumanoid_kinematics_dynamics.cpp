#include <cmath>
#include <iostream>
#include "rhphumanoid_walking/rhphumanoid_kinematics_dynamics.h"

namespace rhp
{

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
static double getSign(double num) { return (num < 0) ? -1.0 : 1.0; }

static Eigen::Matrix3d getRotX(double rad)
{
  Eigen::Matrix3d R;
  R << 1,        0,         0,
       0, cos(rad), -sin(rad),
       0, sin(rad),  cos(rad);
  return R;
}

static Eigen::Matrix3d getRotY(double rad)
{
  Eigen::Matrix3d R;
  R <<  cos(rad), 0, sin(rad),
              0,  1,        0,
       -sin(rad), 0, cos(rad);
  return R;
}

static Eigen::Matrix3d getRotZ(double rad)
{
  Eigen::Matrix3d R;
  R << cos(rad), -sin(rad), 0,
       sin(rad),  cos(rad), 0,
              0,         0, 1;
  return R;
}

static Eigen::Matrix3d convertRPYToRotation(double r, double p, double y)
{
  return getRotZ(y) * getRotY(p) * getRotX(r);
}

// ---------------------------------------------------------------------------
// RHpKinematicsDynamics
// ---------------------------------------------------------------------------
RHpKinematicsDynamics::RHpKinematicsDynamics()
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    link_data_[id] = new LinkData();

  initLinkData();
}

RHpKinematicsDynamics::~RHpKinematicsDynamics()
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    delete link_data_[id];
}

void RHpKinematicsDynamics::initLinkData()
{
  // -------------------------------------------------------------------------
  // base (pelvis)
  // -------------------------------------------------------------------------
  link_data_[ID_BASE]->name_              = "base";
  link_data_[ID_BASE]->parent_            = -1;
  link_data_[ID_BASE]->sibling_           = -1;
  link_data_[ID_BASE]->child_             = ID_R_HIP_YAW;
  link_data_[ID_BASE]->relative_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  link_data_[ID_BASE]->joint_axis_        = Eigen::Vector3d(0.0, 0.0, 0.0);

  // -------------------------------------------------------------------------
  // RIGHT LEG
  // From URDF (rhphumanoid.structure.rleg.xacro):
  //   r_hip_yaw   origin: (0.017, -0.034930, -0.045670)
  //   r_hip_roll  origin: (-0.015830, 0.0, -0.035400)
  //   r_hip_pitch origin: (0.016330, -0.025420, 0.0)
  //   r_knee      origin: (0.0, 0.0, -0.072100)
  //   r_ank_pitch origin: (0.0, 0.0, -0.072100)
  //   r_ank_roll  origin: (-0.016330, 0.025420, 0.0)
  // -------------------------------------------------------------------------

  // r_hip_yaw
  link_data_[ID_R_HIP_YAW]->name_              = "r_hip_yaw";
  link_data_[ID_R_HIP_YAW]->parent_            = ID_BASE;
  link_data_[ID_R_HIP_YAW]->sibling_           = ID_L_HIP_YAW;
  link_data_[ID_R_HIP_YAW]->child_             = ID_R_HIP_ROLL;
  link_data_[ID_R_HIP_YAW]->relative_position_ = Eigen::Vector3d(0.017000, -0.034930, -0.045670);
  link_data_[ID_R_HIP_YAW]->joint_axis_        = Eigen::Vector3d(0.0, 0.0, -1.0);

  // r_hip_roll
  link_data_[ID_R_HIP_ROLL]->name_              = "r_hip_roll";
  link_data_[ID_R_HIP_ROLL]->parent_            = ID_R_HIP_YAW;
  link_data_[ID_R_HIP_ROLL]->sibling_           = -1;
  link_data_[ID_R_HIP_ROLL]->child_             = ID_R_HIP_PITCH;
  link_data_[ID_R_HIP_ROLL]->relative_position_ = Eigen::Vector3d(-0.015830, 0.000000, -0.035400);
  link_data_[ID_R_HIP_ROLL]->joint_axis_        = Eigen::Vector3d(-1.0, 0.0, 0.0);

  // r_hip_pitch
  link_data_[ID_R_HIP_PITCH]->name_              = "r_hip_pitch";
  link_data_[ID_R_HIP_PITCH]->parent_            = ID_R_HIP_ROLL;
  link_data_[ID_R_HIP_PITCH]->sibling_           = -1;
  link_data_[ID_R_HIP_PITCH]->child_             = ID_R_KNEE;
  link_data_[ID_R_HIP_PITCH]->relative_position_ = Eigen::Vector3d(0.016330, -0.025420, 0.000000);
  link_data_[ID_R_HIP_PITCH]->joint_axis_        = Eigen::Vector3d(0.0, -1.0, 0.0);

  // r_knee
  link_data_[ID_R_KNEE]->name_              = "r_knee";
  link_data_[ID_R_KNEE]->parent_            = ID_R_HIP_PITCH;
  link_data_[ID_R_KNEE]->sibling_           = -1;
  link_data_[ID_R_KNEE]->child_             = ID_R_ANK_PITCH;
  link_data_[ID_R_KNEE]->relative_position_ = Eigen::Vector3d(0.000000, 0.000000, -0.072100);
  link_data_[ID_R_KNEE]->joint_axis_        = Eigen::Vector3d(0.0, -1.0, 0.0);

  // r_ank_pitch
  link_data_[ID_R_ANK_PITCH]->name_              = "r_ank_pitch";
  link_data_[ID_R_ANK_PITCH]->parent_            = ID_R_KNEE;
  link_data_[ID_R_ANK_PITCH]->sibling_           = -1;
  link_data_[ID_R_ANK_PITCH]->child_             = ID_R_ANK_ROLL;
  link_data_[ID_R_ANK_PITCH]->relative_position_ = Eigen::Vector3d(0.000000, 0.000000, -0.072100);
  link_data_[ID_R_ANK_PITCH]->joint_axis_        = Eigen::Vector3d(0.0, 1.0, 0.0);

  // r_ank_roll
  link_data_[ID_R_ANK_ROLL]->name_              = "r_ank_roll";
  link_data_[ID_R_ANK_ROLL]->parent_            = ID_R_ANK_PITCH;
  link_data_[ID_R_ANK_ROLL]->sibling_           = -1;
  link_data_[ID_R_ANK_ROLL]->child_             = ID_R_LEG_END;
  link_data_[ID_R_ANK_ROLL]->relative_position_ = Eigen::Vector3d(-0.016330, 0.025420, 0.000000);
  link_data_[ID_R_ANK_ROLL]->joint_axis_        = Eigen::Vector3d(1.0, 0.0, 0.0);

  // r_leg_end (foot contact point)
  link_data_[ID_R_LEG_END]->name_              = "r_leg_end";
  link_data_[ID_R_LEG_END]->parent_            = ID_R_ANK_ROLL;
  link_data_[ID_R_LEG_END]->sibling_           = -1;
  link_data_[ID_R_LEG_END]->child_             = -1;
  link_data_[ID_R_LEG_END]->relative_position_ = Eigen::Vector3d(0.016330, -0.025420, -0.038000);
  link_data_[ID_R_LEG_END]->joint_axis_        = Eigen::Vector3d(0.0, 0.0, 0.0);

  // -------------------------------------------------------------------------
  // LEFT LEG (symmetric to right leg, y-axis mirrored)
  // -------------------------------------------------------------------------

  // l_hip_yaw
  link_data_[ID_L_HIP_YAW]->name_              = "l_hip_yaw";
  link_data_[ID_L_HIP_YAW]->parent_            = ID_BASE;
  link_data_[ID_L_HIP_YAW]->sibling_           = -1;
  link_data_[ID_L_HIP_YAW]->child_             = ID_L_HIP_ROLL;
  link_data_[ID_L_HIP_YAW]->relative_position_ = Eigen::Vector3d(0.017000,  0.034930, -0.045670);
  link_data_[ID_L_HIP_YAW]->joint_axis_        = Eigen::Vector3d(0.0, 0.0, -1.0);

  // l_hip_roll
  link_data_[ID_L_HIP_ROLL]->name_              = "l_hip_roll";
  link_data_[ID_L_HIP_ROLL]->parent_            = ID_L_HIP_YAW;
  link_data_[ID_L_HIP_ROLL]->sibling_           = -1;
  link_data_[ID_L_HIP_ROLL]->child_             = ID_L_HIP_PITCH;
  link_data_[ID_L_HIP_ROLL]->relative_position_ = Eigen::Vector3d(-0.015830, 0.000000, -0.035400);
  link_data_[ID_L_HIP_ROLL]->joint_axis_        = Eigen::Vector3d(-1.0, 0.0, 0.0);

  // l_hip_pitch
  link_data_[ID_L_HIP_PITCH]->name_              = "l_hip_pitch";
  link_data_[ID_L_HIP_PITCH]->parent_            = ID_L_HIP_ROLL;
  link_data_[ID_L_HIP_PITCH]->sibling_           = -1;
  link_data_[ID_L_HIP_PITCH]->child_             = ID_L_KNEE;
  link_data_[ID_L_HIP_PITCH]->relative_position_ = Eigen::Vector3d(0.016330,  0.025420, 0.000000);
  link_data_[ID_L_HIP_PITCH]->joint_axis_        = Eigen::Vector3d(0.0, 1.0, 0.0);

  // l_knee
  link_data_[ID_L_KNEE]->name_              = "l_knee";
  link_data_[ID_L_KNEE]->parent_            = ID_L_HIP_PITCH;
  link_data_[ID_L_KNEE]->sibling_           = -1;
  link_data_[ID_L_KNEE]->child_             = ID_L_ANK_PITCH;
  link_data_[ID_L_KNEE]->relative_position_ = Eigen::Vector3d(0.000000, 0.000000, -0.072100);
  link_data_[ID_L_KNEE]->joint_axis_        = Eigen::Vector3d(0.0, 1.0, 0.0);

  // l_ank_pitch
  link_data_[ID_L_ANK_PITCH]->name_              = "l_ank_pitch";
  link_data_[ID_L_ANK_PITCH]->parent_            = ID_L_KNEE;
  link_data_[ID_L_ANK_PITCH]->sibling_           = -1;
  link_data_[ID_L_ANK_PITCH]->child_             = ID_L_ANK_ROLL;
  link_data_[ID_L_ANK_PITCH]->relative_position_ = Eigen::Vector3d(0.000000, 0.000000, -0.072100);
  link_data_[ID_L_ANK_PITCH]->joint_axis_        = Eigen::Vector3d(0.0, -1.0, 0.0);

  // l_ank_roll
  link_data_[ID_L_ANK_ROLL]->name_              = "l_ank_roll";
  link_data_[ID_L_ANK_ROLL]->parent_            = ID_L_ANK_PITCH;
  link_data_[ID_L_ANK_ROLL]->sibling_           = -1;
  link_data_[ID_L_ANK_ROLL]->child_             = ID_L_LEG_END;
  link_data_[ID_L_ANK_ROLL]->relative_position_ = Eigen::Vector3d(-0.016330, -0.025420, 0.000000);
  link_data_[ID_L_ANK_ROLL]->joint_axis_        = Eigen::Vector3d(1.0, 0.0, 0.0);

  // l_leg_end
  link_data_[ID_L_LEG_END]->name_              = "l_leg_end";
  link_data_[ID_L_LEG_END]->parent_            = ID_L_ANK_ROLL;
  link_data_[ID_L_LEG_END]->sibling_           = -1;
  link_data_[ID_L_LEG_END]->child_             = -1;
  link_data_[ID_L_LEG_END]->relative_position_ = Eigen::Vector3d(0.016330,  0.025420, -0.038000);
  link_data_[ID_L_LEG_END]->joint_axis_        = Eigen::Vector3d(0.0, 0.0, 0.0);

  // -------------------------------------------------------------------------
  // Leg geometry constants
  //   thigh : from r_hip_pitch origin to r_knee origin   = |z| = 0.0721 m
  //   calf  : from r_knee origin to r_ank_pitch origin   = |z| = 0.0721 m
  //   ankle : from r_ank_roll  origin to foot contact    = |z| = 0.0380 m
  //   hip_pitch_offset : x-offset of knee from hip_pitch = 0 m
  // -------------------------------------------------------------------------
  thigh_length_m_      = 0.072100;
  calf_length_m_       = 0.072100;
  ankle_length_m_      = 0.038000;
  hip_pitch_offset_m_  = 0.0;
  hip_offset_angle_rad_= 0.0;
  leg_side_offset_m_   = 2.0 * 0.034930;  // = 0.069860 m
}

// ---------------------------------------------------------------------------
// Forward Kinematics (recursive)
// ---------------------------------------------------------------------------
void RHpKinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == ID_BASE)
  {
    link_data_[ID_BASE]->position_.setZero();
    link_data_[ID_BASE]->orientation_.setIdentity();
  }
  else
  {
    int parent = link_data_[joint_id]->parent_;
    const double angle = link_data_[joint_id]->joint_angle_;
    const Eigen::Vector3d &axis = link_data_[joint_id]->joint_axis_;

    // Rodrigues rotation
    Eigen::AngleAxisd aa(angle, axis.normalized());
    Eigen::Matrix3d Rj = (axis.norm() > 1e-9) ? aa.toRotationMatrix() : Eigen::Matrix3d::Identity();

    link_data_[joint_id]->position_ =
      link_data_[parent]->orientation_ * link_data_[joint_id]->relative_position_
      + link_data_[parent]->position_;

    link_data_[joint_id]->orientation_ =
      link_data_[parent]->orientation_ * Rj;
  }

  calcForwardKinematics(link_data_[joint_id]->sibling_);
  calcForwardKinematics(link_data_[joint_id]->child_);
}

// ---------------------------------------------------------------------------
// Inverse Kinematics for one leg
// (same closed-form algorithm as OP3, adjusted for RHp link lengths)
//
// out[0] = hip_yaw,  out[1] = hip_roll,  out[2] = hip_pitch,
// out[3] = knee,     out[4] = ank_pitch, out[5] = ank_roll
//
// x,y,z : desired foot position relative to hip_yaw parent (pelvis)
// roll,pitch,yaw : desired foot orientation
// ---------------------------------------------------------------------------
bool RHpKinematicsDynamics::calcInverseKinematicsForLeg(
  double *out,
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  // Desired foot rotation matrix
  Eigen::Matrix3d R06 = convertRPYToRotation(roll, pitch, yaw);

  // Vector from hip to ankle centre (subtract ankle_length along foot z-axis)
  Eigen::Vector3d p06(x, y, z);
  p06 += ankle_length_m_ * R06.col(2);

  // q6 (ankle roll)
  Eigen::Vector3d p60 = -R06.transpose() * p06;
  *(out + 5) = std::atan2(p60(1), p60(2));

  // q1 (hip yaw)
  Eigen::Matrix3d R05 = R06 * getRotX(-(*(out + 5)));
  *(out + 0) = std::atan2(-R05(0, 1), R05(1, 1));

  // q4 (knee)
  // hip_pitch_offset_m_ = 0 for RHp, so p03 = zero vector
  Eigen::Vector3d p03 = getRotZ(*(out + 0)) * Eigen::Vector3d(hip_pitch_offset_m_, 0.0, 0.0);
  Eigen::Vector3d p36 = p06 - p03;
  double p36_norm = p36.norm();

  double cos_q4 = (thigh_length_m_ * thigh_length_m_ + calf_length_m_ * calf_length_m_
                   - p36_norm * p36_norm)
                  / (2.0 * thigh_length_m_ * calf_length_m_);
  // clamp to [-1, 1]
  cos_q4 = std::max(-1.0, std::min(1.0, cos_q4));
  *(out + 3) = -std::acos(cos_q4) + M_PI;

  // q5 (ankle pitch)
  double alpha = std::asin(thigh_length_m_ * std::sin(M_PI - *(out + 3)) / p36_norm);
  Eigen::Vector3d p63 = -R06.transpose() * p36;
  *(out + 4) = -std::atan2(p63(0), getSign(p63(2)) * std::sqrt(p63(1)*p63(1) + p63(2)*p63(2))) - alpha;

  // q2, q3 (hip roll, hip pitch)
  Eigen::Matrix3d R13 =
    getRotZ(-(*(out + 0))) * R05 * getRotY(-(*(out + 4) + *(out + 3)));
  *(out + 1) = std::atan2(R13(2, 1), R13(1, 1));
  *(out + 2) = std::atan2(R13(0, 2), R13(0, 0));

  // Apply hip offset angle (= 0 for RHp since hip_pitch_offset_m_ = 0)
  *(out + 2) += hip_offset_angle_rad_;
  *(out + 3) -= hip_offset_angle_rad_;

  return true;
}

// ---------------------------------------------------------------------------
// Right leg IK  — apply joint direction signs
// ---------------------------------------------------------------------------
bool RHpKinematicsDynamics::calcInverseKinematicsForRightLeg(
  double *out,
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  if (!calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw))
    return false;

  for (int i = 0; i < 6; i++)
    out[i] *= getJointDirection(ID_R_LEG_START + i);

  return true;
}

// ---------------------------------------------------------------------------
// Left leg IK  — apply joint direction signs
// ---------------------------------------------------------------------------
bool RHpKinematicsDynamics::calcInverseKinematicsForLeftLeg(
  double *out,
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  if (!calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw))
    return false;

  for (int i = 0; i < 6; i++)
    out[i] *= getJointDirection(ID_L_LEG_START + i);

  return true;
}

// ---------------------------------------------------------------------------
// Joint direction  (sum of joint_axis components = sign)
// ---------------------------------------------------------------------------
double RHpKinematicsDynamics::getJointDirection(int link_id)
{
  if (link_id < 0 || link_id > ALL_JOINT_ID)
    return 0.0;
  const Eigen::Vector3d &ax = link_data_[link_id]->joint_axis_;
  return ax(0) + ax(1) + ax(2);
}

double RHpKinematicsDynamics::getJointDirection(const std::string &link_name)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    if (link_data_[id]->name_ == link_name)
      return getJointDirection(id);
  return 0.0;
}

} // namespace rhp
