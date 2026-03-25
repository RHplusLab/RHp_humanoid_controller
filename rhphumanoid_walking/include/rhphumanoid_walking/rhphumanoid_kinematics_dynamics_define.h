#ifndef RHPHUMANOID_KINEMATICS_DYNAMICS_DEFINE_H_
#define RHPHUMANOID_KINEMATICS_DYNAMICS_DEFINE_H_

namespace rhp
{

#define MAX_JOINT_ID    (14)  // 0~14
#define ALL_JOINT_ID    (14)

#define MAX_LEG_ID      (6)
#define MAX_ITER        (5)

// Link IDs
#define ID_BASE         (0)

#define ID_R_HIP_YAW    (1)
#define ID_R_HIP_ROLL   (2)
#define ID_R_HIP_PITCH  (3)
#define ID_R_KNEE       (4)
#define ID_R_ANK_PITCH  (5)
#define ID_R_ANK_ROLL   (6)
#define ID_R_LEG_END    (7)

#define ID_L_HIP_YAW    (8)
#define ID_L_HIP_ROLL   (9)
#define ID_L_HIP_PITCH  (10)
#define ID_L_KNEE       (11)
#define ID_L_ANK_PITCH  (12)
#define ID_L_ANK_ROLL   (13)
#define ID_L_LEG_END    (14)

// Leg start/end for loop use
#define ID_R_LEG_START  (ID_R_HIP_YAW)
#define ID_L_LEG_START  (ID_L_HIP_YAW)

#define GRAVITY_ACCELERATION (9.8)

} // namespace rhp

#endif /* RHPHUMANOID_KINEMATICS_DYNAMICS_DEFINE_H_ */
