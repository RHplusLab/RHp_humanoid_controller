#ifndef ARM_DEFINITIONS_HPP
#define ARM_DEFINITIONS_HPP

#include <vector>

namespace rhp_arm_motions {

    // 동작 단위 구조체
    struct MotionStep {
        std::vector<double> positions; // 10개 관절
        double move_time;
        double stop_time;
    };

    // -------------------------------------------------------------------------
    // [기본 포즈] 관절 순서 (10개): 
    // L_Sho_P, L_Sho_R, L_El, L_Wst, L_Grp, R_Sho_P, R_Sho_R, R_El, R_Wst, R_Grp
    // -------------------------------------------------------------------------

    // 1. 차렷 (초기 상태)
    const std::vector<double> POSE_INIT = {
        0.15, -1.45, 0.00, -0.50, 0.00,  // Left
        -0.15, 1.45, 0.00, 0.50, 0.00    // Right
    };

    // 2. 오른팔 안쪽으로 들기
    const std::vector<double> POSE_WAVE_IN = {
        0.15, -1.50, 0.00, -0.50, 0.00,
        -0.15, -1.50, 1.50, 1.00, 0.10
    };

    // 3. 오른팔 바깥쪽으로 들기
    const std::vector<double> POSE_WAVE_OUT = {
        0.15, -1.40, 0.00, -0.50, 0.00,
        -0.15, -0.60, 1.50, 0.00, -0.10
    };

    // -------------------------------------------------------------------------
    // [동작 시퀀스]
    // -------------------------------------------------------------------------
    
    // 오른팔 들기
    const std::vector<MotionStep> SEQ_RAISE_ARM = {
        { POSE_WAVE_IN, 1.2, 0.8 }
    };

    // 오른팔 흔들기
    const std::vector<MotionStep> SEQ_WAVE_ARM = {
        { POSE_WAVE_OUT, 0.4, 0.6 },
        { POSE_WAVE_IN,  0.4, 0.6 }
    };

    // 오른팔 내리기
    const std::vector<MotionStep> SEQ_LOWER_ARM = {
        { POSE_INIT, 1.2, 0.8 }
    };

} // namespace rhp_arm_motions

#endif