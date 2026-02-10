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
        0.0, 0.0, 0.0, 0.0, 0.0,  // Left
        0.0, 0.0, 0.0, 0.0, 0.0   // Right
    };

    // 2. 왼팔 들기 (앞으로)
    const std::vector<double> POSE_LEFT_UP = {
        -1.0, 0.0, 0.0, 0.0, 0.0,  // Left (Pitch -1.0)
         0.0, 0.0, 0.0, 0.0, 0.0   // Right
    };

    // 3. 오른팔 들기 (앞으로)
    const std::vector<double> POSE_RIGHT_UP = {
         0.0, 0.0, 0.0, 0.0, 0.0,
        -1.0, 0.0, 0.0, 0.0, 0.0   // Right (Pitch -1.0)
    };

    // 4. 만세 (Both Up)
    const std::vector<double> POSE_HOORAY = {
        -1.5, 0.0, 0.0, 0.0, 0.0,
        -1.5, 0.0, 0.0, 0.0, 0.0
    };

    // -------------------------------------------------------------------------
    // [동작 시퀀스]
    // -------------------------------------------------------------------------
    
    // 왼팔 흔들기
    const std::vector<MotionStep> SEQ_WAVE_LEFT = {
        { POSE_LEFT_UP, 0.8, 0.2 }, // 들기
        { POSE_INIT,    0.8, 0.2 }  // 내리기
    };

    // 오른팔 흔들기
    const std::vector<MotionStep> SEQ_WAVE_RIGHT = {
        { POSE_RIGHT_UP, 0.8, 0.2 },
        { POSE_INIT,     0.8, 0.2 }
    };

    // 만세 삼창 (만세 -> 차렷)
    const std::vector<MotionStep> SEQ_HOORAY = {
        { POSE_HOORAY, 1.0, 0.5 },
        { POSE_INIT,   1.0, 0.5 }
    };

} // namespace rhp_arm_motions

#endif