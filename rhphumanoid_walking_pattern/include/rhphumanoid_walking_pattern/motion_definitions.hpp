#ifndef MOTION_DEFINITIONS_HPP
#define MOTION_DEFINITIONS_HPP

#include <vector>
#include <string>

namespace rhp_motions {

    // [핵심] 하나의 동작 단위를 정의하는 구조체
    struct MotionStep {
        std::vector<double> positions; // 관절 12개 각도
        double move_time;              // 이동하는 데 걸리는 시간 (초)
        double stop_time;              // 도달 후 멈춰있는 시간 (초)
    };

    // -------------------------------------------------------------------------
    // 기본 자세 데이터 (재사용을 위해 변수로 분리)
    // -------------------------------------------------------------------------
    const std::vector<double> POSE_STAND = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Left: Y, R, P, K, AP, AR
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0   // Right: Y, R, P, K, AP, AR
    };

    const std::vector<double> POSE_LEFT_UP = {
        0.0, 0.0, -0.3, 0.6, -0.3, 0.0, // Left (들기)
        0.0, 0.0,  0.0, 0.0,  0.0, 0.0  // Right
    };

    const std::vector<double> POSE_RIGHT_UP = {
        0.0, 0.0,  0.0, 0.0,  0.0, 0.0, // Left
        0.0, 0.0,  0.3, -0.6, 0.3, 0.0  // Right (들기)
    };

    // -------------------------------------------------------------------------
    // 시퀀스 정의 (시간 정보 포함!)
    // -------------------------------------------------------------------------

    // 1. 차렷 시퀀스 (2초 동안 천천히 이동)
    const std::vector<MotionStep> SEQ_INIT_STAND = {
        { POSE_STAND, 2.0, 1.0 }
    };

    // 2. 제자리 걷기 패턴 (왼발 들기 -> 내리기 -> 오른발 들기 -> 내리기)
    const std::vector<MotionStep> SEQ_WALK_IN_PLACE = {
        // { 각도, 이동시간, 정지시간 }
        { POSE_LEFT_UP,  0.5, 0.2 }, // Step 1
        { POSE_STAND,    0.5, 0.1 }, // Step 2
        { POSE_RIGHT_UP, 0.5, 0.2 }, // Step 3
        { POSE_STAND,    0.5, 0.1 }  // Step 4
    };

} // namespace rhp_motions

#endif
