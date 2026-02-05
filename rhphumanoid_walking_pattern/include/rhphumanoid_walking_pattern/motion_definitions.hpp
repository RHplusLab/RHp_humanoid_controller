#ifndef MOTION_DEFINITIONS_HPP
#define MOTION_DEFINITIONS_HPP

#include <vector>

namespace rhp_motions {

    // [구조체] 동작의 최소 단위 (목표 각도 + 이동 시간 + 정지 시간)
    struct MotionStep {
        std::vector<double> positions; // 12개 관절 각도
        double move_time;              // 도달하는 데 걸리는 시간 (초)
        double stop_time;              // 도달 후 대기하는 시간 (초)
    };

    // -------------------------------------------------------------------------
    // 1. 기본 포즈 데이터 (관절 12개)
    // 순서: [Left] HipY, HipR, HipP, Knee, AnkP, AnkR, [Right] HipY, HipR, HipP, Knee, AnkP, AnkR
    // -------------------------------------------------------------------------

    // (1) 차렷 (Stand)
    const std::vector<double> POSE_STAND = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Left Leg
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0   // Right Leg
    };

    // (2) 왼발 들기 (Left Up) - 무릎을 굽혀 다리를 듬
    const std::vector<double> POSE_LEFT_UP = {
        0.0, 0.0, -0.3, 0.6, -0.3, 0.0, // Left: Pitch 관절들을 움직여 듬
        0.0, 0.0,  0.0, 0.0,  0.0, 0.0  // Right: 지지
    };

    // (3) 오른발 들기 (Right Up)
    const std::vector<double> POSE_RIGHT_UP = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   // Left: 지지
        0.0, 0.0, -0.3, 0.6, -0.3, 0.0  // Right: 듬
    };

    // (4) 왼쪽으로 돌기 위한 준비 (Left Turn Pose - 예시)
    // 왼쪽 고관절(Hip Yaw)을 0.3rad 돌림
    const std::vector<double> POSE_TURN_LEFT = {
        -0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    // (5) 뒤로 걷기 위한 준비 (Backward Pose - 예시)
    // 양쪽 고관절을 살짝 뒤로 뺌
    const std::vector<double> POSE_BACKWARD_READY = {
        0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.2, 0.0, 0.0, 0.0
    };


    // -------------------------------------------------------------------------
    // 2. 동작 시퀀스 정의 (Motion Sequences)
    // -------------------------------------------------------------------------

    // [초기화] 차렷 자세로 천천히 이동
    const std::vector<MotionStep> SEQ_INIT_STAND = {
        { POSE_STAND, 2.0, 1.0 }
    };

    // [앞으로 걷기] 한 사이클 (왼발 쿵 -> 오른발 쿵)
    const std::vector<MotionStep> SEQ_WALK_FORWARD = {
        // 1. 왼발 들기 (0.5초 이동, 0.1초 대기)
        { POSE_LEFT_UP,  0.5, 0.1 },
        // 2. 왼발 내리기
        { POSE_STAND,    0.5, 0.1 },
        // 3. 오른발 들기
        { POSE_RIGHT_UP, 0.5, 0.1 },
        // 4. 오른발 내리기
        { POSE_STAND,    0.5, 0.1 }
    };

    // [왼쪽 돌기] (고관절 비틀기 -> 원위치)
    const std::vector<MotionStep> SEQ_TURN_LEFT = {
        // 1. 왼쪽으로 비틀기
        { POSE_TURN_LEFT, 0.8, 0.2 },
        // 2. 원위치
        { POSE_STAND,     0.8, 0.2 }
    };

    // [뒤로 걷기] (뒤로 기울이기 -> 원위치)
    const std::vector<MotionStep> SEQ_WALK_BACKWARD = {
        // 1. 뒤로 자세
        { POSE_BACKWARD_READY, 0.6, 0.2 },
        // 2. 원위치
        { POSE_STAND,          0.6, 0.2 }
    };

} // namespace rhp_motions

#endif
