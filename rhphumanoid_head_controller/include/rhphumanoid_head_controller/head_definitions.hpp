#ifndef HEAD_DEFINITIONS_HPP
#define HEAD_DEFINITIONS_HPP

#include <vector>

namespace rhp_head_motions {

    // Motion step: [pan_rad, tilt_rad], move_time (s), stop_time (s)
    struct MotionStep {
        std::vector<double> positions;  // 2 joints: head_pan, head_tilt
        double move_time;
        double stop_time;
    };

    // -------------------------------------------------------------------------
    // Joint order (2): head_pan, head_tilt
    // Angle convention: 0 rad = neutral/centre
    //   head_pan  positive = left,  negative = right
    //   head_tilt positive = down,  negative = up
    // -------------------------------------------------------------------------

    // Neutral pose (centre)
    const std::vector<double> POSE_CENTER = { 0.0, 0.0 };

    // Scan poses
    const std::vector<double> POSE_PAN_LEFT  = { 0.8, 0.0 };
    const std::vector<double> POSE_PAN_RIGHT = {-0.8, 0.0 };
    const std::vector<double> POSE_TILT_UP   = { 0.0,-0.4 };
    const std::vector<double> POSE_TILT_DOWN = { 0.0, 0.4 };

    // -------------------------------------------------------------------------
    // Motion sequences
    // -------------------------------------------------------------------------

    // Return to centre
    const std::vector<MotionStep> SEQ_CENTER = {
        { POSE_CENTER, 0.8, 0.3 }
    };

    // Horizontal scan: left → centre → right → centre
    const std::vector<MotionStep> SEQ_SCAN_HORIZONTAL = {
        { POSE_PAN_LEFT,  0.6, 0.4 },
        { POSE_CENTER,    0.6, 0.2 },
        { POSE_PAN_RIGHT, 0.6, 0.4 },
        { POSE_CENTER,    0.6, 0.2 }
    };

    // Vertical scan: tilt up → centre → tilt down → centre
    const std::vector<MotionStep> SEQ_SCAN_VERTICAL = {
        { POSE_TILT_UP,   0.5, 0.3 },
        { POSE_CENTER,    0.5, 0.2 },
        { POSE_TILT_DOWN, 0.5, 0.3 },
        { POSE_CENTER,    0.5, 0.2 }
    };

} // namespace rhp_head_motions

#endif  // HEAD_DEFINITIONS_HPP
