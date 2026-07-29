/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef NAVIGATION_GAIT_PROFILE_H
#define NAVIGATION_GAIT_PROFILE_H

#include <cmath>

namespace navigation_gait {

// Keep the upstream flat-ground profile unchanged outside State_move_base.
constexpr double kDefaultSwingHeightM = 0.08;
constexpr double kDefaultGaitPeriodS = 0.45;
constexpr double kTrotStancePhaseRatio = 0.5;

// Livox-derived entry profiles showed a 0.06--0.07 m support-height rise.
// At the observed late-swing collision phase, the cycloid retains at least
// 75 percent of its configured height.  Mode 5 therefore needs a separate
// profile with explicit clearance margin; no building/world truth is used.
constexpr double kMoveBaseSwingHeightM = 0.12;
// Preserve the higher clearance without increasing touchdown speed.  The
// original 0.45 s trot gives only 0.225 s of swing time; doubling the mode-5
// period keeps the peak vertical target speed below the unchanged flat-ground
// profile while retaining the measured entry-step clearance.
constexpr double kMoveBaseGaitPeriodS = 0.90;
constexpr double kLateSwingClearanceFactor = 0.75;
constexpr double kSupportedSurfaceRiseM = 0.07;
constexpr double kRequiredClearanceMarginM = 0.015;

inline bool isValidSwingHeight(double heightM) {
    return std::isfinite(heightM) && heightM > 0.0;
}

inline bool isValidGaitPeriod(double periodS) {
    return std::isfinite(periodS) && periodS > 0.0;
}

constexpr double maximumSwingVerticalSpeed(
        double heightM, double periodS, double stancePhaseRatio) {
    return heightM * 3.14159265358979323846
        / (periodS * (1.0 - stancePhaseRatio));
}

constexpr double lateSwingClearance(double heightM) {
    return heightM * kLateSwingClearanceFactor;
}

constexpr bool hasRequiredClearance(double heightM) {
    return lateSwingClearance(heightM)
        >= kSupportedSurfaceRiseM + kRequiredClearanceMarginM;
}

static_assert(
    kMoveBaseSwingHeightM > kDefaultSwingHeightM,
    "move_base must retain more swing clearance than flat-ground trotting");
static_assert(
    hasRequiredClearance(kMoveBaseSwingHeightM),
    "move_base swing profile does not retain the required late-swing margin");
static_assert(
    maximumSwingVerticalSpeed(
        kMoveBaseSwingHeightM,
        kMoveBaseGaitPeriodS,
        kTrotStancePhaseRatio)
        <= maximumSwingVerticalSpeed(
            kDefaultSwingHeightM,
            kDefaultGaitPeriodS,
            kTrotStancePhaseRatio),
    "move_base clearance must not increase peak vertical touchdown speed");

}  // namespace navigation_gait

#endif  // NAVIGATION_GAIT_PROFILE_H
