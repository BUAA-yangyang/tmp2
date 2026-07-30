/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef NAVIGATION_EARLY_CONTACT_H
#define NAVIGATION_EARLY_CONTACT_H

#include <cmath>

namespace navigation_early_contact {

// Contact before the apex can be a vertical obstacle.  Treat only a fresh,
// measured contact on the descending half of the swing as an early touchdown.
constexpr double kMinimumSwingPhase = 0.55;
constexpr double kFootForceThresholdN = 5.0;
constexpr double kFootForceFreshnessS = 0.20;

struct State {
    bool unloadObserved;
    bool touchdownLatched;
};

inline bool hasFreshForceEvidence(double forceN, double sampleAgeS) {
    return std::isfinite(forceN)
        && std::isfinite(sampleAgeS)
        && forceN >= 0.0
        && sampleAgeS >= 0.0
        && sampleAgeS <= kFootForceFreshnessS;
}

inline bool shouldLatchTouchdown(
        double phase, double forceN, double sampleAgeS) {
    return std::isfinite(phase)
        && phase >= kMinimumSwingPhase
        && phase <= 1.0
        && hasFreshForceEvidence(forceN, sampleAgeS)
        && forceN >= kFootForceThresholdN
        ;
}

inline State updateState(
        bool plannedContact,
        State current,
        double phase,
        double forceN,
        double sampleAgeS) {
    if(plannedContact) {
        return {false, false};
    }
    if(current.touchdownLatched) {
        return current;
    }
    if(!std::isfinite(phase)
            || phase < 0.0
            || phase > 1.0
            || !hasFreshForceEvidence(forceN, sampleAgeS)) {
        return current;
    }
    if(forceN < kFootForceThresholdN) {
        current.unloadObserved = true;
        return current;
    }
    if(current.unloadObserved
            && shouldLatchTouchdown(phase, forceN, sampleAgeS)) {
        current.touchdownLatched = true;
    }
    return current;
}

}  // namespace navigation_early_contact

#endif  // NAVIGATION_EARLY_CONTACT_H
