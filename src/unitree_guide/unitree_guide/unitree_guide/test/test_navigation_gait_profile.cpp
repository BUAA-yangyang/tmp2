#include "FSM/NavigationEarlyContact.h"
#include "FSM/NavigationGaitProfile.h"
#include "Gait/WaveGenerator.h"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

TEST(NavigationGaitProfile, RejectsInvalidSwingHeights) {
    EXPECT_FALSE(navigation_gait::isValidSwingHeight(0.0));
    EXPECT_FALSE(navigation_gait::isValidSwingHeight(-0.01));
    EXPECT_FALSE(navigation_gait::isValidSwingHeight(
        std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(navigation_gait::isValidSwingHeight(
        std::numeric_limits<double>::quiet_NaN()));
    EXPECT_TRUE(navigation_gait::isValidSwingHeight(
        navigation_gait::kMoveBaseSwingHeightM));
}

TEST(NavigationGaitProfile, KeepsOrdinaryTrottingProfileUnchanged) {
    EXPECT_DOUBLE_EQ(0.08, navigation_gait::kDefaultSwingHeightM);
    EXPECT_DOUBLE_EQ(0.45, navigation_gait::kDefaultGaitPeriodS);
    EXPECT_FALSE(navigation_gait::hasRequiredClearance(
        navigation_gait::kDefaultSwingHeightM));
}

TEST(NavigationGaitProfile, BoundsPeakSwingSpeedForMoveBase) {
    EXPECT_DOUBLE_EQ(0.60, navigation_gait::kMoveBaseGaitPeriodS);
    EXPECT_LE(
        navigation_gait::maximumSwingVerticalSpeed(
            navigation_gait::kMoveBaseSwingHeightM,
            navigation_gait::kMoveBaseGaitPeriodS,
            navigation_gait::kTrotStancePhaseRatio),
        1.20 * navigation_gait::maximumSwingVerticalSpeed(
            navigation_gait::kDefaultSwingHeightM,
            navigation_gait::kDefaultGaitPeriodS,
            navigation_gait::kTrotStancePhaseRatio));
}

TEST(NavigationGaitProfile, WaveGeneratorAppliesAndValidatesPeriod) {
    WaveGenerator wave(
        navigation_gait::kDefaultGaitPeriodS,
        navigation_gait::kTrotStancePhaseRatio,
        Vec4(0.0, 0.5, 0.5, 0.0),
        0.004);
    wave.setPeriod(navigation_gait::kMoveBaseGaitPeriodS);
    EXPECT_NEAR(0.60, wave.getT(), 1e-6);
    EXPECT_NEAR(0.30, wave.getTswing(), 1e-6);
    EXPECT_THROW(wave.setPeriod(0.0), std::invalid_argument);
    EXPECT_THROW(
        wave.setPeriod(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}

TEST(NavigationGaitProfile, RetainsLateSwingEntryClearance) {
    EXPECT_DOUBLE_EQ(0.12, navigation_gait::kMoveBaseSwingHeightM);
    EXPECT_GE(
        navigation_gait::lateSwingClearance(
            navigation_gait::kMoveBaseSwingHeightM),
        navigation_gait::kSupportedSurfaceRiseM
            + navigation_gait::kRequiredClearanceMarginM);
}

TEST(NavigationEarlyContact, AcceptsFreshDescendingSwingTouchdown) {
    EXPECT_TRUE(navigation_early_contact::shouldLatchTouchdown(
        0.78, 143.7, 0.004));
    EXPECT_TRUE(navigation_early_contact::shouldLatchTouchdown(
        navigation_early_contact::kMinimumSwingPhase,
        navigation_early_contact::kFootForceThresholdN,
        navigation_early_contact::kFootForceFreshnessS));
}

TEST(NavigationEarlyContact, RejectsEarlySwingAndWeakContact) {
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.40, 143.7, 0.004));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.78, 4.99, 0.004));
}

TEST(NavigationEarlyContact, RejectsStaleRollbackAndNonFiniteEvidence) {
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.78, 143.7, 0.201));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.78, 143.7, -0.001));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        std::numeric_limits<double>::quiet_NaN(), 143.7, 0.004));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.78, std::numeric_limits<double>::quiet_NaN(), 0.004));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.78, 143.7, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        std::numeric_limits<double>::infinity(), 143.7, 0.004));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.78, std::numeric_limits<double>::infinity(), 0.004));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        1.0001, 143.7, 0.004));
    EXPECT_FALSE(navigation_early_contact::shouldLatchTouchdown(
        0.78, -1.0, 0.004));
}

TEST(NavigationEarlyContact, RequiresUnloadBeforeTouchdownLatch) {
    navigation_early_contact::State state{false, false};
    state = navigation_early_contact::updateState(
        false, state, 0.10, 120.0, 0.004);
    EXPECT_FALSE(state.unloadObserved);
    EXPECT_FALSE(state.touchdownLatched);

    state = navigation_early_contact::updateState(
        false, state, 0.30, 0.0, 0.004);
    EXPECT_TRUE(state.unloadObserved);
    EXPECT_FALSE(state.touchdownLatched);

    state = navigation_early_contact::updateState(
        false, state, 0.78, 143.7, 0.004);
    EXPECT_TRUE(state.unloadObserved);
    EXPECT_TRUE(state.touchdownLatched);
}

TEST(NavigationEarlyContact, HoldsLatchUntilPlannedContact) {
    navigation_early_contact::State state{true, true};
    state = navigation_early_contact::updateState(
        false,
        state,
        0.90,
        0.0,
        std::numeric_limits<double>::infinity());
    EXPECT_TRUE(state.unloadObserved);
    EXPECT_TRUE(state.touchdownLatched);

    state = navigation_early_contact::updateState(
        true, state, 0.0, 143.7, 0.004);
    EXPECT_FALSE(state.unloadObserved);
    EXPECT_FALSE(state.touchdownLatched);
}

TEST(NavigationEarlyContact, RejectsInvalidEvidenceWithoutLosingOtherLegState) {
    navigation_early_contact::State validLeg{true, false};
    navigation_early_contact::State invalidLeg{false, false};
    validLeg = navigation_early_contact::updateState(
        false, validLeg, 0.78, 143.7, 0.004);
    invalidLeg = navigation_early_contact::updateState(
        false,
        invalidLeg,
        0.78,
        143.7,
        -0.001);
    EXPECT_TRUE(validLeg.touchdownLatched);
    EXPECT_FALSE(invalidLeg.unloadObserved);
    EXPECT_FALSE(invalidLeg.touchdownLatched);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
