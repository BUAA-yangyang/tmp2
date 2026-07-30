#include "FSM/SafeStandGyroFilter.h"

#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace {
constexpr double kThresholdRadS = 0.15;
}

TEST(SafeStandGyroFilterTest, IsolatedLowRtfSpikeDoesNotResetStableSignal) {
    SafeStandGyroFilter filter(0.10, 0.05);
    EXPECT_TRUE(filter.update(1.000, {{0.0, 0.0, 0.0}}).valid);
    const auto spike = filter.update(1.002, {{1.10, 0.0, 0.0}});
    EXPECT_TRUE(spike.valid);
    EXPECT_FALSE(spike.discontinuity);
    EXPECT_LT(spike.norm, kThresholdRadS);
}

TEST(SafeStandGyroFilterTest, SustainedRotationStillExceedsOriginalThreshold) {
    SafeStandGyroFilter filter(0.10, 0.05);
    filter.update(1.000, {{0.0, 0.0, 0.0}});
    SafeStandGyroFilterResult result{false, false, 0.0};
    for(int index = 1; index <= 100; ++index) {
        result = filter.update(
            1.000 + 0.002 * index, {{0.50, 0.0, 0.0}});
    }
    EXPECT_TRUE(result.valid);
    EXPECT_GT(result.norm, kThresholdRadS);
}

TEST(SafeStandGyroFilterTest, ClockRollbackAndLargeGapAreDiscontinuities) {
    SafeStandGyroFilter filter(0.10, 0.05);
    filter.update(2.000, {{0.0, 0.0, 0.0}});
    EXPECT_TRUE(
        filter.update(1.900, {{0.0, 0.0, 0.0}}).discontinuity);
    EXPECT_TRUE(
        filter.update(2.100, {{0.0, 0.0, 0.0}}).discontinuity);
}

TEST(SafeStandGyroFilterTest, NonFiniteSampleFailsClosed) {
    SafeStandGyroFilter filter(0.10, 0.05);
    const auto result = filter.update(
        1.000,
        {{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}});
    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.discontinuity);
    EXPECT_TRUE(std::isinf(result.norm));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
