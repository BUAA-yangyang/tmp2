#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "unitree_joint_control_tool.h"

namespace {

constexpr double kCalfLower = -2.69653369433;
constexpr double kCalfUpper = -0.916297857297;
constexpr double kTwoPi = 6.28318530717958647692;

TEST(RevolutePositionCanonicalization, PreservesValuesInTheUrdfRepresentation)
{
    EXPECT_DOUBLE_EQ(
        -1.8,
        canonicalizeRevolutePosition(-1.8, kCalfLower, kCalfUpper));
    EXPECT_DOUBLE_EQ(
        3.5,
        canonicalizeRevolutePosition(3.5, -1.0471975512, 4.18879020479));
}

TEST(RevolutePositionCanonicalization, MapsGazeboCalfFeedbackNearLowerLimit)
{
    const double rawGazeboPosition = 3.586629;
    const double canonical = canonicalizeRevolutePosition(
        rawGazeboPosition, kCalfLower, kCalfUpper);

    EXPECT_NEAR(rawGazeboPosition - kTwoPi, canonical, 1e-12);
    EXPECT_GT(-1.8 - canonical, 0.0);
    EXPECT_LT(-1.8 - canonical, 1.0);
}

TEST(RevolutePositionCanonicalization, IsInvariantToWholeTurns)
{
    const double expected = -1.8;
    EXPECT_NEAR(
        expected,
        canonicalizeRevolutePosition(
            expected + 3.0 * kTwoPi, kCalfLower, kCalfUpper),
        1e-12);
    EXPECT_NEAR(
        expected,
        canonicalizeRevolutePosition(
            expected - 2.0 * kTwoPi, kCalfLower, kCalfUpper),
        1e-12);
}

TEST(RevolutePositionCanonicalization, DoesNotHideInvalidInputs)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::isnan(
        canonicalizeRevolutePosition(nan, kCalfLower, kCalfUpper)));
    EXPECT_DOUBLE_EQ(
        1.25,
        canonicalizeRevolutePosition(1.25, kCalfUpper, kCalfLower));
}

TEST(RevolutePositionCanonicalization, CorrectsTheFixedStandTorqueDirection)
{
    ServoCmd command{};
    command.pos = -1.8;
    command.posStiffness = 140.0;
    command.vel = 0.0;
    command.velStiffness = 7.0;
    command.torque = 0.0;

    const double rawGazeboPosition = 3.586629;
    const double rawTorque = computeTorque(rawGazeboPosition, 0.0, command);
    const double canonicalTorque = computeTorque(
        canonicalizeRevolutePosition(
            rawGazeboPosition, kCalfLower, kCalfUpper),
        0.0,
        command);

    EXPECT_LT(rawTorque, 0.0);
    EXPECT_GT(canonicalTorque, 0.0);
}

}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
