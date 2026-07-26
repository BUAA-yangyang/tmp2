#include <gtest/gtest.h>

#include "a1_floor_mapping/core.h"

using a1_floor_mapping::GroundEstimator;
using a1_floor_mapping::GroundSample;
using a1_floor_mapping::OccupancyIntegrator;

TEST(GroundEstimator, RequiresStableInitializationAndPersistentFloorChange) {
  GroundEstimator estimator(3, 4, 0.08, 0.35);
  for (double z : {0.01, 0.0, -0.01}) estimator.update(GroundSample{true, z, 0.01, 100, 90});
  ASSERT_TRUE(estimator.ready());
  EXPECT_NEAR(estimator.floorZ(), 0.0, 0.02);
  estimator.update(GroundSample{true, 0.5, 0.01, 100, 90});
  EXPECT_FALSE(estimator.floorChangeDetected());
  estimator.update(GroundSample{true, 0.01, 0.01, 100, 90});
  EXPECT_EQ(estimator.changeCount(), 0);
  for (int i = 0; i < 4; ++i) estimator.update(GroundSample{true, 0.5, 0.01, 100, 90});
  EXPECT_TRUE(estimator.floorChangeDetected());
  estimator.reset();
  EXPECT_FALSE(estimator.ready());
}

TEST(OccupancyIntegrator, IntegratesAndClipsRays) {
  OccupancyIntegrator grid(0.1, 4.0, 4.0, 0.35, 0.80, 1);
  EXPECT_TRUE(grid.raytrace(-3.0, 0.0, 3.0, 0.0, false));
  EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.0, 0.0, true));
  std::size_t occupied, free, unknown;
  const auto data = grid.data(occupied, free, unknown);
  EXPECT_EQ(data.size(), 1600u);
  EXPECT_GT(free, 0u);
  EXPECT_GT(occupied, 0u);
  EXPECT_EQ(occupied + free + unknown, data.size());
  EXPECT_NEAR(grid.boundaryMargin(0.0, 0.0), 2.0, 1e-6);
  grid.reset();
  grid.data(occupied, free, unknown);
  EXPECT_EQ(occupied, 0u);
  EXPECT_EQ(free, 0u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
