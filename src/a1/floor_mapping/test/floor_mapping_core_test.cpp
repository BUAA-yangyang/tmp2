#include <gtest/gtest.h>

#include "a1_floor_mapping/core.h"
#include "a1_floor_mapping/door_wall.h"

using a1_floor_mapping::GroundEstimator;
using a1_floor_mapping::GroundSample;
using a1_floor_mapping::OccupancyIntegrator;
using a1_floor_mapping::selectGroundSample;

namespace {
std::vector<a1_floor_mapping::HeightPoint> wallPoints(double x, double y_min, double y_max) {
  std::vector<a1_floor_mapping::HeightPoint> result;
  for (double y = y_min; y <= y_max + 1e-6; y += 0.10) {
    for (double height : {0.10, 0.42, 0.78, 1.16}) result.push_back({{x, y}, height});
  }
  return result;
}

std::vector<a1_floor_mapping::HeightPoint> openDoorwayScan(bool include_background) {
  std::vector<a1_floor_mapping::HeightPoint> result = wallPoints(3.0, -2.0, -0.65);
  const auto right = wallPoints(3.0, 0.65, 2.0);
  result.insert(result.end(), right.begin(), right.end());
  if (include_background) {
    const auto background = wallPoints(5.0, -0.90, 0.90);
    result.insert(result.end(), background.begin(), background.end());
  }
  return result;
}

std::vector<a1_floor_mapping::HeightPoint> closedDoorwayScan() {
  std::vector<a1_floor_mapping::HeightPoint> result = openDoorwayScan(false);
  const auto panel = wallPoints(3.0, -0.55, 0.55);
  result.insert(result.end(), panel.begin(), panel.end());
  return result;
}

int cellValue(OccupancyIntegrator& grid, double x, double y) {
  int cell_x = 0, cell_y = 0;
  if (!grid.worldToCell(x, y, cell_x, cell_y)) return -128;
  std::size_t occupied = 0, free = 0, unknown = 0;
  const auto data = grid.data(occupied, free, unknown);
  return data[static_cast<std::size_t>(cell_y) * grid.width() + cell_x];
}
}  // namespace

TEST(GroundSelection, SupportsIndoorColdStartAndAnchoredTracking) {
  std::vector<double> doorway(100, 0.0);
  doorway.insert(doorway.end(), 400, 0.45);
  EXPECT_TRUE(selectGroundSample(doorway, false, 0.0, 80, 0.20, 0.08).valid);
  EXPECT_FALSE(selectGroundSample(doorway, false, 0.0, 80, 0.35, 0.08).valid);

  std::vector<double> wall_dominated(80, 0.0);
  wall_dominated.insert(wall_dominated.end(), 720, 0.45);
  const auto tracked = selectGroundSample(wall_dominated, true, 0.01, 80, 0.20, 0.08);
  EXPECT_TRUE(tracked.valid);
  EXPECT_EQ(tracked.inliers, 80u);
  EXPECT_NEAR(tracked.z, 0.0, 1e-6);
}

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
  grid.beginFrame();
  EXPECT_TRUE(grid.raytrace(-3.0, 0.0, 3.0, 0.0, false));
  EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.0, 0.0, true));
  grid.endFrame();
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

TEST(OccupancyIntegrator, SameFrameObstacleEndpointDominatesClearingRays) {
  OccupancyIntegrator grid(0.1, 4.0, 4.0, 0.35, 0.80, 1);
  grid.beginFrame();
  for (int ray = 0; ray < 20; ++ray)
    EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.8, 0.0, true));
  EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, true));
  grid.endFrame();
  EXPECT_EQ(cellValue(grid, 1.05, 0.0), 100);
}

TEST(OccupancyIntegrator, ConfirmedObstacleIgnoresRayTraversalUntilGroundEndpointsClearIt) {
  OccupancyIntegrator grid(0.1, 4.0, 4.0, 0.35, 0.80, 2);
  for (int frame = 0; frame < 2; ++frame) {
    grid.beginFrame();
    EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, true));
    grid.endFrame();
  }
  ASSERT_EQ(cellValue(grid, 1.05, 0.0), 100);

  for (int frame = 0; frame < 20; ++frame) {
    grid.beginFrame();
    EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.8, 0.0, true));
    grid.endFrame();
  }
  EXPECT_EQ(cellValue(grid, 1.05, 0.0), 100)
      << "a 3-D ray crossing the 2-D cell is not ground evidence";

  for (int confirmation = 0; confirmation < 2; ++confirmation) {
    grid.beginFrame();
    EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, false));
    grid.endFrame();
    EXPECT_EQ(cellValue(grid, 1.05, 0.0), 100);
  }
  grid.beginFrame();
  EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, false));
  grid.endFrame();
  EXPECT_EQ(cellValue(grid, 1.05, 0.0), 0);
}

TEST(OccupancyIntegrator, ObstacleEndpointResetsPendingGroundConfirmation) {
  OccupancyIntegrator grid(0.1, 4.0, 4.0, 0.35, 0.80, 1);
  grid.beginFrame();
  EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, true));
  grid.endFrame();
  ASSERT_EQ(cellValue(grid, 1.05, 0.0), 100);

  for (int confirmation = 0; confirmation < 2; ++confirmation) {
    grid.beginFrame();
    EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, false));
    grid.endFrame();
  }
  grid.beginFrame();
  EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, true));
  grid.endFrame();
  for (int confirmation = 0; confirmation < 2; ++confirmation) {
    grid.beginFrame();
    EXPECT_TRUE(grid.raytrace(0.0, 0.0, 1.05, 0.0, false));
    grid.endFrame();
  }
  EXPECT_EQ(cellValue(grid, 1.05, 0.0), 100);
}

TEST(OccupancyIntegrator, RequiresExplicitFrameBoundaries) {
  OccupancyIntegrator grid(0.1, 4.0, 4.0, 0.35, 0.80, 1);
  EXPECT_THROW(grid.raytrace(0.0, 0.0, 1.0, 0.0, true), std::logic_error);
  EXPECT_THROW(grid.endFrame(), std::logic_error);
}

TEST(DoorWallRecognizer, RequiresFreeRaysTracksOpeningAndObservesClose) {
  a1_floor_mapping::DoorWallRecognizer recognizer;
  a1_floor_mapping::DoorWallFrame frame;
  for (int iteration = 0; iteration < 7; ++iteration) frame = recognizer.update(openDoorwayScan(true), {0.0, 0.0});
  ASSERT_FALSE(frame.walls.empty());
  ASSERT_EQ(frame.doorways.size(), 1u);
  const uint32_t doorway_id = frame.doorways.front().id;
  EXPECT_EQ(frame.doorways.front().state, a1_floor_mapping::DoorState::OPEN);
  EXPECT_TRUE(frame.doorways.front().stable);
  EXPECT_GT(frame.doorways.front().usable_width, 0.70);

  for (int iteration = 0; iteration < 4; ++iteration) frame = recognizer.update(closedDoorwayScan(), {0.0, 0.0});
  ASSERT_EQ(frame.doorways.size(), 1u);
  EXPECT_EQ(frame.doorways.front().id, doorway_id);
  EXPECT_EQ(frame.doorways.front().state, a1_floor_mapping::DoorState::CLOSED);
  EXPECT_NEAR(frame.doorways.front().usable_width, 0.0, 1e-6);
}

TEST(DoorWallRecognizer, DoesNotTurnAnUnobservedGapIntoDoorway) {
  a1_floor_mapping::DoorWallRecognizer recognizer;
  a1_floor_mapping::DoorWallFrame frame;
  for (int iteration = 0; iteration < 8; ++iteration) frame = recognizer.update(openDoorwayScan(false), {0.0, 0.0});
  EXPECT_FALSE(frame.walls.empty());
  EXPECT_TRUE(frame.doorways.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
