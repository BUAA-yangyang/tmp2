#pragma once

// Geometry-only door and wall perception.  This header deliberately has no
// ROS/PCL dependency so that the extraction and tracking contracts can be
// regression-tested from deterministic synthetic scans.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace a1_floor_mapping {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

inline Point2 operator+(const Point2& a, const Point2& b) { return {a.x + b.x, a.y + b.y}; }
inline Point2 operator-(const Point2& a, const Point2& b) { return {a.x - b.x, a.y - b.y}; }
inline Point2 operator*(const Point2& a, double scale) { return {a.x * scale, a.y * scale}; }
inline double dot(const Point2& a, const Point2& b) { return a.x * b.x + a.y * b.y; }
inline double cross(const Point2& a, const Point2& b) { return a.x * b.y - a.y * b.x; }
inline double norm(const Point2& value) { return std::hypot(value.x, value.y); }
inline Point2 unit(const Point2& value) {
  const double length = norm(value);
  return length > 1e-9 ? value * (1.0 / length) : Point2{1.0, 0.0};
}
inline Point2 perpendicular(const Point2& value) { return {-value.y, value.x}; }
inline double clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }
inline double undirectedAngle(const Point2& a, const Point2& b) {
  return std::acos(clamp(std::abs(dot(unit(a), unit(b))), -1.0, 1.0));
}

// Height is measured above the currently trusted floor.  Raw geometry is kept
// in odom so all tracked IDs remain meaningful only for one mapping session.
struct HeightPoint {
  Point2 xy;
  double height{0.0};
};

struct WallSegmentGeometry {
  Point2 start;
  Point2 end;
  Point2 direction;
  double length{0.0};
  double residual{0.0};
  double height_support{0.0};
  std::size_t support_points{0};
};

enum class DoorState : uint8_t {
  UNKNOWN = 0,
  OPEN = 1,
  CLOSED = 2,
  PARTIALLY_OPEN = 3,
  BLOCKED = 4,
};

struct DoorWallParameters {
  double extraction_cell_size{0.08};
  double line_fit_error{0.08};
  double max_segment_gap{0.24};
  double minimum_wall_length{0.80};
  double elevator_opening_flank_min_length{0.55};
  double minimum_wall_height{0.60};
  std::size_t minimum_wall_support{12};
  std::size_t elevator_opening_flank_min_support{6};
  std::size_t max_ransac_iterations{360};
  std::size_t max_segments_per_frame{24};

  double wall_track_distance{0.15};
  double wall_track_angle_rad{0.15};
  std::size_t wall_stable_frames{3};
  std::size_t max_wall_misses{6};

  double collinear_distance{0.12};
  double opening_min_width{1.00};
  double opening_max_width{2.20};
  std::size_t opening_stable_frames{3};
  double doorway_match_distance{0.30};
  std::size_t max_doorway_misses{45};

  double state_min_height{0.25};
  double state_band_depth{0.14};
  double free_ray_clearance{0.15};
  std::size_t state_bins{8};
  std::size_t minimum_state_bins{3};
  double open_fraction{0.50};
  double closed_fraction{0.65};
  std::size_t state_stable_frames{3};
  double robot_radius{0.35};
  double traversable_margin{0.15};
  double approach_offset{0.70};
};

struct TrackedWall {
  uint32_t id{0};
  WallSegmentGeometry geometry;
  std::size_t observation_count{0};
  std::size_t misses{0};
  bool stable{false};
};

struct TrackedDoorway {
  uint32_t id{0};
  Point2 center;
  Point2 normal;
  Point2 left_boundary;
  Point2 right_boundary;
  double width{0.0};
  double usable_width{0.0};
  DoorState state{DoorState::UNKNOWN};
  double confidence{0.0};
  std::size_t observation_count{0};
  std::size_t misses{0};
  bool stable{false};
};

struct DoorWallFrame {
  std::vector<TrackedWall> walls;
  std::vector<TrackedDoorway> doorways;
};

class DoorWallRecognizer {
 public:
  explicit DoorWallRecognizer(DoorWallParameters parameters = DoorWallParameters())
      : parameters_(parameters) {}

  void reset() {
    walls_.clear();
    doorways_.clear();
    next_wall_id_ = 1;
    next_doorway_id_ = 1;
  }

  DoorWallFrame update(const std::vector<HeightPoint>& obstacle_points, const Point2& sensor_origin,
                       bool elevator_scan_active = false) {
    updateWalls(extractSegments(obstacle_points, elevator_scan_active));
    const std::vector<DoorGeometry> candidates = findOpenings(obstacle_points, sensor_origin);
    updateDoorways(candidates, obstacle_points, sensor_origin);
    DoorWallFrame frame;
    for (const WallTrack& track : walls_) {
      if (track.observations >= parameters_.wall_stable_frames && track.misses == 0) {
        frame.walls.push_back(TrackedWall{track.id, track.geometry, track.observations, track.misses, true});
      }
    }
    for (const DoorTrack& track : doorways_) {
      if (track.observations >= parameters_.opening_stable_frames) {
        frame.doorways.push_back(makeTrackedDoorway(track));
      }
    }
    return frame;
  }

 private:
  struct ReducedPoint {
    Point2 xy;
    double minimum_height{std::numeric_limits<double>::infinity()};
    double maximum_height{-std::numeric_limits<double>::infinity()};
    std::size_t count{0};
  };

  struct WallTrack {
    uint32_t id{0};
    WallSegmentGeometry geometry;
    std::size_t observations{0};
    std::size_t misses{0};
  };

  struct DoorGeometry {
    Point2 center;
    Point2 tangent;
    Point2 normal;
    Point2 left;
    Point2 right;
    double width{0.0};
  };

  struct DoorEvidence {
    std::size_t free_bins{0};
    std::size_t occupied_bins{0};
    double usable_width{0.0};
    bool observed{false};
  };

  struct DoorTrack {
    uint32_t id{0};
    DoorGeometry geometry;
    std::size_t observations{0};
    std::size_t misses{0};
    DoorState state{DoorState::UNKNOWN};
    DoorState pending_state{DoorState::UNKNOWN};
    std::size_t pending_state_frames{0};
    DoorEvidence evidence;
  };

  std::vector<ReducedPoint> reduce(const std::vector<HeightPoint>& points) const {
    struct Aggregate {
      double x{0.0};
      double y{0.0};
      double minimum_height{std::numeric_limits<double>::infinity()};
      double maximum_height{-std::numeric_limits<double>::infinity()};
      std::size_t count{0};
    };
    std::map<std::pair<int, int>, Aggregate> cells;
    for (const HeightPoint& point : points) {
      if (!std::isfinite(point.xy.x) || !std::isfinite(point.xy.y) || !std::isfinite(point.height)) continue;
      const int x = static_cast<int>(std::floor(point.xy.x / parameters_.extraction_cell_size));
      const int y = static_cast<int>(std::floor(point.xy.y / parameters_.extraction_cell_size));
      Aggregate& aggregate = cells[std::make_pair(x, y)];
      aggregate.x += point.xy.x;
      aggregate.y += point.xy.y;
      aggregate.minimum_height = std::min(aggregate.minimum_height, point.height);
      aggregate.maximum_height = std::max(aggregate.maximum_height, point.height);
      ++aggregate.count;
    }
    std::vector<ReducedPoint> result;
    result.reserve(cells.size());
    for (const auto& entry : cells) {
      const Aggregate& aggregate = entry.second;
      result.push_back(ReducedPoint{{aggregate.x / aggregate.count, aggregate.y / aggregate.count},
                                   aggregate.minimum_height, aggregate.maximum_height, aggregate.count});
    }
    return result;
  }

  static Point2 average(const std::vector<ReducedPoint>& points, const std::vector<std::size_t>& indices) {
    Point2 result;
    for (std::size_t index : indices) result = result + points[index].xy;
    return result * (1.0 / std::max<std::size_t>(1, indices.size()));
  }

  static Point2 principalDirection(const std::vector<ReducedPoint>& points, const std::vector<std::size_t>& indices,
                                   const Point2& mean) {
    double xx = 0.0, xy = 0.0, yy = 0.0;
    for (std::size_t index : indices) {
      const Point2 delta = points[index].xy - mean;
      xx += delta.x * delta.x;
      xy += delta.x * delta.y;
      yy += delta.y * delta.y;
    }
    const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
    return {std::cos(angle), std::sin(angle)};
  }

  WallSegmentGeometry makeSegment(const std::vector<ReducedPoint>& points, const std::vector<std::size_t>& indices,
                                  const Point2& origin, const Point2& direction) const {
    WallSegmentGeometry result;
    if (indices.empty()) return result;
    double minimum_projection = std::numeric_limits<double>::infinity();
    double maximum_projection = -std::numeric_limits<double>::infinity();
    double residual_sum = 0.0;
    double minimum_height = std::numeric_limits<double>::infinity();
    double maximum_height = -std::numeric_limits<double>::infinity();
    for (std::size_t index : indices) {
      const Point2 delta = points[index].xy - origin;
      const double projection = dot(delta, direction);
      minimum_projection = std::min(minimum_projection, projection);
      maximum_projection = std::max(maximum_projection, projection);
      residual_sum += std::abs(cross(delta, direction));
      minimum_height = std::min(minimum_height, points[index].minimum_height);
      maximum_height = std::max(maximum_height, points[index].maximum_height);
    }
    result.start = origin + direction * minimum_projection;
    result.end = origin + direction * maximum_projection;
    result.direction = direction;
    result.length = maximum_projection - minimum_projection;
    result.residual = residual_sum / indices.size();
    result.height_support = maximum_height - minimum_height;
    result.support_points = indices.size();
    return result;
  }

  bool validSegment(const WallSegmentGeometry& segment, bool elevator_scan_active) const {
    const double minimum_length = elevator_scan_active
        ? parameters_.elevator_opening_flank_min_length
        : parameters_.minimum_wall_length;
    const std::size_t minimum_support = elevator_scan_active
        ? parameters_.elevator_opening_flank_min_support
        : parameters_.minimum_wall_support;
    return segment.length >= minimum_length &&
           segment.residual <= parameters_.line_fit_error &&
           segment.height_support >= parameters_.minimum_wall_height &&
           segment.support_points >= minimum_support;
  }

  std::vector<WallSegmentGeometry> extractSegments(const std::vector<HeightPoint>& points,
                                                    bool elevator_scan_active) const {
    const double extraction_minimum_length = elevator_scan_active
        ? parameters_.elevator_opening_flank_min_length
        : parameters_.minimum_wall_length;
    const std::size_t extraction_minimum_support = elevator_scan_active
        ? parameters_.elevator_opening_flank_min_support
        : parameters_.minimum_wall_support;
    const std::vector<ReducedPoint> reduced = reduce(points);
    std::vector<std::size_t> active;
    active.reserve(reduced.size());
    for (std::size_t index = 0; index < reduced.size(); ++index) active.push_back(index);
    std::vector<WallSegmentGeometry> segments;
    uint32_t random_state = 0x6d2b79f5u;
    while (active.size() >= extraction_minimum_support &&
           segments.size() < parameters_.max_segments_per_frame) {
      std::vector<std::size_t> best;
      double best_score = 0.0;
      for (std::size_t iteration = 0; iteration < parameters_.max_ransac_iterations; ++iteration) {
        random_state = random_state * 1664525u + 1013904223u;
        const std::size_t first = active[random_state % active.size()];
        random_state = random_state * 1664525u + 1013904223u;
        const std::size_t second = active[random_state % active.size()];
        if (first == second) continue;
        const Point2 direction = unit(reduced[second].xy - reduced[first].xy);
        if (norm(reduced[second].xy - reduced[first].xy) < extraction_minimum_length * 0.5) continue;
        std::vector<std::size_t> inliers;
        double minimum_projection = std::numeric_limits<double>::infinity();
        double maximum_projection = -std::numeric_limits<double>::infinity();
        for (std::size_t index : active) {
          const Point2 delta = reduced[index].xy - reduced[first].xy;
          if (std::abs(cross(delta, direction)) > parameters_.line_fit_error) continue;
          inliers.push_back(index);
          const double projection = dot(delta, direction);
          minimum_projection = std::min(minimum_projection, projection);
          maximum_projection = std::max(maximum_projection, projection);
        }
        const double span = maximum_projection - minimum_projection;
        const double score = inliers.size() * std::max(0.0, span);
        if (inliers.size() >= extraction_minimum_support &&
            span >= extraction_minimum_length && score > best_score) {
          best = inliers;
          best_score = score;
        }
      }
      if (best.empty()) break;

      const Point2 origin = average(reduced, best);
      const Point2 direction = principalDirection(reduced, best, origin);
      std::vector<std::pair<double, std::size_t>> line_points;
      for (std::size_t index : active) {
        const Point2 delta = reduced[index].xy - origin;
        if (std::abs(cross(delta, direction)) <= parameters_.line_fit_error) {
          line_points.push_back(std::make_pair(dot(delta, direction), index));
        }
      }
      if (line_points.empty()) break;
      std::sort(line_points.begin(), line_points.end());

      std::vector<std::size_t> group;
      std::vector<bool> consumed(reduced.size(), false);
      const auto flush_group = [&]() {
        const WallSegmentGeometry segment = makeSegment(reduced, group, origin, direction);
        if (validSegment(segment, elevator_scan_active) &&
            segments.size() < parameters_.max_segments_per_frame) segments.push_back(segment);
      };
      double previous_projection = line_points.front().first;
      for (const auto& item : line_points) {
        if (!group.empty() && item.first - previous_projection > parameters_.max_segment_gap) {
          flush_group();
          group.clear();
        }
        group.push_back(item.second);
        consumed[item.second] = true;
        previous_projection = item.first;
      }
      flush_group();
      active.erase(std::remove_if(active.begin(), active.end(), [&consumed](std::size_t index) {
        return consumed[index];
      }), active.end());
    }
    return segments;
  }

  static Point2 center(const WallSegmentGeometry& segment) { return (segment.start + segment.end) * 0.5; }

  bool matchesWall(const WallTrack& track, const WallSegmentGeometry& candidate) const {
    if (undirectedAngle(track.geometry.direction, candidate.direction) > parameters_.wall_track_angle_rad) return false;
    const Point2 direction = unit(track.geometry.direction);
    if (std::abs(cross(center(candidate) - center(track.geometry), direction)) > parameters_.wall_track_distance) return false;
    const double a0 = std::min(dot(track.geometry.start, direction), dot(track.geometry.end, direction));
    const double a1 = std::max(dot(track.geometry.start, direction), dot(track.geometry.end, direction));
    const double b0 = std::min(dot(candidate.start, direction), dot(candidate.end, direction));
    const double b1 = std::max(dot(candidate.start, direction), dot(candidate.end, direction));
    const double interval_gap = std::max(0.0, std::max(a0, b0) - std::min(a1, b1));
    return interval_gap <= parameters_.wall_track_distance;
  }

  void updateWalls(const std::vector<WallSegmentGeometry>& observations) {
    for (WallTrack& track : walls_) ++track.misses;
    std::vector<bool> used(walls_.size(), false);
    for (const WallSegmentGeometry& observation : observations) {
      std::size_t best = walls_.size();
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < walls_.size(); ++index) {
        if (used[index] || !matchesWall(walls_[index], observation)) continue;
        const double distance = norm(center(walls_[index].geometry) - center(observation));
        if (distance < best_distance) { best = index; best_distance = distance; }
      }
      if (best == walls_.size()) {
        walls_.push_back(WallTrack{next_wall_id_++, observation, 1, 0});
        used.push_back(true);
      } else {
        WallTrack& track = walls_[best];
        track.geometry = observation;
        ++track.observations;
        track.misses = 0;
        used[best] = true;
      }
    }
    walls_.erase(std::remove_if(walls_.begin(), walls_.end(), [this](const WallTrack& track) {
      return track.misses > parameters_.max_wall_misses;
    }), walls_.end());
  }

  static Point2 endpointAt(const WallSegmentGeometry& segment, const Point2& direction, bool maximum) {
    const double first = dot(segment.start, direction);
    const double second = dot(segment.end, direction);
    return (first > second) == maximum ? segment.start : segment.end;
  }

  DoorEvidence evaluate(const DoorGeometry& geometry, const std::vector<HeightPoint>& points,
                        const Point2& sensor_origin) const {
    const std::size_t bins = std::max<std::size_t>(1, parameters_.state_bins);
    std::vector<bool> free(bins, false);
    std::vector<bool> occupied(bins, false);
    const auto binFor = [&geometry, bins](double projection) {
      return std::min<std::size_t>(bins - 1, static_cast<std::size_t>(
          clamp((projection + geometry.width * 0.5) / geometry.width * bins, 0.0, static_cast<double>(bins - 1))));
    };
    for (const HeightPoint& point : points) {
      if (point.height < parameters_.state_min_height) continue;
      const Point2 relative = point.xy - geometry.center;
      const double projection = dot(relative, geometry.tangent);
      if (std::abs(projection) >= geometry.width * 0.5) continue;
      if (std::abs(dot(relative, geometry.normal)) <= parameters_.state_band_depth) occupied[binFor(projection)] = true;

      const Point2 ray = point.xy - sensor_origin;
      const double denominator = cross(ray, geometry.tangent);
      if (std::abs(denominator) < 1e-8) continue;
      const double ratio = cross(geometry.center - sensor_origin, geometry.tangent) / denominator;
      if (ratio <= 0.02 || ratio >= 0.999) continue;
      const Point2 intersection = sensor_origin + ray * ratio;
      const double intersection_projection = dot(intersection - geometry.center, geometry.tangent);
      if (std::abs(intersection_projection) >= geometry.width * 0.5) continue;
      if (norm(ray) * (1.0 - ratio) >= parameters_.free_ray_clearance) free[binFor(intersection_projection)] = true;
    }
    DoorEvidence result;
    for (std::size_t index = 0; index < bins; ++index) {
      if (free[index]) ++result.free_bins;
      if (occupied[index]) ++result.occupied_bins;
      if (free[index] && !occupied[index]) result.usable_width += geometry.width / bins;
    }
    result.observed = std::max(result.free_bins, result.occupied_bins) >= parameters_.minimum_state_bins;
    return result;
  }

  std::vector<DoorGeometry> findOpenings(const std::vector<HeightPoint>& points, const Point2& sensor_origin) const {
    std::vector<DoorGeometry> result;
    for (std::size_t first = 0; first < walls_.size(); ++first) {
      const WallTrack& a = walls_[first];
      if (a.observations < parameters_.wall_stable_frames || a.misses != 0) continue;
      for (std::size_t second = first + 1; second < walls_.size(); ++second) {
        const WallTrack& b = walls_[second];
        if (b.observations < parameters_.wall_stable_frames || b.misses != 0 ||
            undirectedAngle(a.geometry.direction, b.geometry.direction) > parameters_.wall_track_angle_rad) continue;
        const Point2 tangent = unit(a.geometry.direction);
        if (std::abs(cross(center(b.geometry) - center(a.geometry), tangent)) > parameters_.collinear_distance) continue;
        const double a0 = std::min(dot(a.geometry.start, tangent), dot(a.geometry.end, tangent));
        const double a1 = std::max(dot(a.geometry.start, tangent), dot(a.geometry.end, tangent));
        const double b0 = std::min(dot(b.geometry.start, tangent), dot(b.geometry.end, tangent));
        const double b1 = std::max(dot(b.geometry.start, tangent), dot(b.geometry.end, tangent));
        const WallTrack* left = &a;
        const WallTrack* right = &b;
        double left_max = a1;
        double right_min = b0;
        if (b1 <= a0) { left = &b; right = &a; left_max = b1; right_min = a0; }
        const double width = right_min - left_max;
        if (width < parameters_.opening_min_width || width > parameters_.opening_max_width) continue;
        const Point2 left_boundary = endpointAt(left->geometry, tangent, true);
        const Point2 right_boundary = endpointAt(right->geometry, tangent, false);
        DoorGeometry geometry;
        geometry.left = left_boundary;
        geometry.right = right_boundary;
        geometry.center = (left_boundary + right_boundary) * 0.5;
        geometry.tangent = tangent;
        geometry.normal = perpendicular(tangent);
        geometry.width = width;
        const DoorEvidence evidence = evaluate(geometry, points, sensor_origin);
        // A gap in sparse returns is not an opening.  Require rays that really
        // travel through it before starting a semantic track.
        if (evidence.free_bins < parameters_.minimum_state_bins) continue;
        bool duplicate = false;
        for (const DoorGeometry& existing : result) {
          if (norm(existing.center - geometry.center) < parameters_.doorway_match_distance &&
              undirectedAngle(existing.tangent, geometry.tangent) < parameters_.wall_track_angle_rad) {
            duplicate = true;
            break;
          }
        }
        if (!duplicate) result.push_back(geometry);
      }
    }
    return result;
  }

  bool matchesDoorway(const DoorTrack& track, const DoorGeometry& candidate) const {
    return norm(track.geometry.center - candidate.center) <= parameters_.doorway_match_distance &&
           undirectedAngle(track.geometry.tangent, candidate.tangent) <= parameters_.wall_track_angle_rad &&
           std::abs(track.geometry.width - candidate.width) <= parameters_.doorway_match_distance;
  }

  DoorState classify(const DoorEvidence& evidence) const {
    if (!evidence.observed) return DoorState::UNKNOWN;
    const double free_fraction = static_cast<double>(evidence.free_bins) / parameters_.state_bins;
    const double occupied_fraction = static_cast<double>(evidence.occupied_bins) / parameters_.state_bins;
    if (occupied_fraction >= parameters_.closed_fraction && free_fraction < parameters_.open_fraction) return DoorState::CLOSED;
    if (free_fraction >= parameters_.open_fraction && occupied_fraction <= 0.20) return DoorState::OPEN;
    if (free_fraction >= static_cast<double>(parameters_.minimum_state_bins) / parameters_.state_bins &&
        occupied_fraction >= static_cast<double>(parameters_.minimum_state_bins) / parameters_.state_bins) return DoorState::PARTIALLY_OPEN;
    if (occupied_fraction >= static_cast<double>(parameters_.minimum_state_bins) / parameters_.state_bins) return DoorState::BLOCKED;
    return DoorState::UNKNOWN;
  }

  void updateState(DoorTrack& track, const DoorEvidence& evidence) const {
    track.evidence = evidence;
    const DoorState observation = classify(evidence);
    if (observation == track.pending_state) ++track.pending_state_frames;
    else { track.pending_state = observation; track.pending_state_frames = 1; }
    if (track.pending_state_frames >= parameters_.state_stable_frames) track.state = observation;
    if (evidence.observed) track.misses = 0;
  }

  void updateDoorways(const std::vector<DoorGeometry>& observations, const std::vector<HeightPoint>& points,
                      const Point2& sensor_origin) {
    for (DoorTrack& track : doorways_) ++track.misses;
    std::vector<bool> used(doorways_.size(), false);
    for (const DoorGeometry& observation : observations) {
      std::size_t best = doorways_.size();
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < doorways_.size(); ++index) {
        if (used[index] || !matchesDoorway(doorways_[index], observation)) continue;
        const double distance = norm(doorways_[index].geometry.center - observation.center);
        if (distance < best_distance) { best = index; best_distance = distance; }
      }
      if (best == doorways_.size()) {
        DoorTrack track;
        track.id = next_doorway_id_++;
        track.geometry = observation;
        track.observations = 1;
        track.misses = 0;
        doorways_.push_back(track);
        used.push_back(true);
      } else {
        DoorTrack& track = doorways_[best];
        track.geometry = observation;
        ++track.observations;
        track.misses = 0;
        used[best] = true;
      }
    }
    for (DoorTrack& track : doorways_) updateState(track, evaluate(track.geometry, points, sensor_origin));
    doorways_.erase(std::remove_if(doorways_.begin(), doorways_.end(), [this](const DoorTrack& track) {
      return track.misses > parameters_.max_doorway_misses;
    }), doorways_.end());
  }

  TrackedDoorway makeTrackedDoorway(const DoorTrack& track) const {
    const bool state_stable = track.pending_state_frames >= parameters_.state_stable_frames;
    const double geometry_confidence = std::min(1.0, static_cast<double>(track.observations) /
                                                parameters_.opening_stable_frames);
    const double evidence_confidence = std::min(1.0, static_cast<double>(
        std::max(track.evidence.free_bins, track.evidence.occupied_bins)) / parameters_.state_bins);
    return TrackedDoorway{track.id, track.geometry.center, track.geometry.normal, track.geometry.left,
                          track.geometry.right, track.geometry.width, track.evidence.usable_width, track.state,
                          0.65 * geometry_confidence + 0.35 * evidence_confidence, track.observations,
                          track.misses, state_stable};
  }

  DoorWallParameters parameters_;
  std::vector<WallTrack> walls_;
  std::vector<DoorTrack> doorways_;
  uint32_t next_wall_id_{1};
  uint32_t next_doorway_id_{1};
};

}  // namespace a1_floor_mapping
