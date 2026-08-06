#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace a1_floor_mapping {

struct GroundSample {
  bool valid{false};
  double z{0.0};
  double dispersion{0.0};
  std::size_t candidates{0};
  std::size_t inliers{0};
};

inline GroundSample selectGroundSample(std::vector<double> candidates, bool floor_ready,
                                       double floor_z, std::size_t minimum_candidates,
                                       double minimum_inlier_ratio, double maximum_dispersion) {
  GroundSample empty; empty.candidates = candidates.size();
  if (candidates.size() < minimum_candidates) return empty;
  const auto fit_band = [&](double center, bool require_ratio) {
    GroundSample sample; sample.candidates = candidates.size(); std::vector<double> band; band.reserve(candidates.size());
    for (double z : candidates) if (std::abs(z - center) <= 0.06) band.push_back(z);
    sample.inliers = band.size();
    if (band.empty()) return sample;
    auto middle = band.begin() + band.size() / 2; std::nth_element(band.begin(), middle, band.end()); sample.z = *middle;
    if (band.size() < minimum_candidates ||
        (require_ratio && static_cast<double>(band.size()) / candidates.size() < minimum_inlier_ratio)) return sample;
    std::vector<double> deviations; deviations.reserve(band.size());
    for (double z : band) deviations.push_back(std::abs(z - sample.z));
    auto deviation_middle = deviations.begin() + deviations.size() / 2;
    std::nth_element(deviations.begin(), deviation_middle, deviations.end()); sample.dispersion = *deviation_middle;
    sample.valid = sample.dispersion <= maximum_dispersion; return sample;
  };
  if (floor_ready) { const auto tracked = fit_band(floor_z, false); if (tracked.valid) return tracked; }
  const std::size_t lower_decile = candidates.size() / 10;
  std::nth_element(candidates.begin(), candidates.begin() + lower_decile, candidates.end());
  return fit_band(candidates[lower_decile], true);
}

class GroundEstimator {
 public:
  GroundEstimator(int initialization_frames, int floor_change_frames,
                  double maximum_frame_delta, double unsupported_floor_delta)
      : initialization_frames_(initialization_frames), floor_change_frames_(floor_change_frames),
        maximum_frame_delta_(maximum_frame_delta), unsupported_floor_delta_(unsupported_floor_delta) {}

  void reset() { history_.clear(); ready_ = false; floor_z_ = 0.0; change_count_ = 0; candidate_z_ = 0.0; }
  bool ready() const { return ready_; }
  double floorZ() const { return floor_z_; }
  int changeCount() const { return change_count_; }
  double candidateZ() const { return candidate_z_; }
  double confidence() const {
    if (!ready_) return std::min(0.99, static_cast<double>(history_.size()) / initialization_frames_);
    return std::max(0.0, 1.0 - std::min(1.0, static_cast<double>(change_count_) / floor_change_frames_));
  }
  bool floorChangeDetected() const { return change_count_ >= floor_change_frames_; }

  void update(const GroundSample& sample) {
    if (!sample.valid) return;
    candidate_z_ = sample.z;
    if (!ready_) {
      history_.push_back(sample.z);
      if (static_cast<int>(history_.size()) >= initialization_frames_) {
        auto middle = history_.begin() + history_.size() / 2;
        std::nth_element(history_.begin(), middle, history_.end());
        floor_z_ = *middle;
        ready_ = true;
      }
      return;
    }
    const double delta = sample.z - floor_z_;
    if (std::abs(delta) > unsupported_floor_delta_) {
      ++change_count_;
      return;
    }
    change_count_ = 0;
    if (std::abs(delta) <= maximum_frame_delta_)
      floor_z_ += std::max(-0.01, std::min(0.01, delta * 0.1));
  }

 private:
  int initialization_frames_, floor_change_frames_;
  double maximum_frame_delta_, unsupported_floor_delta_;
  std::vector<double> history_;
  bool ready_{false};
  double floor_z_{0.0}, candidate_z_{0.0};
  int change_count_{0};
};

class OccupancyIntegrator {
 public:
  OccupancyIntegrator(double resolution, double width_m, double height_m, double p_free, double p_occupied,
                      unsigned minimum_observations,
                      double origin_x = std::numeric_limits<double>::quiet_NaN(),
                      double origin_y = std::numeric_limits<double>::quiet_NaN(),
                      unsigned occupied_clear_confirmations = 3)
      : resolution_(resolution), width_(static_cast<unsigned>(std::ceil(width_m / resolution))),
        height_(static_cast<unsigned>(std::ceil(height_m / resolution))),
        origin_x_(std::isfinite(origin_x) ? origin_x : -0.5 * width_ * resolution),
        origin_y_(std::isfinite(origin_y) ? origin_y : -0.5 * height_ * resolution),
        free_delta_(static_cast<float>(logit(p_free))), occupied_delta_(static_cast<float>(logit(p_occupied))),
        minimum_observations_(minimum_observations),
        occupied_clear_confirmations_(occupied_clear_confirmations) {
    if (occupied_clear_confirmations_ < 1)
      throw std::invalid_argument("occupied clear confirmations must be positive");
    reset();
  }

  void reset() {
    evidence_.assign(width_ * height_, std::numeric_limits<float>::quiet_NaN());
    observations_.assign(width_ * height_, 0);
    occupied_latched_.assign(width_ * height_, 0);
    clear_confirmations_.assign(width_ * height_, 0);
    frame_updates_.assign(width_ * height_, kNone);
    touched_indices_.clear();
    frame_active_ = false;
  }
  void beginFrame() {
    if (frame_active_) throw std::logic_error("occupancy frame already active");
    frame_active_ = true;
    touched_indices_.clear();
  }
  void endFrame() {
    if (!frame_active_) throw std::logic_error("no occupancy frame is active");
    for (const std::size_t index : touched_indices_) {
      const uint8_t update = frame_updates_[index];
      if (update == kOccupiedEndpoint) {
        clear_confirmations_[index] = 0;
        integrate(index, occupied_delta_);
        const double probability = 1.0 / (1.0 + std::exp(-evidence_[index]));
        if (observations_[index] >= minimum_observations_ && probability >= 0.65)
          occupied_latched_[index] = 1;
      } else if (occupied_latched_[index]) {
        // A 3-D ray passing over or beside an object does not prove the whole
        // 2-D column is empty.  Only repeated, near-floor endpoints in this
        // exact cell may release an already-confirmed obstacle.
        if (update == kFreeEndpoint) {
          if (clear_confirmations_[index] != std::numeric_limits<uint16_t>::max())
            ++clear_confirmations_[index];
          if (clear_confirmations_[index] >= occupied_clear_confirmations_) {
            occupied_latched_[index] = 0;
            clear_confirmations_[index] = 0;
            evidence_[index] = -4.0f;
          }
        }
      } else if (update == kFreeRay || update == kFreeEndpoint) {
        integrate(index, free_delta_);
      }
      frame_updates_[index] = kNone;
    }
    touched_indices_.clear();
    frame_active_ = false;
  }
  unsigned width() const { return width_; }
  unsigned height() const { return height_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  bool worldToCell(double x, double y, int& mx, int& my) const {
    mx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    my = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return inside(mx, my);
  }

  bool raytrace(double sx, double sy, double ex, double ey, bool occupied) {
    if (!frame_active_) throw std::logic_error("raytrace requires an active frame");
    if (!clip(sx, sy, ex, ey)) return false;
    int x0, y0, x1, y1;
    if (!worldToCell(sx, sy, x0, y0) || !worldToCell(ex, ey, x1, y1)) return false;
    int dx = std::abs(x1 - x0), step_x = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), step_y = y0 < y1 ? 1 : -1, error = dx + dy;
    int x = x0, y = y0;
    while (x != x1 || y != y1) {
      record(x, y, kFreeRay);
      const int twice = 2 * error;
      if (twice >= dy) { error += dy; x += step_x; }
      if (twice <= dx) { error += dx; y += step_y; }
    }
    record(x1, y1, occupied ? kOccupiedEndpoint : kFreeEndpoint);
    return true;
  }

  std::vector<int8_t> data(std::size_t& occupied, std::size_t& free, std::size_t& unknown) const {
    std::vector<int8_t> result(evidence_.size(), -1); occupied = free = unknown = 0;
    for (std::size_t i = 0; i < evidence_.size(); ++i) {
      if (!std::isfinite(evidence_[i]) || observations_[i] < minimum_observations_) { ++unknown; continue; }
      if (occupied_latched_[i]) { result[i] = 100; ++occupied; continue; }
      const double probability = 1.0 / (1.0 + std::exp(-evidence_[i]));
      if (probability >= 0.65) { result[i] = 100; ++occupied; }
      else if (probability <= 0.4) { result[i] = 0; ++free; }
      else ++unknown;
    }
    return result;
  }

  double boundaryMargin(double x, double y) const {
    return std::min(std::min(x - origin_x_, origin_x_ + width_ * resolution_ - x),
                    std::min(y - origin_y_, origin_y_ + height_ * resolution_ - y));
  }

 private:
  enum FrameUpdate : uint8_t {
    kNone = 0,
    kFreeRay = 1,
    kFreeEndpoint = 2,
    kOccupiedEndpoint = 3,
  };
  static double logit(double p) { return std::log(p / (1.0 - p)); }
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < static_cast<int>(width_) && y < static_cast<int>(height_); }
  void integrate(std::size_t index, float delta) {
    if (!std::isfinite(evidence_[index])) evidence_[index] = 0.0f;
    evidence_[index] = std::max(-4.0f, std::min(4.0f, evidence_[index] + delta));
    if (observations_[index] != std::numeric_limits<uint16_t>::max()) ++observations_[index];
  }
  void record(int x, int y, FrameUpdate update) {
    if (!inside(x, y)) return;
    const std::size_t index = static_cast<std::size_t>(y) * width_ + x;
    if (frame_updates_[index] == kNone) touched_indices_.push_back(index);
    if (update > frame_updates_[index]) frame_updates_[index] = update;
  }
  bool clip(double& x0, double& y0, double& x1, double& y1) const {
    const double xmin = origin_x_ + 1e-6, ymin = origin_y_ + 1e-6;
    const double xmax = origin_x_ + width_ * resolution_ - 1e-6, ymax = origin_y_ + height_ * resolution_ - 1e-6;
    double t0 = 0.0, t1 = 1.0, dx = x1 - x0, dy = y1 - y0;
    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};
    for (int i = 0; i < 4; ++i) {
      if (std::abs(p[i]) < 1e-12) { if (q[i] < 0.0) return false; continue; }
      const double ratio = q[i] / p[i];
      if (p[i] < 0.0) t0 = std::max(t0, ratio); else t1 = std::min(t1, ratio);
      if (t0 > t1) return false;
    }
    const double ox = x0, oy = y0;
    x0 = ox + t0 * dx; y0 = oy + t0 * dy;
    x1 = ox + t1 * dx; y1 = oy + t1 * dy;
    return true;
  }

  double resolution_;
  unsigned width_, height_;
  double origin_x_, origin_y_;
  float free_delta_, occupied_delta_;
  unsigned minimum_observations_, occupied_clear_confirmations_;
  std::vector<float> evidence_;
  std::vector<uint16_t> observations_;
  std::vector<uint8_t> occupied_latched_, frame_updates_;
  std::vector<uint16_t> clear_confirmations_;
  std::vector<std::size_t> touched_indices_;
  bool frame_active_{false};
};

}  // namespace a1_floor_mapping
