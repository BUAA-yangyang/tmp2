#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <a1_navigation_interfaces/Doorway.h>
#include <a1_navigation_interfaces/DoorwayArray.h>
#include <a1_navigation_interfaces/WallSegment.h>
#include <a1_navigation_interfaces/WallSegmentArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Header.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "a1_floor_mapping/core.h"
#include "a1_floor_mapping/door_wall.h"

namespace {
diagnostic_msgs::KeyValue kv(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue result; result.key = key; result.value = value; return result;
}
template <typename T> std::string number(T value) { return std::to_string(value); }
bool isFiniteValue(double value) { return std::isfinite(value); }
bool validQuaternion(double x, double y, double z, double w) {
  const double norm = x*x + y*y + z*z + w*w;
  return isFiniteValue(norm) && norm > 1e-8;
}
Eigen::Affine3f eigenTransform(const geometry_msgs::TransformStamped& transform) {
  const auto& q = transform.transform.rotation; const auto& t = transform.transform.translation;
  Eigen::Affine3f result = Eigen::Affine3f::Identity(); result.translation() << t.x, t.y, t.z;
  result.linear() = Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix(); return result;
}
}  // namespace

class FloorMappingNode {
 public:
  FloorMappingNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    loadParameters();
    ground_.reset(new a1_floor_mapping::GroundEstimator(init_frames_, floor_change_frames_, max_floor_delta_, unsupported_delta_));
    grid_.reset(new a1_floor_mapping::OccupancyIntegrator(
        resolution_, grid_width_m_, grid_height_m_, p_free_, p_occ_,
        minimum_observations_, grid_origin_x_, grid_origin_y_));
    recognizer_.reset(new a1_floor_mapping::DoorWallRecognizer(door_wall_parameters_));
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic_, 2);
    clearing_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(clearing_cloud_topic_, 2);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
    walls_pub_ = nh_.advertise<a1_navigation_interfaces::WallSegmentArray>(walls_topic_, 1);
    doorways_pub_ = nh_.advertise<a1_navigation_interfaces::DoorwayArray>(doorways_topic_, 1);
    status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>(status_topic_, 1, true);
    structure_status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>(structure_status_topic_, 1, true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 1, true);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &FloorMappingNode::cloudCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &FloorMappingNode::odomCallback, this);
    localization_sub_ = nh_.subscribe(localization_topic_, 2, &FloorMappingNode::localizationCallback, this);
    supervisor_sub_ = nh_.subscribe(supervisor_topic_, 2, &FloorMappingNode::supervisorCallback, this);
    reset_service_ = nh_.advertiseService(reset_service_name_, &FloorMappingNode::resetCallback, this);
    status_timer_ = nh_.createTimer(ros::Duration(0.2), &FloorMappingNode::statusTimer, this);
    map_timer_ = nh_.createTimer(ros::Duration(1.0 / map_publish_rate_), &FloorMappingNode::mapTimer, this);
    tf_queue_timer_ = nh_.createWallTimer(ros::WallDuration(tf_queue_poll_wall_), &FloorMappingNode::tfQueueTimer, this);
    setState("WAITING_FOR_LOCALIZATION", "startup");
  }

 private:
  template <typename T> void param(const std::string& name, T& value, const T& fallback) { pnh_.param(name, value, fallback); }
  void positive(const std::string& name, double value) { if (!isFiniteValue(value) || value <= 0.0) throw std::runtime_error("invalid parameter: " + name); }
  void loadParameters() {
    param("frames/odom", odom_frame_, std::string("odom")); param("frames/base", base_frame_, std::string("base")); param("frames/sensor", sensor_frame_, std::string("laser_livox"));
    param("topics/pointcloud", cloud_topic_, std::string("/a1_localization/livox_pointcloud")); param("topics/odom", odom_topic_, std::string("/a1/localization/odom"));
    param("topics/localization_status", localization_topic_, std::string("/a1/localization/status")); param("topics/supervisor_status", supervisor_topic_, std::string("/a1/localization/supervisor_status"));
    param("topics/obstacle_cloud", out_cloud_topic_, std::string("/a1/floor_mapping/obstacle_cloud")); param("topics/clearing_cloud", clearing_cloud_topic_, std::string("/a1/floor_mapping/clearing_cloud")); param("topics/occupancy_grid", map_topic_, std::string("/a1/floor_mapping/map"));
    param("topics/walls", walls_topic_, std::string("/a1/floor_mapping/walls")); param("topics/doorways", doorways_topic_, std::string("/a1/floor_mapping/doorways"));
    param("topics/status", status_topic_, std::string("/a1/floor_mapping/status")); param("topics/structure_status", structure_status_topic_, std::string("/a1/floor_mapping/structure_status")); param("topics/diagnostics", diagnostics_topic_, std::string("/a1/floor_mapping/diagnostics"));
    param("services/reset", reset_service_name_, std::string("/a1/floor_mapping/reset"));
    param("timeouts/tf_queue_ros", tf_queue_ros_, 0.30); param("timeouts/tf_queue_wall", tf_queue_wall_, 2.0); param("timeouts/tf_queue_poll_wall", tf_queue_poll_wall_, 0.01); param("timeouts/input_degraded", input_degraded_, 1.0); param("timeouts/input_lost", input_lost_, 3.0);
    param("queues/pointcloud", pointcloud_queue_size_, 10);
    param("recovery/valid_frames", recovery_frames_, 3); param("ground/search_radius", ground_radius_, 2.5); param("ground/min_relative_z", min_relative_z_, -0.8);
    param("ground/max_relative_z", max_relative_z_, 0.15); param("ground/minimum_candidates", min_ground_candidates_, 80); param("ground/minimum_inlier_ratio", minimum_inlier_ratio_, 0.20);
    param("ground/maximum_dispersion", maximum_dispersion_, 0.08); param("ground/initialization_frames", init_frames_, 6); param("ground/maximum_frame_delta", max_floor_delta_, 0.08);
    param("ground/unsupported_floor_delta", unsupported_delta_, 0.35); param("ground/floor_change_frames", floor_change_frames_, 10);
    param("ground/invalid_frame_tolerance", invalid_ground_tolerance_, 3);
    param("filter/self_clear_radius", self_radius_, 0.45); param("filter/minimum_range", min_range_, 0.5); param("filter/maximum_range", max_range_, 8.0);
    param("filter/ground_band_below", ground_below_, 0.05); param("filter/ground_clearance", ground_clearance_, 0.08); param("filter/maximum_obstacle_height", max_obstacle_height_, 1.5); param("filter/voxel_leaf_size", voxel_leaf_, 0.05);
    param("grid/resolution", resolution_, 0.05); param("grid/width", grid_width_m_, 40.0); param("grid/height", grid_height_m_, 40.0);
    param("grid/origin_x", grid_origin_x_, -0.5 * grid_width_m_); param("grid/origin_y", grid_origin_y_, -0.5 * grid_height_m_); param("grid/publish_rate", map_publish_rate_, 1.0);
    param("grid/p_free", p_free_, 0.35); param("grid/p_occupied", p_occ_, 0.80); param("grid/minimum_observations", minimum_observations_, 2);
    param("door_wall/enabled", door_wall_enabled_, true);
    param("door_wall/extraction_cell_size", door_wall_parameters_.extraction_cell_size, 0.08); param("door_wall/line_fit_error", door_wall_parameters_.line_fit_error, 0.08); param("door_wall/max_segment_gap", door_wall_parameters_.max_segment_gap, 0.24); param("door_wall/elevator_opening_flank_min_length", door_wall_parameters_.elevator_opening_flank_min_length, 0.55);
    param("door_wall/minimum_wall_length", door_wall_parameters_.minimum_wall_length, 0.80); param("door_wall/minimum_wall_height", door_wall_parameters_.minimum_wall_height, 0.60); param("door_wall/elevator_opening_flank_min_support", door_wall_flank_minimum_support_, 6); door_wall_parameters_.elevator_opening_flank_min_support=static_cast<std::size_t>(door_wall_flank_minimum_support_); param("door_wall/wall_track_distance", door_wall_parameters_.wall_track_distance, 0.15); param("door_wall/wall_track_angle_rad", door_wall_parameters_.wall_track_angle_rad, 0.15);
    param("door_wall/collinear_distance", door_wall_parameters_.collinear_distance, 0.12); param("door_wall/opening_min_width", door_wall_parameters_.opening_min_width, 1.00); param("door_wall/opening_max_width", door_wall_parameters_.opening_max_width, 2.20); param("door_wall/doorway_match_distance", door_wall_parameters_.doorway_match_distance, 0.30);
    param("door_wall/state_min_height", door_wall_parameters_.state_min_height, 0.25); param("door_wall/state_band_depth", door_wall_parameters_.state_band_depth, 0.14); param("door_wall/free_ray_clearance", door_wall_parameters_.free_ray_clearance, 0.15); param("door_wall/open_fraction", door_wall_parameters_.open_fraction, 0.50); param("door_wall/closed_fraction", door_wall_parameters_.closed_fraction, 0.65);
    param("door_wall/robot_radius", door_wall_parameters_.robot_radius, 0.35); param("door_wall/traversable_margin", door_wall_parameters_.traversable_margin, 0.15); param("door_wall/approach_offset", door_wall_parameters_.approach_offset, 0.70);
    param("door_wall/minimum_wall_support", door_wall_minimum_support_, 12); param("door_wall/max_ransac_iterations", door_wall_max_ransac_iterations_, 360); param("door_wall/max_segments_per_frame", door_wall_max_segments_, 24); param("door_wall/wall_stable_frames", door_wall_stable_frames_, 3); param("door_wall/max_wall_misses", door_wall_max_wall_misses_, 6); param("door_wall/opening_stable_frames", doorway_stable_frames_, 3); param("door_wall/max_doorway_misses", doorway_max_misses_, 45); param("door_wall/state_bins", doorway_state_bins_, 8); param("door_wall/minimum_state_bins", doorway_minimum_state_bins_, 3); param("door_wall/state_stable_frames", doorway_state_stable_frames_, 3);
    door_wall_parameters_.minimum_wall_support = static_cast<std::size_t>(door_wall_minimum_support_); door_wall_parameters_.max_ransac_iterations = static_cast<std::size_t>(door_wall_max_ransac_iterations_); door_wall_parameters_.max_segments_per_frame = static_cast<std::size_t>(door_wall_max_segments_); door_wall_parameters_.wall_stable_frames = static_cast<std::size_t>(door_wall_stable_frames_); door_wall_parameters_.max_wall_misses = static_cast<std::size_t>(door_wall_max_wall_misses_); door_wall_parameters_.opening_stable_frames = static_cast<std::size_t>(doorway_stable_frames_); door_wall_parameters_.max_doorway_misses = static_cast<std::size_t>(doorway_max_misses_); door_wall_parameters_.state_bins = static_cast<std::size_t>(doorway_state_bins_); door_wall_parameters_.minimum_state_bins = static_cast<std::size_t>(doorway_minimum_state_bins_); door_wall_parameters_.state_stable_frames = static_cast<std::size_t>(doorway_state_stable_frames_);
    for (const auto& item : std::vector<std::pair<std::string,double>>{{"tf_queue_ros",tf_queue_ros_},{"tf_queue_wall",tf_queue_wall_},{"tf_queue_poll_wall",tf_queue_poll_wall_},{"input_degraded",input_degraded_},{"input_lost",input_lost_},{"ground_radius",ground_radius_},{"maximum_range",max_range_},{"resolution",resolution_},{"width",grid_width_m_},{"height",grid_height_m_},{"publish_rate",map_publish_rate_},{"door_wall/extraction_cell_size",door_wall_parameters_.extraction_cell_size},{"door_wall/line_fit_error",door_wall_parameters_.line_fit_error},{"door_wall/max_segment_gap",door_wall_parameters_.max_segment_gap},{"door_wall/minimum_wall_length",door_wall_parameters_.minimum_wall_length},{"door_wall/minimum_wall_height",door_wall_parameters_.minimum_wall_height},{"door_wall/wall_track_distance",door_wall_parameters_.wall_track_distance},{"door_wall/wall_track_angle_rad",door_wall_parameters_.wall_track_angle_rad},{"door_wall/collinear_distance",door_wall_parameters_.collinear_distance},{"door_wall/opening_min_width",door_wall_parameters_.opening_min_width},{"door_wall/opening_max_width",door_wall_parameters_.opening_max_width},{"door_wall/doorway_match_distance",door_wall_parameters_.doorway_match_distance},{"door_wall/state_min_height",door_wall_parameters_.state_min_height},{"door_wall/state_band_depth",door_wall_parameters_.state_band_depth},{"door_wall/free_ray_clearance",door_wall_parameters_.free_ray_clearance},{"door_wall/robot_radius",door_wall_parameters_.robot_radius},{"door_wall/traversable_margin",door_wall_parameters_.traversable_margin},{"door_wall/approach_offset",door_wall_parameters_.approach_offset}}) positive(item.first,item.second);
    if (!isFiniteValue(grid_origin_x_) || !isFiniteValue(grid_origin_y_) || input_degraded_ >= input_lost_ || recovery_frames_ < 1 || floor_change_frames_ < 2 || init_frames_ < 1 || invalid_ground_tolerance_ < 0 || min_ground_candidates_ < 3 || minimum_observations_ < 1 || pointcloud_queue_size_ < 1 || min_range_ >= max_range_ || p_free_ <= 0 || p_free_ >= 0.5 || p_occ_ <= 0.5 || p_occ_ >= 1 || minimum_inlier_ratio_ <= 0 || minimum_inlier_ratio_ > 1 || door_wall_minimum_support_ < 3 || door_wall_flank_minimum_support_ < 3 || door_wall_max_ransac_iterations_ < 1 || door_wall_max_segments_ < 1 || door_wall_stable_frames_ < 1 || door_wall_max_wall_misses_ < 1 || doorway_stable_frames_ < 1 || doorway_max_misses_ < 1 || doorway_state_bins_ < 1 || doorway_minimum_state_bins_ < 1 || doorway_minimum_state_bins_ > doorway_state_bins_ || doorway_state_stable_frames_ < 1 || door_wall_parameters_.opening_min_width > door_wall_parameters_.opening_max_width || door_wall_parameters_.open_fraction <= 0 || door_wall_parameters_.open_fraction > 1 || door_wall_parameters_.closed_fraction <= 0 || door_wall_parameters_.closed_fraction > 1) throw std::runtime_error("inconsistent floor_mapping parameters");
  }
  std::string value(const diagnostic_msgs::DiagnosticStatus& message, const std::string& key) const { for (const auto& item : message.values) if (item.key == key) return item.value; return ""; }
  bool sticky() const { return state_ == "LOST" || state_ == "FLOOR_CHANGE_UNSUPPORTED"; }
  void setState(const std::string& state, const std::string& reason) { if (sticky() && state != "RESETTING") return; state_ = state; reason_ = reason; }
  void failSticky(const std::string& reason) { state_ = "LOST"; reason_ = reason; map_valid_ = obstacle_valid_ = structure_valid_ = false; recovery_count_ = 0; }
  void clearSession(const std::string& reason) {
    ++floor_session_id_;
    ground_->reset(); grid_->reset(); recognizer_->reset(); pending_clouds_.clear(); map_valid_ = obstacle_valid_ = structure_valid_ = false; recovery_count_ = 0; invalid_ground_count_ = 0; last_cloud_stamp_ = ros::Time(); last_received_cloud_stamp_ = ros::Time(); last_success_tf_stamp_ = ros::Time(); last_cloud_wall_ = ros::WallTime(); last_received_cloud_wall_ = ros::WallTime(); last_success_tf_wall_ = ros::WallTime();
    input_points_ = finite_points_ = ground_points_ = obstacle_points_ = map_sequence_ = 0; wall_count_ = doorway_count_ = open_doorway_count_ = closed_doorway_count_ = 0; occupied_cells_ = free_cells_ = unknown_cells_ = 0; state_ = "RESETTING"; reason_ = reason;
  }
  void localizationCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_); localization_valid_ = value(*message,"state") == "TRACKING" && value(*message,"results_valid") == "true";
    if (!localization_valid_) { pending_clouds_.clear(); map_valid_ = obstacle_valid_ = structure_valid_ = false; recovery_count_ = 0; if (!sticky()) { state_="WAITING_FOR_LOCALIZATION"; reason_="localization_not_tracking"; } }
  }
  void supervisorCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_); const std::string raw=value(*message,"generation"); if(raw.empty()) return; int next;
    try { next=std::stoi(raw); } catch (...) { failSticky("invalid_generation"); return; }
    if (!generation_known_ || next != generation_) { generation_=next; generation_known_=true; clearSession(generation_ < 0 ? "generation_invalid" : "generation_changed"); }
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& p=message->pose.pose.position; const auto& q=message->pose.pose.orientation;
    if (message->header.frame_id != odom_frame_ || message->child_frame_id != base_frame_ || message->header.stamp.isZero() || !isFiniteValue(p.x)||!isFiniteValue(p.y)||!isFiniteValue(p.z)||!validQuaternion(q.x,q.y,q.z,q.w)) { odom_valid_=false; return; }
    if (!last_odom_stamp_.isZero() && message->header.stamp < last_odom_stamp_) { odom_valid_=false; failSticky("odom_time_regression"); return; }
    robot_x_=p.x; robot_y_=p.y; robot_z_=p.z; odom_valid_=true; last_odom_stamp_=message->header.stamp; last_odom_wall_=ros::WallTime::now();
  }
  bool validCloudLayout(const sensor_msgs::PointCloud2& message) const {
    if (message.width == 0 || message.height == 0 || message.point_step == 0 || message.data.empty() || message.row_step < message.point_step * message.width) return false;
    for (const std::string name : {"x","y","z"}) { bool found=false; for (const auto& field : message.fields) if (field.name==name && field.datatype==sensor_msgs::PointField::FLOAT32 && field.count==1 && field.offset+sizeof(float)<=message.point_step) { found=true; break; } if(!found) return false; }
    return message.data.size() >= static_cast<std::size_t>(message.row_step) * message.height;
  }
  bool validTransform(const geometry_msgs::TransformStamped& t) const { const auto& p=t.transform.translation;const auto&q=t.transform.rotation;return isFiniteValue(p.x)&&isFiniteValue(p.y)&&isFiniteValue(p.z)&&validQuaternion(q.x,q.y,q.z,q.w); }
  struct PendingCloud { sensor_msgs::PointCloud2::ConstPtr message; ros::WallTime received_wall; };
  a1_floor_mapping::GroundSample estimateGround(const pcl::PointCloud<pcl::PointXYZI>& cloud) const {
    std::vector<double> z; z.reserve(cloud.size());
    for(const auto&p:cloud){const double dx=p.x-robot_x_,dy=p.y-robot_y_,r2=dx*dx+dy*dy;if(r2>ground_radius_*ground_radius_||r2<self_radius_*self_radius_||p.z<robot_z_+min_relative_z_||p.z>robot_z_+max_relative_z_)continue;z.push_back(p.z);}
    // Once a floor is established, nearby walls and door frames may dominate
    // the cylindrical candidate set. Track the trusted floor by absolute
    // support instead of requiring it to remain a fixed share of all returns. If that
    // support disappears, retain the strict low-quantile path so a persistent
    // new floor can still be detected and rejected by GroundEstimator.
    return a1_floor_mapping::selectGroundSample(std::move(z),ground_->ready(),ground_->floorZ(),
                                                 min_ground_candidates_,minimum_inlier_ratio_,maximum_dispersion_);
  }
  geometry_msgs::Point structurePoint(const a1_floor_mapping::Point2& point) const {
    geometry_msgs::Point result; result.x=point.x; result.y=point.y; result.z=ground_->floorZ(); return result;
  }
  geometry_msgs::Vector3 structureNormal(const a1_floor_mapping::Point2& normal) const {
    geometry_msgs::Vector3 result; result.x=normal.x; result.y=normal.y; result.z=0.0; return result;
  }
  geometry_msgs::Pose structurePose(const a1_floor_mapping::Point2& point,const a1_floor_mapping::Point2& normal) const {
    geometry_msgs::Pose result; result.position=structurePoint(point); const double yaw=std::atan2(normal.y,normal.x); result.orientation.z=std::sin(0.5*yaw); result.orientation.w=std::cos(0.5*yaw); return result;
  }
  void publishStructureFrame(const a1_floor_mapping::DoorWallFrame& frame,const ros::Time& stamp) {
    std_msgs::Header header; header.stamp=stamp; header.frame_id=odom_frame_;
    a1_navigation_interfaces::WallSegmentArray walls; walls.header=header;
    for(const auto& track:frame.walls){a1_navigation_interfaces::WallSegment message;message.header=header;message.detection_id=track.id;message.localization_generation=generation_;message.floor_session_id=floor_session_id_;message.start=structurePoint(track.geometry.start);message.end=structurePoint(track.geometry.end);message.normal=structureNormal(a1_floor_mapping::perpendicular(track.geometry.direction));message.length=track.geometry.length;message.fit_error=track.geometry.residual;message.height_support=track.geometry.height_support;message.confidence=std::min(1.0f,static_cast<float>(track.observation_count)/static_cast<float>(door_wall_parameters_.wall_stable_frames));message.observation_count=track.observation_count;message.stable=track.stable;message.status="observed";walls.walls.push_back(message);}
    a1_navigation_interfaces::DoorwayArray doorways; doorways.header=header;
    for(const auto& track:frame.doorways){a1_navigation_interfaces::Doorway message;message.header=header;message.detection_id=track.id;message.localization_generation=generation_;message.floor_session_id=floor_session_id_;message.structure_type=a1_navigation_interfaces::Doorway::WALL_OPENING;message.state=static_cast<uint8_t>(track.state);message.center=structurePoint(track.center);message.normal=structureNormal(track.normal);message.left_boundary=structurePoint(track.left_boundary);message.right_boundary=structurePoint(track.right_boundary);message.width=track.width;message.usable_width=track.usable_width;message.entry_pose=structurePose(track.center+track.normal*(-door_wall_parameters_.approach_offset),track.normal);message.exit_pose=structurePose(track.center+track.normal*door_wall_parameters_.approach_offset,track.normal);message.passage_direction_resolved=false;message.traversable=track.stable&&(track.state==a1_floor_mapping::DoorState::OPEN||track.state==a1_floor_mapping::DoorState::PARTIALLY_OPEN)&&track.usable_width>=2.0*door_wall_parameters_.robot_radius+door_wall_parameters_.traversable_margin;message.confidence=track.confidence;message.observation_count=track.observation_count;message.stable=track.stable;message.control_id_matched=false;message.control_door_id="";message.control_id_confidence=0.0;message.door_kind="unclassified";doorways.doorways.push_back(message);}
    walls_pub_.publish(walls);doorways_pub_.publish(doorways);
    wall_count_=walls.walls.size();doorway_count_=doorways.doorways.size();open_doorway_count_=closed_doorway_count_=0;
    for(const auto& doorway:doorways.doorways){if(doorway.state==a1_navigation_interfaces::Doorway::OPEN)++open_doorway_count_;if(doorway.state==a1_navigation_interfaces::Doorway::CLOSED)++closed_doorway_count_;}
  }
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_); input_points_=static_cast<uint64_t>(message->width)*message->height;
    if(!localization_valid_||!generation_known_||!odom_valid_||sticky())return;
    if(message->header.frame_id!=sensor_frame_||message->header.stamp.isZero()||!validCloudLayout(*message)){failSticky("invalid_cloud_contract");return;}
    if(!last_received_cloud_stamp_.isZero()&&message->header.stamp<=last_received_cloud_stamp_){failSticky("pointcloud_time_regression");return;}
    last_received_cloud_stamp_=message->header.stamp;last_received_cloud_wall_=ros::WallTime::now();
    if(pending_clouds_.size()>=static_cast<std::size_t>(pointcloud_queue_size_)){++tf_failures_;pending_clouds_.pop_front();map_valid_=obstacle_valid_=false;recovery_count_=0;setState("WAITING_FOR_TF","pointcloud_tf_queue_overflow");}
    pending_clouds_.push_back(PendingCloud{message,ros::WallTime::now()});
    drainTfQueue();
  }
  void processCloud(const sensor_msgs::PointCloud2::ConstPtr& message,const geometry_msgs::TransformStamped& to_odom,const geometry_msgs::TransformStamped& to_base) {
    const auto started=std::chrono::steady_clock::now();
    if(!validTransform(to_odom)||!validTransform(to_base)){failSticky("invalid_tf");return;}
    const Eigen::Affine3f odom_from_sensor=eigenTransform(to_odom);const Eigen::Affine3f sensor_from_odom=odom_from_sensor.inverse();
    robot_x_=to_base.transform.translation.x;robot_y_=to_base.transform.translation.y;robot_z_=to_base.transform.translation.z;
    last_success_tf_stamp_=message->header.stamp;last_success_tf_wall_=ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr sensor(new pcl::PointCloud<pcl::PointXYZI>),world(new pcl::PointCloud<pcl::PointXYZI>),filtered(new pcl::PointCloud<pcl::PointXYZI>);try{pcl::fromROSMsg(*message,*sensor);}catch(const std::exception&){failSticky("pointcloud_conversion_failed");return;}pcl::transformPointCloud(*sensor,*world,eigenTransform(to_odom));
    pcl::PointCloud<pcl::PointXYZI> finite_cloud;finite_cloud.reserve(world->size());for(const auto&p:*world)if(pcl::isFinite(p))finite_cloud.push_back(p);finite_points_=finite_cloud.size();if(finite_points_==0){failSticky("no_finite_points");return;}
    pcl::VoxelGrid<pcl::PointXYZI> voxel;voxel.setInputCloud(finite_cloud.makeShared());voxel.setLeafSize(voxel_leaf_,voxel_leaf_,voxel_leaf_);voxel.filter(*filtered);
    const auto sample=estimateGround(*filtered);ground_candidate_z_=sample.z;ground_dispersion_=sample.dispersion;ground_candidates_=sample.candidates;ground_inliers_=sample.inliers;
    if(!sample.valid){++invalid_ground_count_;if(!ground_->ready()||invalid_ground_count_>invalid_ground_tolerance_){map_valid_=obstacle_valid_=structure_valid_=false;recovery_count_=0;setState(ground_->ready()?"DEGRADED":"INITIALIZING_GROUND","invalid_ground_sample");return;}}else{invalid_ground_count_=0;ground_->update(sample);}
    if(ground_->floorChangeDetected()){state_="FLOOR_CHANGE_UNSUPPORTED";reason_="persistent_floor_delta";map_valid_=obstacle_valid_=structure_valid_=false;return;}
    if(!ground_->ready()){map_valid_=obstacle_valid_=structure_valid_=false;setState("INITIALIZING_GROUND","collecting_stable_ground");return;}
    if(recovery_count_<recovery_frames_){++recovery_count_;map_valid_=obstacle_valid_=structure_valid_=false;setState("DEGRADED","recovering_valid_frames");last_cloud_stamp_=message->header.stamp;last_cloud_wall_=ros::WallTime::now();return;}
    pcl::PointCloud<pcl::PointXYZI> obstacle_world,clearing_world;std::vector<const pcl::PointXYZI*> ground_returns,obstacle_returns;std::vector<a1_floor_mapping::HeightPoint> structure_points;ground_points_=obstacle_points_=0;const double sx=to_odom.transform.translation.x,sy=to_odom.transform.translation.y;
    for(const auto&p:*filtered){const double range=std::hypot(p.x-sx,p.y-sy);if(range<min_range_||range>max_range_||std::hypot(p.x-robot_x_,p.y-robot_y_)<self_radius_)continue;const double h=p.z-ground_->floorZ();const bool is_ground=h>=-ground_below_&&h<ground_clearance_;const bool is_obstacle=h>=ground_clearance_&&h<=max_obstacle_height_;if(!is_ground&&!is_obstacle)continue;if(is_ground){clearing_world.push_back(p);++ground_points_;ground_returns.push_back(&p);}else{obstacle_world.push_back(p);clearing_world.push_back(p);++obstacle_points_;obstacle_returns.push_back(&p);structure_points.push_back(a1_floor_mapping::HeightPoint{{p.x,p.y},h});}}
    for(const auto*p:ground_returns)grid_->raytrace(sx,sy,p->x,p->y,false);
    for(const auto*p:obstacle_returns)grid_->raytrace(sx,sy,p->x,p->y,true);
    pcl::PointCloud<pcl::PointXYZI> obstacle_sensor,clearing_sensor;pcl::transformPointCloud(obstacle_world,obstacle_sensor,sensor_from_odom);pcl::transformPointCloud(clearing_world,clearing_sensor,sensor_from_odom);sensor_msgs::PointCloud2 obstacle_output,clearing_output;pcl::toROSMsg(obstacle_sensor,obstacle_output);pcl::toROSMsg(clearing_sensor,clearing_output);obstacle_output.header.stamp=clearing_output.header.stamp=message->header.stamp;obstacle_output.header.frame_id=clearing_output.header.frame_id=sensor_frame_;obstacle_output.is_dense=clearing_output.is_dense=true;cloud_pub_.publish(obstacle_output);clearing_cloud_pub_.publish(clearing_output);
    if(door_wall_enabled_){bool elevator_scan_active=false;nh_.getParam("/frontier_explorer/runtime/elevator_scan_active",elevator_scan_active);const auto frame=recognizer_->update(structure_points,a1_floor_mapping::Point2{sx,sy},elevator_scan_active);if(elevator_scan_active){ROS_INFO_THROTTLE(1.0,"ELEVATOR_SHORT_FLANK_SCAN min_length=%.2f min_support=%zu normal_min_length=%.2f normal_min_support=%zu walls=%zu doorways=%zu",door_wall_parameters_.elevator_opening_flank_min_length,door_wall_parameters_.elevator_opening_flank_min_support,door_wall_parameters_.minimum_wall_length,door_wall_parameters_.minimum_wall_support,frame.walls.size(),frame.doorways.size());}publishStructureFrame(frame,message->header.stamp);structure_valid_=true;}else{wall_count_=doorway_count_=open_doorway_count_=closed_doorway_count_=0;structure_valid_=false;}
    last_cloud_stamp_=last_map_stamp_=message->header.stamp;last_cloud_wall_=ros::WallTime::now();++map_sequence_;map_valid_=obstacle_valid_=true;state_="MAPPING";reason_="healthy";minimum_boundary_margin_=std::min(minimum_boundary_margin_,grid_->boundaryMargin(robot_x_,robot_y_));processing_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  }
  void drainTfQueue(){
    while(!pending_clouds_.empty()&&!sticky()&&localization_valid_&&generation_known_){
      const PendingCloud pending=pending_clouds_.front();const ros::Time stamp=pending.message->header.stamp;
      const bool sensor_ready=tf_buffer_.canTransform(odom_frame_,sensor_frame_,stamp,ros::Duration(0.0));
      const bool base_ready=tf_buffer_.canTransform(odom_frame_,base_frame_,stamp,ros::Duration(0.0));
      if(!sensor_ready||!base_ready){
        const ros::Time now=ros::Time::now();const double ros_wait=now.isZero()?0.0:std::max(0.0,(now-stamp).toSec());const double wall_wait=(ros::WallTime::now()-pending.received_wall).toSec();
        if(ros_wait<=tf_queue_ros_&&wall_wait<=tf_queue_wall_)return;
        ++tf_failures_;pending_clouds_.pop_front();map_valid_=obstacle_valid_=false;recovery_count_=0;setState("WAITING_FOR_TF",sensor_ready?"base_tf_wait_timeout":"sensor_tf_wait_timeout");continue;
      }
      try{
        const auto to_odom=tf_buffer_.lookupTransform(odom_frame_,sensor_frame_,stamp,ros::Duration(0.0));
        const auto to_base=tf_buffer_.lookupTransform(odom_frame_,base_frame_,stamp,ros::Duration(0.0));
        pending_clouds_.pop_front();processCloud(pending.message,to_odom,to_base);
      }catch(const tf2::TransformException&){return;}
    }
  }
  void tfQueueTimer(const ros::WallTimerEvent&){std::lock_guard<std::mutex>lock(mutex_);drainTfQueue();}
  nav_msgs::OccupancyGrid makeMap(){nav_msgs::OccupancyGrid map;map.header.stamp=last_map_stamp_;map.header.frame_id=odom_frame_;map.info.map_load_time=last_map_stamp_;map.info.resolution=grid_->resolution();map.info.width=grid_->width();map.info.height=grid_->height();map.info.origin.position.x=grid_->originX();map.info.origin.position.y=grid_->originY();map.info.origin.orientation.w=1.0;map.data=grid_->data(occupied_cells_,free_cells_,unknown_cells_);return map;}
  void mapTimer(const ros::TimerEvent&){std::lock_guard<std::mutex>lock(mutex_);if(map_valid_&&state_=="MAPPING")map_pub_.publish(makeMap());}
  double age(const ros::Time& stamp)const{const ros::Time now=ros::Time::now();return stamp.isZero()||now.isZero()?-1.0:std::max(0.0,(now-stamp).toSec());}
  double wallAge(const ros::WallTime& stamp)const{return stamp.isZero()?-1.0:(ros::WallTime::now()-stamp).toSec();}
  diagnostic_msgs::DiagnosticStatus makeStatus(){const double cloud_age=age(last_cloud_stamp_),odom_age=age(last_odom_stamp_),tf_age=age(last_success_tf_stamp_);const bool structure_results_valid=door_wall_enabled_&&structure_valid_&&map_valid_&&state_=="MAPPING";diagnostic_msgs::DiagnosticStatus s;s.name="a1_floor_mapping/health";s.hardware_id="livox_floor_mapper";s.level=state_=="MAPPING"?diagnostic_msgs::DiagnosticStatus::OK:(sticky()?diagnostic_msgs::DiagnosticStatus::ERROR:diagnostic_msgs::DiagnosticStatus::WARN);s.message=state_;s.values={kv("state",state_),kv("reason",reason_),kv("map_valid",map_valid_?"true":"false"),kv("obstacle_cloud_valid",obstacle_valid_?"true":"false"),kv("structure_results_valid",structure_results_valid?"true":"false"),kv("localization_generation",generation_known_?number(generation_):"unknown"),kv("floor_session_id",number(floor_session_id_)),kv("floor_id","unassigned"),kv("floor_z",number(ground_->floorZ())),kv("floor_candidate_z",number(ground_candidate_z_)),kv("floor_dispersion",number(ground_dispersion_)),kv("floor_confidence",number(ground_->confidence())),kv("floor_change_count",number(ground_->changeCount())),kv("recovery_valid_frames",number(recovery_count_)),kv("pointcloud_age_sec",number(cloud_age)),kv("pointcloud_input_age_sec",number(age(last_received_cloud_stamp_))),kv("odom_age_sec",number(odom_age)),kv("last_success_tf_age_sec",number(tf_age)),kv("pointcloud_wall_heartbeat_age_sec",number(wallAge(last_cloud_wall_))),kv("pointcloud_input_wall_heartbeat_age_sec",number(wallAge(last_received_cloud_wall_))),kv("odom_wall_heartbeat_age_sec",number(wallAge(last_odom_wall_))),kv("tf_wall_heartbeat_age_sec",number(wallAge(last_success_tf_wall_))),kv("tf_failure_count",number(tf_failures_)),kv("tf_pending_clouds",number(pending_clouds_.size())),kv("input_points",number(input_points_)),kv("finite_points",number(finite_points_)),kv("ground_candidates",number(ground_candidates_)),kv("ground_inliers",number(ground_inliers_)),kv("ground_points",number(ground_points_)),kv("obstacle_points",number(obstacle_points_)),kv("wall_count",number(wall_count_)),kv("doorway_count",number(doorway_count_)),kv("doorway_open_count",number(open_doorway_count_)),kv("doorway_closed_count",number(closed_doorway_count_)),kv("occupied_cells",number(occupied_cells_)),kv("free_cells",number(free_cells_)),kv("unknown_cells",number(unknown_cells_)),kv("processing_time_ms",number(processing_ms_)),kv("minimum_boundary_margin_m",isFiniteValue(minimum_boundary_margin_)?number(minimum_boundary_margin_):"unknown"),kv("map_update_sequence",number(map_sequence_))};return s;}
  diagnostic_msgs::DiagnosticStatus makeStructureStatus(){const bool valid=door_wall_enabled_&&structure_valid_&&map_valid_&&state_=="MAPPING";diagnostic_msgs::DiagnosticStatus s;s.name="a1_floor_mapping/structure";s.hardware_id="livox_floor_mapper";s.level=valid?diagnostic_msgs::DiagnosticStatus::OK:diagnostic_msgs::DiagnosticStatus::WARN;s.message=valid?"STRUCTURE_READY":"STRUCTURE_UNAVAILABLE";s.values={kv("results_valid",valid?"true":"false"),kv("reason",valid?"healthy":reason_),kv("localization_generation",generation_known_?number(generation_):"unknown"),kv("floor_session_id",number(floor_session_id_)),kv("frame_id",odom_frame_),kv("wall_count",number(wall_count_)),kv("doorway_count",number(doorway_count_)),kv("doorway_open_count",number(open_doorway_count_)),kv("doorway_closed_count",number(closed_doorway_count_)),kv("pointcloud_age_sec",number(age(last_cloud_stamp_))),kv("processing_time_ms",number(processing_ms_))};return s;}
  void statusTimer(const ros::TimerEvent&){std::lock_guard<std::mutex>lock(mutex_);if(!sticky()){const double cloud_input_age=age(last_received_cloud_stamp_),odom_age=age(last_odom_stamp_);if((cloud_input_age>=input_lost_&&!last_received_cloud_stamp_.isZero())||(odom_age>=input_lost_&&!last_odom_stamp_.isZero())){map_valid_=obstacle_valid_=structure_valid_=false;recovery_count_=0;state_="LOST";reason_="input_timeout";}else if((cloud_input_age>=input_degraded_&&!last_received_cloud_stamp_.isZero())||(odom_age>=input_degraded_&&!last_odom_stamp_.isZero())){map_valid_=obstacle_valid_=structure_valid_=false;recovery_count_=0;setState("DEGRADED","input_stale");}}auto s=makeStatus();status_pub_.publish(s);structure_status_pub_.publish(makeStructureStatus());diagnostic_msgs::DiagnosticArray a;a.header.stamp=ros::Time::now();a.status.push_back(s);diagnostics_pub_.publish(a);}
  bool resetCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response&response){std::lock_guard<std::mutex>lock(mutex_);clearSession("explicit_reset");response.success=true;response.message="floor session cleared";return true;}

  ros::NodeHandle nh_,pnh_;ros::Subscriber cloud_sub_,odom_sub_,localization_sub_,supervisor_sub_;ros::Publisher cloud_pub_,clearing_cloud_pub_,map_pub_,walls_pub_,doorways_pub_,status_pub_,structure_status_pub_,diagnostics_pub_;ros::ServiceServer reset_service_;ros::Timer status_timer_,map_timer_;ros::WallTimer tf_queue_timer_;tf2_ros::Buffer tf_buffer_;tf2_ros::TransformListener tf_listener_;std::mutex mutex_;
  std::unique_ptr<a1_floor_mapping::GroundEstimator> ground_;std::unique_ptr<a1_floor_mapping::OccupancyIntegrator> grid_;std::unique_ptr<a1_floor_mapping::DoorWallRecognizer> recognizer_;
  std::string odom_frame_,base_frame_,sensor_frame_,cloud_topic_,odom_topic_,localization_topic_,supervisor_topic_,out_cloud_topic_,clearing_cloud_topic_,map_topic_,walls_topic_,doorways_topic_,status_topic_,structure_status_topic_,diagnostics_topic_,reset_service_name_,state_,reason_;
  double tf_queue_ros_,tf_queue_wall_,tf_queue_poll_wall_,input_degraded_,input_lost_,ground_radius_,min_relative_z_,max_relative_z_,minimum_inlier_ratio_,maximum_dispersion_,max_floor_delta_,unsupported_delta_,self_radius_,min_range_,max_range_,ground_below_,ground_clearance_,max_obstacle_height_,voxel_leaf_,resolution_,grid_width_m_,grid_height_m_,grid_origin_x_,grid_origin_y_,map_publish_rate_,p_free_,p_occ_,robot_x_=0,robot_y_=0,robot_z_=0,ground_candidate_z_=0,ground_dispersion_=0,processing_ms_=0,minimum_boundary_margin_=std::numeric_limits<double>::infinity();
  a1_floor_mapping::DoorWallParameters door_wall_parameters_;int pointcloud_queue_size_,recovery_frames_,recovery_count_=0,min_ground_candidates_,init_frames_,floor_change_frames_,invalid_ground_tolerance_,invalid_ground_count_=0,minimum_observations_,generation_=-1,door_wall_minimum_support_=12,door_wall_flank_minimum_support_=6,door_wall_max_ransac_iterations_=360,door_wall_max_segments_=24,door_wall_stable_frames_=3,door_wall_max_wall_misses_=6,doorway_stable_frames_=3,doorway_max_misses_=45,doorway_state_bins_=8,doorway_minimum_state_bins_=3,doorway_state_stable_frames_=3;bool localization_valid_=false,generation_known_=false,odom_valid_=false,map_valid_=false,obstacle_valid_=false,structure_valid_=false,door_wall_enabled_=true;uint64_t floor_session_id_=0,tf_failures_=0,input_points_=0,finite_points_=0,ground_candidates_=0,ground_inliers_=0,ground_points_=0,obstacle_points_=0,map_sequence_=0;std::size_t wall_count_=0,doorway_count_=0,open_doorway_count_=0,closed_doorway_count_=0,occupied_cells_=0,free_cells_=0,unknown_cells_=0;std::deque<PendingCloud> pending_clouds_;ros::Time last_cloud_stamp_,last_received_cloud_stamp_,last_map_stamp_,last_odom_stamp_,last_success_tf_stamp_;ros::WallTime last_cloud_wall_,last_received_cloud_wall_,last_odom_wall_,last_success_tf_wall_;
};

int main(int argc,char**argv){ros::init(argc,argv,"floor_mapping");try{FloorMappingNode node;ros::spin();}catch(const std::exception&e){ROS_FATAL_STREAM("floor_mapping startup failed: "<<e.what());return 2;}return 0;}
