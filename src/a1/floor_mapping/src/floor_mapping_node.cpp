#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

namespace {
diagnostic_msgs::KeyValue kv(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue result; result.key = key; result.value = value; return result;
}
template <typename T> std::string str(const T& value) { return std::to_string(value); }
double logit(double p) { return std::log(p / (1.0 - p)); }
Eigen::Affine3f eigenTransform(const geometry_msgs::TransformStamped& t) {
  const auto& q=t.transform.rotation; const auto& v=t.transform.translation;
  Eigen::Affine3f result=Eigen::Affine3f::Identity(); result.translation()<<v.x,v.y,v.z;
  result.linear()=Eigen::Quaternionf(q.w,q.x,q.y,q.z).normalized().toRotationMatrix(); return result;
}
}

class FloorMappingNode {
 public:
  FloorMappingNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    loadParameters();
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic_, 2);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
    status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>(status_topic_, 1, true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 1, true);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &FloorMappingNode::cloudCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &FloorMappingNode::odomCallback, this);
    localization_sub_ = nh_.subscribe(localization_topic_, 2, &FloorMappingNode::localizationCallback, this);
    supervisor_sub_ = nh_.subscribe(supervisor_topic_, 2, &FloorMappingNode::supervisorCallback, this);
    reset_service_ = nh_.advertiseService("/a1/floor_mapping/reset", &FloorMappingNode::resetCallback, this);
    const double status_rate = 5.0;
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_rate), &FloorMappingNode::statusTimer, this);
    map_timer_ = nh_.createTimer(ros::Duration(1.0 / map_publish_rate_), &FloorMappingNode::mapTimer, this);
    initializeGrid();
    setState("WAITING_FOR_LOCALIZATION", "startup");
  }

 private:
  void requiredPositive(const std::string& name, double value) {
    if (!std::isfinite(value) || value <= 0.0) throw std::runtime_error("invalid positive parameter: " + name);
  }
  template <typename T> void param(const std::string& name, T& value, const T& fallback) { pnh_.param(name, value, fallback); }
  void loadParameters() {
    param("frames/odom", odom_frame_, std::string("odom")); param("frames/base", base_frame_, std::string("base"));
    param("frames/sensor", sensor_frame_, std::string("laser_livox"));
    param("topics/pointcloud", cloud_topic_, std::string("/a1_localization/livox_pointcloud"));
    param("topics/odom", odom_topic_, std::string("/a1/localization/odom"));
    param("topics/localization_status", localization_topic_, std::string("/a1/localization/status"));
    param("topics/supervisor_status", supervisor_topic_, std::string("/a1/localization/supervisor_status"));
    param("topics/obstacle_cloud", out_cloud_topic_, std::string("/a1/floor_mapping/obstacle_cloud"));
    param("topics/occupancy_grid", map_topic_, std::string("/a1/floor_mapping/map"));
    param("topics/status", status_topic_, std::string("/a1/floor_mapping/status"));
    param("topics/diagnostics", diagnostics_topic_, std::string("/a1/floor_mapping/diagnostics"));
    param("timeouts/tf_lookup", tf_timeout_, 0.10); param("timeouts/input_lost", input_lost_, 3.0);
    param("ground/search_radius", ground_radius_, 2.5); param("ground/min_relative_z", min_relative_z_, -0.8);
    param("ground/max_relative_z", max_relative_z_, 0.15); param("ground/minimum_candidates", min_ground_candidates_, 80);
    param("ground/initialization_frames", init_frames_, 6); param("ground/maximum_frame_delta", max_floor_delta_, 0.08);
    param("ground/unsupported_floor_delta", unsupported_delta_, 0.35);
    param("filter/self_clear_radius", self_radius_, 0.45); param("filter/minimum_range", min_range_, 0.5);
    param("filter/maximum_range", max_range_, 8.0); param("filter/ground_band_below", ground_below_, 0.05);
    param("filter/ground_clearance", ground_clearance_, 0.08); param("filter/maximum_obstacle_height", max_obstacle_height_, 1.5);
    param("filter/voxel_leaf_size", voxel_leaf_, 0.05);
    param("grid/resolution", resolution_, 0.05); param("grid/width", grid_width_m_, 40.0); param("grid/height", grid_height_m_, 40.0);
    param("grid/publish_rate", map_publish_rate_, 1.0); param("grid/p_free", p_free_, 0.35); param("grid/p_occupied", p_occ_, 0.70);
    requiredPositive("tf_lookup", tf_timeout_); requiredPositive("ground/search_radius", ground_radius_);
    requiredPositive("filter/maximum_range", max_range_); requiredPositive("grid/resolution", resolution_);
    requiredPositive("grid/width", grid_width_m_); requiredPositive("grid/height", grid_height_m_); requiredPositive("grid/publish_rate", map_publish_rate_);
    if (sensor_frame_.empty() || odom_frame_.empty() || min_range_ >= max_range_ || ground_clearance_ >= max_obstacle_height_ ||
        p_free_ <= 0.0 || p_free_ >= 0.5 || p_occ_ <= 0.5 || p_occ_ >= 1.0 || init_frames_ < 1 || min_ground_candidates_ < 3)
      throw std::runtime_error("inconsistent floor_mapping parameters");
  }
  void initializeGrid() {
    width_ = static_cast<unsigned>(std::ceil(grid_width_m_ / resolution_)); height_ = static_cast<unsigned>(std::ceil(grid_height_m_ / resolution_));
    origin_x_ = -0.5 * width_ * resolution_; origin_y_ = -0.5 * height_ * resolution_;
    evidence_.assign(width_ * height_, std::numeric_limits<float>::quiet_NaN());
    observations_.assign(width_ * height_, 0); map_sequence_ = 0; last_map_stamp_ = ros::Time(0);
  }
  void clearSession(const std::string& reason) {
    initializeGrid(); ground_history_.clear(); floor_ready_ = false; floor_z_ = 0.0; map_valid_ = false;
    last_cloud_stamp_ = ros::Time(0); setState("RESETTING", reason);
  }
  void setState(const std::string& state, const std::string& reason) { state_ = state; reason_ = reason; }
  std::string value(const diagnostic_msgs::DiagnosticStatus& msg, const std::string& key) const {
    for (const auto& item : msg.values) {
      if (item.key == key) return item.value;
    }
    return "";
  }
  void localizationCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); localization_valid_ = value(*msg, "state") == "TRACKING" && value(*msg, "results_valid") == "true";
    if (!localization_valid_) { map_valid_ = false; setState("WAITING_FOR_LOCALIZATION", "localization_not_tracking"); }
  }
  void supervisorCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); const std::string raw = value(*msg, "generation"); if (raw.empty()) return;
    int next = -1; try { next = std::stoi(raw); } catch (...) { setState("LOST", "invalid_generation"); map_valid_ = false; return; }
    if (!generation_known_) { generation_ = next; generation_known_ = true; clearSession("generation_initialized"); }
    else if (next != generation_) { generation_ = next; clearSession("generation_changed"); }
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); if (msg->header.frame_id != odom_frame_) return;
    robot_x_ = msg->pose.pose.position.x; robot_y_ = msg->pose.pose.position.y; robot_z_ = msg->pose.pose.position.z; odom_received_ = true; last_odom_wall_ = ros::WallTime::now();
  }
  bool estimateGround(const pcl::PointCloud<pcl::PointXYZI>& cloud, double& estimate) {
    std::vector<double> z; z.reserve(cloud.size());
    for (const auto& p : cloud) { const double dx=p.x-robot_x_, dy=p.y-robot_y_; if (dx*dx+dy*dy > ground_radius_*ground_radius_ || dx*dx+dy*dy < self_radius_*self_radius_) continue; if (p.z < robot_z_+min_relative_z_ || p.z > robot_z_+max_relative_z_) continue; z.push_back(p.z); }
    if (static_cast<int>(z.size()) < min_ground_candidates_) return false;
    const size_t q = z.size()/10; std::nth_element(z.begin(), z.begin()+q, z.end()); const double low=z[q];
    std::vector<double> band; for (double v : z) if (std::abs(v-low) <= 0.06) band.push_back(v); if (band.size() < static_cast<size_t>(min_ground_candidates_/2)) return false;
    auto mid=band.begin()+band.size()/2; std::nth_element(band.begin(),mid,band.end()); estimate=*mid; return true;
  }
  bool worldToCell(double x, double y, int& mx, int& my) const { mx=static_cast<int>(std::floor((x-origin_x_)/resolution_)); my=static_cast<int>(std::floor((y-origin_y_)/resolution_)); return mx>=0&&my>=0&&mx<static_cast<int>(width_)&&my<static_cast<int>(height_); }
  void addEvidence(int x, int y, float delta) { if(x<0||y<0||x>=static_cast<int>(width_)||y>=static_cast<int>(height_)) return; const size_t i=y*width_+x; if(!std::isfinite(evidence_[i])) evidence_[i]=0.0f; evidence_[i]=std::max(-4.0f,std::min(4.0f,evidence_[i]+delta)); observations_[i]++; }
  void raytrace(double sx,double sy,double ex,double ey,bool occupied) {
    int x0,y0,x1,y1; if(!worldToCell(sx,sy,x0,y0)||!worldToCell(ex,ey,x1,y1)) return; int dx=std::abs(x1-x0), sxn=x0<x1?1:-1, dy=-std::abs(y1-y0), syn=y0<y1?1:-1, err=dx+dy;
    int x=x0,y=y0; while(x!=x1||y!=y1){ addEvidence(x,y,static_cast<float>(logit(p_free_))); int e2=2*err;if(e2>=dy){err+=dy;x+=sxn;}if(e2<=dx){err+=dx;y+=syn;} }
    addEvidence(x1,y1,static_cast<float>(occupied?logit(p_occ_):logit(p_free_)));
  }
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); input_points_=msg->width*msg->height;
    if(!localization_valid_||!generation_known_||!odom_received_) return;
    if(msg->header.frame_id!=sensor_frame_||msg->header.stamp.isZero()){map_valid_=false;setState("LOST","invalid_cloud_header");return;}
    if(!last_cloud_stamp_.isZero()&&msg->header.stamp<=last_cloud_stamp_){map_valid_=false;setState("LOST","pointcloud_time_regression");return;}
    geometry_msgs::TransformStamped to_odom, to_sensor;
    try { to_odom=tf_buffer_.lookupTransform(odom_frame_,sensor_frame_,msg->header.stamp,ros::Duration(tf_timeout_)); to_sensor=tf_buffer_.lookupTransform(sensor_frame_,odom_frame_,msg->header.stamp,ros::Duration(tf_timeout_)); }
    catch(const tf2::TransformException& e){++tf_failures_;map_valid_=false;setState("WAITING_FOR_TF",e.what());return;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr sensor_cloud(new pcl::PointCloud<pcl::PointXYZI>), cloud(new pcl::PointCloud<pcl::PointXYZI>), filtered(new pcl::PointCloud<pcl::PointXYZI>); pcl::fromROSMsg(*msg,*sensor_cloud); pcl::transformPointCloud(*sensor_cloud,*cloud,eigenTransform(to_odom));
    pcl::PointCloud<pcl::PointXYZI> finite; finite.reserve(cloud->size()); for(const auto&p:*cloud)if(pcl::isFinite(p))finite.push_back(p); finite_points_=finite.size(); *cloud=finite;
    pcl::VoxelGrid<pcl::PointXYZI> voxel; voxel.setInputCloud(cloud); voxel.setLeafSize(voxel_leaf_,voxel_leaf_,voxel_leaf_); voxel.filter(*filtered);
    double candidate=0.0; if(!estimateGround(*filtered,candidate)){map_valid_=false;setState("INITIALIZING_GROUND","insufficient_ground_candidates");return;}
    if(!floor_ready_){ground_history_.push_back(candidate);if(static_cast<int>(ground_history_.size())<init_frames_){setState("INITIALIZING_GROUND","collecting_stable_ground");return;}auto mid=ground_history_.begin()+ground_history_.size()/2;std::nth_element(ground_history_.begin(),mid,ground_history_.end());floor_z_=*mid;floor_ready_=true;}
    else {const double delta=candidate-floor_z_;if(std::abs(delta)>unsupported_delta_){map_valid_=false;setState("FLOOR_CHANGE_UNSUPPORTED","persistent_floor_delta");return;}if(std::abs(delta)<=max_floor_delta_)floor_z_+=std::max(-0.01,std::min(0.01,delta*0.1));}
    pcl::PointCloud<pcl::PointXYZI> output_odom; output_odom.reserve(filtered->size()); ground_points_=0; obstacle_points_=0;
    const double sensor_x=to_odom.transform.translation.x,sensor_y=to_odom.transform.translation.y;
    for(const auto&p:*filtered){double dx=p.x-sensor_x,dy=p.y-sensor_y,d=std::hypot(dx,dy);if(d<min_range_||d>max_range_)continue;if(std::hypot(p.x-robot_x_,p.y-robot_y_)<self_radius_)continue;const double h=p.z-floor_z_;const bool ground=h>=-ground_below_&&h<ground_clearance_;const bool obstacle=h>=ground_clearance_&&h<=max_obstacle_height_;if(!ground&&!obstacle)continue;output_odom.push_back(p);if(ground)++ground_points_;else ++obstacle_points_;raytrace(sensor_x,sensor_y,p.x,p.y,obstacle);}
    pcl::PointCloud<pcl::PointXYZI> output_sensor;pcl::transformPointCloud(output_odom,output_sensor,eigenTransform(to_sensor));sensor_msgs::PointCloud2 output_sensor_msg;pcl::toROSMsg(output_sensor,output_sensor_msg);output_sensor_msg.header.stamp=msg->header.stamp;output_sensor_msg.header.frame_id=sensor_frame_;output_sensor_msg.is_dense=true;cloud_pub_.publish(output_sensor_msg);
    last_cloud_stamp_=msg->header.stamp;last_cloud_wall_=ros::WallTime::now();last_map_stamp_=msg->header.stamp;++map_sequence_;map_valid_=true;setState("MAPPING","healthy");
  }
  nav_msgs::OccupancyGrid makeMap() const { nav_msgs::OccupancyGrid map;map.header.stamp=last_map_stamp_;map.header.frame_id=odom_frame_;map.info.map_load_time=last_map_stamp_;map.info.resolution=resolution_;map.info.width=width_;map.info.height=height_;map.info.origin.position.x=origin_x_;map.info.origin.position.y=origin_y_;map.info.origin.orientation.w=1.0;map.data.resize(evidence_.size(),-1);for(size_t i=0;i<evidence_.size();++i){if(!std::isfinite(evidence_[i])||observations_[i]<2)continue;double p=1.0/(1.0+std::exp(-evidence_[i]));map.data[i]=p>=0.65?100:(p<=0.4?0:-1);}return map; }
  void mapTimer(const ros::TimerEvent&) { std::lock_guard<std::mutex> lock(mutex_);if(map_valid_&&state_=="MAPPING")map_pub_.publish(makeMap()); }
  diagnostic_msgs::DiagnosticStatus makeStatus() const {diagnostic_msgs::DiagnosticStatus s;s.name="a1_floor_mapping/health";s.hardware_id="livox_floor_mapper";s.level=state_=="MAPPING"?diagnostic_msgs::DiagnosticStatus::OK:(state_=="LOST"||state_=="FLOOR_CHANGE_UNSUPPORTED"?diagnostic_msgs::DiagnosticStatus::ERROR:diagnostic_msgs::DiagnosticStatus::WARN);s.message=state_;s.values={kv("state",state_),kv("reason",reason_),kv("map_valid",map_valid_?"true":"false"),kv("obstacle_cloud_valid",map_valid_?"true":"false"),kv("localization_generation",generation_known_?str(generation_):"unknown"),kv("floor_id",generation_known_?"session_floor_0":"unknown"),kv("floor_z",str(floor_z_)),kv("floor_confidence",floor_ready_?"1.0":"0.0"),kv("tf_failure_count",str(tf_failures_)),kv("input_points",str(input_points_)),kv("finite_points",str(finite_points_)),kv("ground_points",str(ground_points_)),kv("obstacle_points",str(obstacle_points_)),kv("map_update_sequence",str(map_sequence_))};return s;}
  void statusTimer(const ros::TimerEvent&) {std::lock_guard<std::mutex> lock(mutex_);if(map_valid_&&!last_cloud_wall_.isZero()&&(ros::WallTime::now()-last_cloud_wall_).toSec()>input_lost_){map_valid_=false;setState("DEGRADED","pointcloud_timeout");}auto s=makeStatus();status_pub_.publish(s);diagnostic_msgs::DiagnosticArray a;a.header.stamp=ros::Time::now();a.status.push_back(s);diagnostics_pub_.publish(a);}
  bool resetCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response){std::lock_guard<std::mutex> lock(mutex_);clearSession("explicit_reset");response.success=true;response.message="floor session cleared";return true;}

  ros::NodeHandle nh_,pnh_; ros::Subscriber cloud_sub_,odom_sub_,localization_sub_,supervisor_sub_;ros::Publisher cloud_pub_,map_pub_,status_pub_,diagnostics_pub_;ros::ServiceServer reset_service_;ros::Timer status_timer_,map_timer_;
  tf2_ros::Buffer tf_buffer_;tf2_ros::TransformListener tf_listener_;std::mutex mutex_;
  std::string odom_frame_,base_frame_,sensor_frame_,cloud_topic_,odom_topic_,localization_topic_,supervisor_topic_,out_cloud_topic_,map_topic_,status_topic_,diagnostics_topic_,state_,reason_;
  double tf_timeout_,input_lost_,ground_radius_,min_relative_z_,max_relative_z_,max_floor_delta_,unsupported_delta_,self_radius_,min_range_,max_range_,ground_below_,ground_clearance_,max_obstacle_height_,voxel_leaf_,resolution_,grid_width_m_,grid_height_m_,map_publish_rate_,p_free_,p_occ_,robot_x_=0,robot_y_=0,robot_z_=0,floor_z_=0,origin_x_=0,origin_y_=0;
  int min_ground_candidates_,init_frames_,generation_=-1;unsigned width_=0,height_=0;bool localization_valid_=false,generation_known_=false,odom_received_=false,floor_ready_=false,map_valid_=false;uint64_t tf_failures_=0,input_points_=0,finite_points_=0,ground_points_=0,obstacle_points_=0,map_sequence_=0;
  std::vector<double> ground_history_;std::vector<float> evidence_;std::vector<uint16_t> observations_;ros::Time last_cloud_stamp_,last_map_stamp_;ros::WallTime last_cloud_wall_,last_odom_wall_;
};

int main(int argc,char**argv){ros::init(argc,argv,"floor_mapping");try{FloorMappingNode node;ros::spin();}catch(const std::exception&e){ROS_FATAL_STREAM("floor_mapping startup failed: "<<e.what());return 2;}return 0;}
