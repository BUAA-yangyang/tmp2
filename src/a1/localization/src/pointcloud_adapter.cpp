#include <cmath>
#include <mutex>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <diagnostic_updater/diagnostic_updater.h>

class PointcloudAdapter {
 public:
  PointcloudAdapter() : pnh_("~") {
    pnh_.param("input_topic", input_, std::string("/scan"));
    pnh_.param("output_topic", output_, std::string("/a1_localization/livox_pointcloud"));
    pnh_.param("expected_frame", frame_, std::string("laser_livox"));
    pnh_.param("strict_frame_check", strict_frame_, true);
    pnh_.param("drop_non_finite", drop_non_finite_, true);
    pnh_.param("default_intensity", default_intensity_, 0.0f);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_, 2);
    sub_ = nh_.subscribe(input_, 2, &PointcloudAdapter::callback, this,
                         ros::TransportHints().tcpNoDelay());
    updater_.setHardwareID("a1_livox_pointcloud_adapter");
    updater_.add("pointcloud adapter", this, &PointcloudAdapter::diagnose);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &PointcloudAdapter::timer, this);
  }
 private:
  void callback(const sensor_msgs::PointCloud::ConstPtr& in) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (strict_frame_ && in->header.frame_id != frame_) { ++errors_; last_error_ = "unexpected frame"; return; }
    if (have_stamp_ && in->header.stamp <= last_stamp_) { ++errors_; last_error_ = "non-monotonic timestamp"; return; }
    last_stamp_ = in->header.stamp; have_stamp_ = true;
    const sensor_msgs::ChannelFloat32* intensity = nullptr;
    for (const auto& c : in->channels) if (c.name == "intensity") { intensity = &c; break; }
    const bool valid_intensity = intensity && intensity->values.size() == in->points.size();
    sensor_msgs::PointCloud2 out; out.header = in->header; out.height = 1; out.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);
    std::size_t count = 0;
    for (const auto& p : in->points) if (!drop_non_finite_ || (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))) ++count;
    mod.resize(count); out.is_dense = true;
    sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z"), i(out, "intensity");
    for (std::size_t n = 0; n < in->points.size(); ++n) {
      const auto& p = in->points[n]; if (drop_non_finite_ && (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))) continue;
      *x = p.x; *y = p.y; *z = p.z; *i = valid_intensity ? intensity->values[n] : default_intensity_;
      ++x; ++y; ++z; ++i;
    }
    pub_.publish(out); ++published_; last_error_.clear();
  }
  void timer(const ros::WallTimerEvent&) { updater_.update(); }
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper& s) {
    std::lock_guard<std::mutex> lock(mutex_); s.summary(errors_ ? diagnostic_msgs::DiagnosticStatus::ERROR : (published_ ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN), errors_ ? last_error_ : (published_ ? "healthy" : "waiting for pointcloud")); s.add("input_topic", input_); s.add("output_topic", output_); s.add("published_frames", published_); s.add("errors", errors_);
  }
  ros::NodeHandle nh_, pnh_; ros::Subscriber sub_; ros::Publisher pub_; ros::WallTimer timer_; diagnostic_updater::Updater updater_; std::mutex mutex_; std::string input_, output_, frame_, last_error_; bool strict_frame_=true, drop_non_finite_=true, have_stamp_=false; float default_intensity_=0.0f; ros::Time last_stamp_; unsigned long published_=0, errors_=0;
};
int main(int argc, char** argv) { ros::init(argc, argv, "a1_pointcloud_adapter"); PointcloudAdapter node; ros::spin(); return 0; }
