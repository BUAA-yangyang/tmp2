#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
bool finitePose(const geometry_msgs::Pose& pose)
{
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
}

bool loadVector(ros::NodeHandle& nh, const std::string& name, std::size_t size,
                std::vector<double>* value)
{
    if (!nh.getParam(name, *value) || value->size() != size)
    {
        ROS_FATAL_STREAM("parameter ~" << name << " must contain " << size << " numbers");
        return false;
    }
    for (double component : *value)
    {
        if (!std::isfinite(component))
        {
            ROS_FATAL_STREAM("parameter ~" << name << " contains a non-finite value");
            return false;
        }
    }
    return true;
}

diagnostic_msgs::KeyValue keyValue(const std::string& key, const std::string& value)
{
    diagnostic_msgs::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
}
}  // namespace

class LocalizationPoseAdapter
{
public:
    LocalizationPoseAdapter() : nh_(), pnh_("~")
    {
        loadParameters();
        if (!valid_configuration_)
        {
            ros::requestShutdown();
            return;
        }

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 20);
        registered_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(output_registered_cloud_topic_, 5);
        // A latched map would remain apparently available after localization is invalidated.
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_map_topic_, 1, false);
        status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>(status_topic_, 1, true);
        diagnostics_pub_ =
            nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 1, true);

        odom_sub_ = nh_.subscribe(input_odom_topic_, 20,
                                  &LocalizationPoseAdapter::odomCallback, this);
        registered_cloud_sub_ = nh_.subscribe(input_registered_cloud_topic_, 5,
            &LocalizationPoseAdapter::registeredCloudCallback, this);
        map_sub_ = nh_.subscribe(input_map_topic_, 1,
                                 &LocalizationPoseAdapter::mapCallback, this);
        if (health_enabled_)
        {
            pointcloud_health_sub_ = nh_.subscribe(input_pointcloud_topic_, 10,
                &LocalizationPoseAdapter::pointcloudHealthCallback, this);
            imu_health_sub_ = nh_.subscribe(input_imu_topic_, 100,
                &LocalizationPoseAdapter::imuHealthCallback, this);
            if (monitor_clock_)
            {
                clock_sub_ = nh_.subscribe(clock_topic_, 10,
                                           &LocalizationPoseAdapter::clockCallback, this);
            }
        }
        health_timer_ = nh_.createTimer(ros::Duration(0.1),
                                        &LocalizationPoseAdapter::healthTimer, this);
        setState(health_enabled_ ? State::WAITING_FOR_SENSORS : State::TRACKING,
                 health_enabled_ ? "WAITING_FOR_INPUTS" : "HEALTH_GATE_DISABLED");
    }

    bool valid() const { return valid_configuration_; }

private:
    enum class State { STOPPED, WAITING_FOR_SENSORS, INITIALIZING, TRACKING, DEGRADED, LOST };

    struct InputWatch
    {
        bool received{false};
        ros::WallTime wall_time;
        ros::Time stamp;
    };

    void loadParameters()
    {
        pnh_.param("input_odom_topic", input_odom_topic_,
                    std::string("/a1_localization/fast_lio/odom_raw"));
        pnh_.param("output_odom_topic", output_odom_topic_,
                    std::string("/a1/localization/odom"));
        pnh_.param("input_registered_cloud_topic", input_registered_cloud_topic_,
                    std::string("/a1_localization/fast_lio/cloud_registered_raw"));
        pnh_.param("output_registered_cloud_topic", output_registered_cloud_topic_,
                    std::string("/a1/localization/cloud_registered"));
        pnh_.param("input_map_topic", input_map_topic_,
                    std::string("/a1_localization/fast_lio/map_raw"));
        pnh_.param("output_map_topic", output_map_topic_, std::string("/a1/localization/map"));
        pnh_.param("input_pointcloud_topic", input_pointcloud_topic_,
                    std::string("/a1_localization/livox_pointcloud"));
        pnh_.param("input_imu_topic", input_imu_topic_, std::string("/trunk_imu"));
        pnh_.param("clock_topic", clock_topic_, std::string("/clock"));
        pnh_.param("status_topic", status_topic_, std::string("/a1/localization/status"));
        pnh_.param("diagnostics_topic", diagnostics_topic_,
                    std::string("/a1/localization/diagnostics"));
        pnh_.param("input_world_frame", input_world_frame_, std::string("camera_init"));
        pnh_.param("input_body_frame", input_body_frame_, std::string("body"));
        pnh_.param("odom_frame", odom_frame_, std::string("odom"));
        pnh_.param("base_frame", base_frame_, std::string("base"));
        pnh_.param("world_alignment_enabled", world_alignment_enabled_, true);
        pnh_.param("world_frame", world_frame_, std::string("world"));
        pnh_.param("publish_tf", publish_tf_, true);
        pnh_.param("strict_input_frames", strict_input_frames_, true);
        pnh_.param("rewrite_registered_cloud_frame", rewrite_registered_cloud_frame_, true);
        pnh_.param("rewrite_map_frame", rewrite_map_frame_, true);
        pnh_.param("health_enabled", health_enabled_, true);
        pnh_.param("monitor_clock", monitor_clock_, true);
        pnh_.param("unknown_twist_variance", unknown_twist_variance_, 1.0e6);
        pnh_.param("sensor_warn_timeout", sensor_warn_timeout_, 0.5);
        pnh_.param("sensor_lost_timeout", sensor_lost_timeout_, 1.5);
        pnh_.param("odom_warn_timeout", odom_warn_timeout_, 0.5);
        pnh_.param("odom_lost_timeout", odom_lost_timeout_, 1.5);
        pnh_.param("clock_warn_timeout", clock_warn_timeout_, 0.5);
        pnh_.param("clock_lost_timeout", clock_lost_timeout_, 1.5);
        pnh_.param("initialization_samples", initialization_samples_, 5);
        pnh_.param("max_translation_jump", max_translation_jump_, 1.0);
        pnh_.param("max_rotation_jump", max_rotation_jump_, 1.0);

        valid_configuration_ = std::isfinite(unknown_twist_variance_) &&
            unknown_twist_variance_ > 0.0 && sensor_warn_timeout_ > 0.0 &&
            sensor_lost_timeout_ > sensor_warn_timeout_ && odom_warn_timeout_ > 0.0 &&
            odom_lost_timeout_ > odom_warn_timeout_ && clock_warn_timeout_ > 0.0 &&
            clock_lost_timeout_ > clock_warn_timeout_ && initialization_samples_ > 0 &&
            max_translation_jump_ > 0.0 && max_rotation_jump_ > 0.0;
        if (!valid_configuration_)
        {
            ROS_FATAL("invalid localization health threshold configuration");
            return;
        }

        std::vector<double> translation;
        std::vector<double> rotation;
        valid_configuration_ =
            loadVector(pnh_, "imu_to_base_translation", 3, &translation) &&
            loadVector(pnh_, "imu_to_base_rotation_xyzw", 4, &rotation);
        if (!valid_configuration_) return;
        tf2::Quaternion quaternion(rotation[0], rotation[1], rotation[2], rotation[3]);
        if (quaternion.length2() < 1e-12)
        {
            ROS_FATAL("~imu_to_base_rotation_xyzw must not be a zero quaternion");
            valid_configuration_ = false;
            return;
        }
        quaternion.normalize();
        imu_to_base_.setOrigin(tf2::Vector3(translation[0], translation[1], translation[2]));
        imu_to_base_.setRotation(quaternion);

        if (world_alignment_enabled_)
        {
            std::vector<double> world_translation;
            std::vector<double> world_rotation;
            valid_configuration_ =
                loadVector(pnh_, "initial_world_to_base_translation", 3, &world_translation) &&
                loadVector(pnh_, "initial_world_to_base_rotation_xyzw", 4, &world_rotation);
            if (!valid_configuration_) return;
            tf2::Quaternion world_quaternion(world_rotation[0], world_rotation[1],
                                             world_rotation[2], world_rotation[3]);
            if (world_quaternion.length2() < 1e-12)
            {
                ROS_FATAL("~initial_world_to_base_rotation_xyzw must not be a zero quaternion");
                valid_configuration_ = false;
                return;
            }
            world_quaternion.normalize();
            initial_world_to_base_.setOrigin(tf2::Vector3(
                world_translation[0], world_translation[1], world_translation[2]));
            initial_world_to_base_.setRotation(world_quaternion);
        }
    }

    bool observe(InputWatch* watch, const ros::Time& stamp, const std::string& rollback_reason)
    {
        const bool rollback = watch->received && !stamp.isZero() && stamp < watch->stamp;
        watch->received = true;
        watch->wall_time = ros::WallTime::now();
        watch->stamp = stamp;
        if (rollback)
        {
            invalidate(rollback_reason, true);
            return false;
        }
        return true;
    }

    void pointcloudHealthCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        observe(&pointcloud_watch_, input->header.stamp, "POINTCLOUD_TIME_ROLLBACK");
    }

    void imuHealthCallback(const sensor_msgs::Imu::ConstPtr& input)
    {
        observe(&imu_watch_, input->header.stamp, "IMU_TIME_ROLLBACK");
    }

    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& input)
    {
        observe(&clock_watch_, input->clock, "CLOCK_TIME_ROLLBACK");
    }

    bool allRequiredInputsReceived() const
    {
        return pointcloud_watch_.received && imu_watch_.received && odom_watch_.received &&
               (!monitor_clock_ || clock_watch_.received);
    }

    double age(const InputWatch& watch) const
    {
        const ros::Time now = ros::Time::now();
        if (!watch.received || watch.stamp.isZero() || now.isZero()) return INFINITY;
        return std::max(0.0, (now - watch.stamp).toSec());
    }

    double wallAge(const InputWatch& watch) const
    {
        if (!watch.received) return INFINITY;
        return (ros::WallTime::now() - watch.wall_time).toSec();
    }

    void healthTimer(const ros::TimerEvent&)
    {
        if (!health_enabled_)
        {
            publishStatus();
            return;
        }
        if (!allRequiredInputsReceived())
        {
            if (state_ != State::LOST) setState(State::WAITING_FOR_SENSORS, "WAITING_FOR_INPUTS");
            publishStatus();
            return;
        }

        const double pointcloud_age = age(pointcloud_watch_);
        const double imu_age = age(imu_watch_);
        const double odom_age = age(odom_watch_);
        const double clock_age = monitor_clock_ ? age(clock_watch_) : 0.0;
        if (pointcloud_age > sensor_lost_timeout_ || imu_age > sensor_lost_timeout_ ||
            odom_age > odom_lost_timeout_ || clock_age > clock_lost_timeout_)
        {
            invalidate("INPUT_TIMEOUT_LOST");
        }
        else if (pointcloud_age > sensor_warn_timeout_ || imu_age > sensor_warn_timeout_ ||
                 odom_age > odom_warn_timeout_ || clock_age > clock_warn_timeout_)
        {
            if (state_ == State::TRACKING || state_ == State::INITIALIZING)
            {
                consecutive_valid_ = 0;
                setState(State::DEGRADED, "INPUT_TIMEOUT_WARN");
            }
        }
        else if (!reinitialization_required_ &&
                 (state_ == State::WAITING_FOR_SENSORS || state_ == State::DEGRADED ||
                  state_ == State::LOST) && consecutive_valid_ > 0)
        {
            setState(State::INITIALIZING, "RECOVERING_VALID_SAMPLES");
        }
        publishStatus();
    }

    void invalidate(const std::string& reason, bool reinitialization_required = false)
    {
        consecutive_valid_ = 0;
        have_previous_pose_ = false;
        if (reinitialization_required) world_anchor_established_ = false;
        // Preserve the first fatal cause until the estimator process is restarted.
        // Secondary sensor timeouts after a reset must not hide CLOCK_TIME_ROLLBACK.
        if (reinitialization_required_)
        {
            return;
        }
        reinitialization_required_ = reinitialization_required_ || reinitialization_required;
        setState(State::LOST, reason);
    }

    bool poseJumped(const tf2::Transform& pose) const
    {
        if (!have_previous_pose_) return false;
        const double translation = (pose.getOrigin() - previous_pose_.getOrigin()).length();
        const double rotation = previous_pose_.getRotation().angleShortestPath(pose.getRotation());
        return translation > max_translation_jump_ || rotation > max_rotation_jump_;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& input)
    {
        if (strict_input_frames_ &&
            (input->header.frame_id != input_world_frame_ ||
             input->child_frame_id != input_body_frame_))
        {
            invalidate("ODOM_FRAME_INVALID");
            return;
        }
        if (input->header.stamp.isZero() || !finitePose(input->pose.pose))
        {
            invalidate("ODOM_NON_FINITE_OR_ZERO_STAMP");
            return;
        }
        if (!observe(&odom_watch_, input->header.stamp, "ODOM_TIME_ROLLBACK")) return;
        if (reinitialization_required_) return;

        tf2::Transform odom_to_imu;
        tf2::fromMsg(input->pose.pose, odom_to_imu);
        const tf2::Transform odom_to_base = odom_to_imu * imu_to_base_;
        if (world_alignment_enabled_ && !world_anchor_established_)
        {
            world_to_odom_ = initial_world_to_base_ * odom_to_base.inverse();
            world_anchor_established_ = true;
            ROS_INFO_STREAM("fixed-start world anchor established: world=" << world_frame_
                            << " local=" << odom_frame_);
        }
        if (poseJumped(odom_to_base))
        {
            invalidate("ODOM_POSE_JUMP", true);
            previous_pose_ = odom_to_base;
            have_previous_pose_ = true;
            return;
        }
        previous_pose_ = odom_to_base;
        have_previous_pose_ = true;

        if (health_enabled_)
        {
            if (!allRequiredInputsReceived())
            {
                setState(State::WAITING_FOR_SENSORS, "WAITING_FOR_INPUTS");
                return;
            }
            if (age(pointcloud_watch_) > sensor_warn_timeout_ ||
                age(imu_watch_) > sensor_warn_timeout_ ||
                (monitor_clock_ && age(clock_watch_) > clock_warn_timeout_))
            {
                consecutive_valid_ = 0;
                setState(State::DEGRADED, "INPUT_TIMEOUT_WARN");
                return;
            }
            ++consecutive_valid_;
            if (consecutive_valid_ < initialization_samples_)
            {
                setState(State::INITIALIZING, "COLLECTING_VALID_ODOMETRY");
                return;
            }
            setState(State::TRACKING, "HEALTHY");
        }
        publishOdometry(*input, odom_to_base);
    }

    void publishOdometry(const nav_msgs::Odometry& input, const tf2::Transform& odom_to_base)
    {
        nav_msgs::Odometry output = input;
        output.header.frame_id = odom_frame_;
        output.child_frame_id = base_frame_;
        output.pose.pose.position.x = odom_to_base.getOrigin().x();
        output.pose.pose.position.y = odom_to_base.getOrigin().y();
        output.pose.pose.position.z = odom_to_base.getOrigin().z();
        output.pose.pose.orientation = tf2::toMsg(odom_to_base.getRotation());
        output.twist.covariance.fill(0.0);
        for (std::size_t index = 0; index < 6; ++index)
            output.twist.covariance[index * 6 + index] = unknown_twist_variance_;
        // Publish the transform first. Consumers commonly receive odometry and
        // immediately query TF for the same stamp; reversing this order creates
        // a deterministic transient lookup race even within this process.
        if (publish_tf_)
        {
            if (world_alignment_enabled_ && world_anchor_established_)
            {
                geometry_msgs::TransformStamped world_transform;
                world_transform.header.stamp = output.header.stamp;
                world_transform.header.frame_id = world_frame_;
                world_transform.child_frame_id = odom_frame_;
                world_transform.transform = tf2::toMsg(world_to_odom_);
                tf_broadcaster_.sendTransform(world_transform);
            }
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = output.header.stamp;
            transform.header.frame_id = odom_frame_;
            transform.child_frame_id = base_frame_;
            transform.transform = tf2::toMsg(odom_to_base);
            tf_broadcaster_.sendTransform(transform);
        }
        odom_pub_.publish(output);
    }

    void registeredCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        if (state_ != State::TRACKING) return;
        if (strict_input_frames_ && input->header.frame_id != input_world_frame_)
        {
            invalidate("REGISTERED_CLOUD_FRAME_INVALID");
            return;
        }
        sensor_msgs::PointCloud2 output = *input;
        if (world_alignment_enabled_)
        {
            if (!transformCloud(&output)) return;
        }
        else if (rewrite_registered_cloud_frame_) output.header.frame_id = odom_frame_;
        registered_cloud_pub_.publish(output);
    }

    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        if (state_ != State::TRACKING) return;
        if (strict_input_frames_ && input->header.frame_id != input_world_frame_)
        {
            invalidate("MAP_FRAME_INVALID");
            return;
        }
        sensor_msgs::PointCloud2 output = *input;
        if (world_alignment_enabled_)
        {
            if (!transformCloud(&output)) return;
        }
        else if (rewrite_map_frame_) output.header.frame_id = odom_frame_;
        map_pub_.publish(output);
    }

    bool transformCloud(sensor_msgs::PointCloud2* cloud)
    {
        if (!world_anchor_established_)
        {
            invalidate("WORLD_ANCHOR_NOT_ESTABLISHED");
            return false;
        }
        try
        {
            sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> y(*cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> z(*cloud, "z");
            for (; x != x.end(); ++x, ++y, ++z)
            {
                const tf2::Vector3 transformed =
                    world_to_odom_ * tf2::Vector3(*x, *y, *z);
                *x = static_cast<float>(transformed.x());
                *y = static_cast<float>(transformed.y());
                *z = static_cast<float>(transformed.z());
            }
        }
        catch (const std::runtime_error& exception)
        {
            ROS_ERROR_STREAM("cannot transform localization cloud: " << exception.what());
            invalidate("CLOUD_LAYOUT_INVALID");
            return false;
        }
        cloud->header.frame_id = world_frame_;
        return true;
    }

    const char* stateName() const
    {
        switch (state_)
        {
            case State::STOPPED: return "STOPPED";
            case State::WAITING_FOR_SENSORS: return "WAITING_FOR_SENSORS";
            case State::INITIALIZING: return "INITIALIZING";
            case State::TRACKING: return "TRACKING";
            case State::DEGRADED: return "DEGRADED";
            case State::LOST: return "LOST";
        }
        return "UNKNOWN";
    }

    void setState(State state, const std::string& reason)
    {
        if (state_ == state && reason_ == reason) return;
        state_ = state;
        reason_ = reason;
        ROS_INFO_STREAM("localization state=" << stateName() << " reason=" << reason_);
        publishStatus();
    }

    void publishStatus()
    {
        if (!status_pub_) return;
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "a1_localization/health";
        status.hardware_id = "fast_lio";
        status.level = state_ == State::TRACKING ? diagnostic_msgs::DiagnosticStatus::OK :
            (state_ == State::LOST ? diagnostic_msgs::DiagnosticStatus::ERROR :
                                    diagnostic_msgs::DiagnosticStatus::WARN);
        status.message = stateName();
        status.values.push_back(keyValue("state", stateName()));
        status.values.push_back(keyValue("reason", reason_));
        status.values.push_back(keyValue("results_valid",
                                         state_ == State::TRACKING ? "true" : "false"));
        status.values.push_back(keyValue("reinitialization_required",
                                         reinitialization_required_ ? "true" : "false"));
        status.values.push_back(keyValue("world_alignment_enabled",
                                         world_alignment_enabled_ ? "true" : "false"));
        status.values.push_back(keyValue("world_anchor_established",
                                         world_anchor_established_ ? "true" : "false"));
        status.values.push_back(keyValue("map_frame",
                                         world_alignment_enabled_ ? world_frame_ : odom_frame_));
        status.values.push_back(keyValue("consecutive_valid_odometry",
                                         std::to_string(consecutive_valid_)));
        status.values.push_back(keyValue("pointcloud_age_sec", std::to_string(age(pointcloud_watch_))));
        status.values.push_back(keyValue("imu_age_sec", std::to_string(age(imu_watch_))));
        status.values.push_back(keyValue("odom_age_sec", std::to_string(age(odom_watch_))));
        if (monitor_clock_)
            status.values.push_back(keyValue("clock_age_sec", std::to_string(age(clock_watch_))));
        status.values.push_back(keyValue("pointcloud_wall_heartbeat_age_sec", std::to_string(wallAge(pointcloud_watch_))));
        status.values.push_back(keyValue("imu_wall_heartbeat_age_sec", std::to_string(wallAge(imu_watch_))));
        status.values.push_back(keyValue("odom_wall_heartbeat_age_sec", std::to_string(wallAge(odom_watch_))));
        if (monitor_clock_)
            status.values.push_back(keyValue("clock_wall_heartbeat_age_sec", std::to_string(wallAge(clock_watch_))));
        status_pub_.publish(status);
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = ros::Time::now();
        array.status.push_back(status);
        diagnostics_pub_.publish(array);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_, registered_cloud_sub_, map_sub_;
    ros::Subscriber pointcloud_health_sub_, imu_health_sub_, clock_sub_;
    ros::Publisher odom_pub_, registered_cloud_pub_, map_pub_, status_pub_, diagnostics_pub_;
    ros::Timer health_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2::Transform imu_to_base_, previous_pose_, initial_world_to_base_, world_to_odom_;
    InputWatch pointcloud_watch_, imu_watch_, odom_watch_, clock_watch_;

    std::string input_odom_topic_, output_odom_topic_;
    std::string input_registered_cloud_topic_, output_registered_cloud_topic_;
    std::string input_map_topic_, output_map_topic_, input_pointcloud_topic_, input_imu_topic_;
    std::string clock_topic_, status_topic_, diagnostics_topic_;
    std::string input_world_frame_, input_body_frame_, odom_frame_, base_frame_, world_frame_;
    bool publish_tf_{true}, strict_input_frames_{true};
    bool rewrite_registered_cloud_frame_{true}, rewrite_map_frame_{true};
    bool health_enabled_{true}, monitor_clock_{true}, valid_configuration_{false};
    bool have_previous_pose_{false};
    bool world_alignment_enabled_{true}, world_anchor_established_{false};
    bool reinitialization_required_{false};
    double unknown_twist_variance_{1.0e6};
    double sensor_warn_timeout_{0.5}, sensor_lost_timeout_{1.5};
    double odom_warn_timeout_{0.5}, odom_lost_timeout_{1.5};
    double clock_warn_timeout_{0.5}, clock_lost_timeout_{1.5};
    double max_translation_jump_{1.0}, max_rotation_jump_{1.0};
    int initialization_samples_{5};
    int consecutive_valid_{0};
    State state_{State::STOPPED};
    std::string reason_{"STARTING"};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_pose_adapter");
    LocalizationPoseAdapter adapter;
    if (!adapter.valid()) return 2;
    ros::spin();
    return 0;
}
