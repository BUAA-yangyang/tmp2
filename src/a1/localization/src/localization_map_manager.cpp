#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <openssl/sha.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
bool directoryExists(const std::string& path)
{
    struct stat info {};
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool pathExists(const std::string& path)
{
    struct stat info {};
    return stat(path.c_str(), &info) == 0;
}

bool createDirectories(const std::string& path, std::string* error)
{
    if (path.empty()) return false;
    std::string current;
    if (path.front() == '/') current = "/";
    std::stringstream stream(path);
    std::string part;
    while (std::getline(stream, part, '/'))
    {
        if (part.empty()) continue;
        if (current.size() > 1) current += "/";
        current += part;
        if (!directoryExists(current) && mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
        {
            *error = "cannot create " + current + ": " + std::strerror(errno);
            return false;
        }
    }
    return true;
}

std::string sha256(const std::string& path, std::string* error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        *error = "cannot open PCD for hashing";
        return {};
    }
    SHA256_CTX context;
    SHA256_Init(&context);
    std::vector<char> buffer(1024 * 1024);
    while (input)
    {
        input.read(buffer.data(), buffer.size());
        if (input.gcount() > 0) SHA256_Update(&context, buffer.data(), input.gcount());
    }
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256_Final(digest, &context);
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (unsigned char byte : digest) output << std::setw(2) << static_cast<int>(byte);
    return output.str();
}

bool safeMapId(const std::string& value)
{
    if (value.empty() || value == "." || value == "..") return false;
    for (char character : value)
    {
        if (!(std::isalnum(static_cast<unsigned char>(character)) || character == '-' ||
              character == '_')) return false;
    }
    return true;
}
}  // namespace

class LocalizationMapManager
{
public:
    LocalizationMapManager() : nh_(), pnh_("~")
    {
        pnh_.param("input_map_topic", input_map_topic_, std::string("/a1/localization/map"));
        pnh_.param("status_topic", status_topic_, std::string("/a1/localization/status"));
        pnh_.param("save_service", save_service_, std::string("/a1/localization/save_map"));
        pnh_.param("expected_frame", expected_frame_, std::string("odom"));
        pnh_.param("output_root", output_root_, std::string("/tmp/a1_localization_maps"));
        pnh_.param("map_id", map_id_, std::string("latest"));
        pnh_.param("overwrite", overwrite_, false);
        pnh_.param("product_version", product_version_, 1);
        pnh_.param("localization_version", localization_version_, std::string("unknown"));
        pnh_.param("fast_lio_version", fast_lio_version_, std::string("unknown"));
        pnh_.param("scene_manifest_id", scene_manifest_id_, std::string("unspecified"));
        pnh_.param("input_pointcloud_topic", pointcloud_topic_, std::string());
        pnh_.param("input_imu_topic", imu_topic_, std::string());
        pnh_.param("map_resolution", map_resolution_, 0.0);
        pnh_.param("map_timeout", map_timeout_, 5.0);
        pnh_.param("minimum_points", minimum_points_, 10);

        map_sub_ = nh_.subscribe(input_map_topic_, 1, &LocalizationMapManager::mapCallback, this);
        status_sub_ = nh_.subscribe(status_topic_, 1, &LocalizationMapManager::statusCallback, this);
        save_server_ = nh_.advertiseService(save_service_, &LocalizationMapManager::save, this);
        ROS_INFO_STREAM("localization map manager ready; products root=" << output_root_);
    }

private:
    void statusCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& message)
    {
        const bool was_tracking = tracking_;
        tracking_ = message->message == "TRACKING";
        for (const auto& value : message->values)
        {
            if (value.key == "results_valid") tracking_ = tracking_ && value.value == "true";
        }
        if (was_tracking && !tracking_)
        {
            latest_map_.reset();
            ROS_WARN("discarded cached map because localization results became invalid");
        }
    }

    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
    {
        if (!tracking_) return;
        if (message->header.frame_id != expected_frame_)
        {
            ROS_ERROR_STREAM_THROTTLE(5.0, "rejecting map in frame " << message->header.frame_id
                                      << "; expected " << expected_frame_);
            return;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        try { pcl::fromROSMsg(*message, *cloud); }
        catch (const std::exception& exception)
        {
            ROS_ERROR_STREAM("cannot decode online map: " << exception.what());
            return;
        }
        if (cloud->size() < static_cast<std::size_t>(minimum_points_))
        {
            ROS_WARN_STREAM_THROTTLE(5.0, "rejecting map with only " << cloud->size() << " points");
            return;
        }
        for (const auto& point : cloud->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
                !std::isfinite(point.intensity))
            {
                ROS_ERROR_THROTTLE(5.0, "rejecting online map containing NaN or Inf");
                return;
            }
        }
        latest_map_ = cloud;
        latest_stamp_ = message->header.stamp;
        latest_wall_time_ = ros::WallTime::now();
    }

    bool save(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
    {
        pnh_.param("output_root", output_root_, output_root_);
        pnh_.param("map_id", map_id_, map_id_);
        pnh_.param("overwrite", overwrite_, overwrite_);
        if (!tracking_) return fail("localization is not TRACKING", response);
        if (!latest_map_) return fail("no valid online map is available", response);
        if ((ros::WallTime::now() - latest_wall_time_).toSec() > map_timeout_)
            return fail("online map is stale", response);
        if (!safeMapId(map_id_)) return fail("map_id may only contain letters, digits, '-' and '_'", response);

        std::string error;
        if (!createDirectories(output_root_, &error)) return fail(error, response);
        const std::string final_directory = output_root_ + "/" + map_id_;
        if (pathExists(final_directory))
        {
            if (!overwrite_) return fail("map product already exists: " + final_directory, response);
            return fail("overwrite=true is reserved; atomic replacement is not yet supported", response);
        }
        const std::string temporary_directory = output_root_ + "/." + map_id_ + ".tmp-" +
                                                std::to_string(getpid());
        if (pathExists(temporary_directory)) return fail("temporary save path already exists", response);
        if (!createDirectories(temporary_directory, &error)) return fail(error, response);

        const std::string pcd_path = temporary_directory + "/map.pcd";
        const std::string metadata_path = temporary_directory + "/metadata.yaml";
        if (pcl::io::savePCDFileBinaryCompressed(pcd_path, *latest_map_) != 0)
            return cleanupFail(temporary_directory, "failed to write PCD", response);
        pcl::PointCloud<pcl::PointXYZI> verification;
        if (pcl::io::loadPCDFile(pcd_path, verification) != 0 || verification.size() != latest_map_->size())
            return cleanupFail(temporary_directory, "saved PCD failed read-back validation", response);
        const std::string digest = sha256(pcd_path, &error);
        if (digest.empty()) return cleanupFail(temporary_directory, error, response);

        pcl::PointXYZI minimum, maximum;
        pcl::getMinMax3D(*latest_map_, minimum, maximum);
        std::ofstream metadata(metadata_path);
        if (!metadata) return cleanupFail(temporary_directory, "failed to create metadata", response);
        metadata << std::setprecision(10)
                 << "map_id: " << map_id_ << "\nproduct_version: " << product_version_
                 << "\ncreated_at_ros_sec: " << ros::Time::now().toSec()
                 << "\nsource_stamp_sec: " << latest_stamp_.toSec()
                 << "\nlocalization_version: " << localization_version_
                 << "\nfast_lio_version: " << fast_lio_version_
                 << "\nscene_manifest_id: " << scene_manifest_id_
                 << "\nframe: " << expected_frame_ << "\npoint_count: " << latest_map_->size()
                 << "\nresolution: " << map_resolution_
                 << "\nbounds:\n  min: [" << minimum.x << ", " << minimum.y << ", " << minimum.z
                 << "]\n  max: [" << maximum.x << ", " << maximum.y << ", " << maximum.z << "]"
                 << "\npcd_file: map.pcd\npcd_sha256: " << digest
                 << "\ninput_pointcloud_topic: " << pointcloud_topic_
                 << "\ninput_imu_topic: " << imu_topic_ << "\n";
        writeArrayParameter(metadata, "extrinsic_translation");
        writeArrayParameter(metadata, "extrinsic_rotation");
        metadata.close();
        if (!metadata) return cleanupFail(temporary_directory, "failed to finish metadata", response);
        if (rename(temporary_directory.c_str(), final_directory.c_str()) != 0)
            return cleanupFail(temporary_directory, "atomic product publish failed: " +
                               std::string(std::strerror(errno)), response);
        response.success = true;
        response.message = final_directory;
        ROS_INFO_STREAM("saved validated localization map product to " << final_directory);
        return true;
    }

    void writeArrayParameter(std::ofstream& output, const std::string& name)
    {
        XmlRpc::XmlRpcValue value;
        output << name << ": [";
        if (pnh_.getParam(name, value) && value.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int index = 0; index < value.size(); ++index)
            {
                if (index) output << ", ";
                if (value[index].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    output << static_cast<double>(value[index]);
                else if (value[index].getType() == XmlRpc::XmlRpcValue::TypeInt)
                    output << static_cast<int>(value[index]);
            }
        }
        output << "]\n";
    }

    bool cleanupFail(const std::string& directory, const std::string& reason,
                     std_srvs::Trigger::Response& response)
    {
        std::remove((directory + "/map.pcd").c_str());
        std::remove((directory + "/metadata.yaml").c_str());
        rmdir(directory.c_str());
        return fail(reason, response);
    }

    bool fail(const std::string& reason, std_srvs::Trigger::Response& response)
    {
        response.success = false;
        response.message = reason;
        ROS_WARN_STREAM("save_map rejected: " << reason);
        return true;
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber map_sub_, status_sub_;
    ros::ServiceServer save_server_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_map_;
    ros::Time latest_stamp_;
    ros::WallTime latest_wall_time_;
    std::string input_map_topic_, status_topic_, save_service_, expected_frame_, output_root_, map_id_;
    std::string pointcloud_topic_, imu_topic_, localization_version_, fast_lio_version_;
    std::string scene_manifest_id_;
    bool tracking_{false}, overwrite_{false};
    int product_version_{1}, minimum_points_{10};
    double map_resolution_{0.0}, map_timeout_{5.0};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_map_manager");
    LocalizationMapManager manager;
    ros::spin();
    return 0;
}
