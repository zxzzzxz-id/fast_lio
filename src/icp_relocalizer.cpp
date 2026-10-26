// Stationary prior-map registration service. Only map -> odom is corrected.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <future>
#include <thread>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fast_lio/srv/relocalize.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

namespace
{
struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const std::size_t hx = std::hash<int>{}(key.x);
    const std::size_t hy = std::hash<int>{}(key.y);
    const std::size_t hz = std::hash<int>{}(key.z);
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

VoxelKey voxel_key(const pcl::PointXYZ & point, double voxel)
{
  return {
    static_cast<int>(std::floor(point.x / voxel)),
    static_cast<int>(std::floor(point.y / voxel)),
    static_cast<int>(std::floor(point.z / voxel))};
}

double yaw_deg(const Eigen::Matrix4f & transform)
{
  return std::atan2(transform(1, 0), transform(0, 0)) * 180.0 / M_PI;
}

double wrap_deg(double angle)
{
  while (angle > 180.0) {
    angle -= 360.0;
  }
  while (angle < -180.0) {
    angle += 360.0;
  }
  return angle;
}
}  // namespace

class IcpRelocalizer : public rclcpp::Node
{
public:
  using Relocalize = fast_lio::srv::Relocalize;
  using SteadyClock = std::chrono::steady_clock;

  ~IcpRelocalizer() override
  {
    // The worker reads target clouds and the TF buffer owned by this node.
    if (worker_.valid()) {
      worker_.wait();
    }
  }

  IcpRelocalizer() : Node("icp_relocalizer")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    const double legacy_map_voxel = declare_parameter<double>("map_voxel", 0.2);
    const double legacy_source_voxel = declare_parameter<double>("source_voxel", 0.2);
    const double legacy_corr_dist =
      declare_parameter<double>("max_correspondence_distance", 1.0);
    const int legacy_iterations = declare_parameter<int>("max_iterations", 50);
    const double legacy_fitness = declare_parameter<double>("fitness_threshold", 0.3);

    coarse_map_voxel_ =
      declare_parameter<double>("coarse_map_voxel", std::max(legacy_map_voxel, 0.3));
    coarse_source_voxel_ =
      declare_parameter<double>("coarse_source_voxel", std::max(legacy_source_voxel, 0.3));
    fine_map_voxel_ =
      declare_parameter<double>("fine_map_voxel", std::min(legacy_map_voxel, 0.1));
    fine_source_voxel_ =
      declare_parameter<double>("fine_source_voxel", std::min(legacy_source_voxel, 0.1));
    z_min_ = declare_parameter<double>("z_min", -0.5);
    z_max_ = declare_parameter<double>("z_max", 1.5);
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    request_topic_ = declare_parameter<std::string>("request_topic", "/reloc/request");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/reloc/pose");

    coarse_max_correspondence_distance_ = declare_parameter<double>(
      "coarse_max_correspondence_distance", std::max(legacy_corr_dist, 1.5));
    fine_max_correspondence_distance_ = declare_parameter<double>(
      "fine_max_correspondence_distance", std::min(legacy_corr_dist, 0.5));
    coarse_max_iterations_ =
      declare_parameter<int>("coarse_max_iterations", legacy_iterations);
    fine_max_iterations_ =
      declare_parameter<int>("fine_max_iterations", std::max(legacy_iterations, 80));
    coarse_fitness_threshold_ =
      declare_parameter<double>("coarse_fitness_threshold", std::max(legacy_fitness, 0.7));
    fine_fitness_threshold_ =
      declare_parameter<double>("fine_fitness_threshold", std::min(legacy_fitness, 0.25));
    gicp_correspondence_randomness_ =
      declare_parameter<int>("gicp_correspondence_randomness", 20);

    min_source_points_ = declare_parameter<int>("min_source_points", 200);
    min_map_consistent_points_ =
      declare_parameter<int>("min_map_consistent_points", 200);
    min_correspondences_ = declare_parameter<int>("min_correspondences", 300);
    min_overlap_ratio_ = declare_parameter<double>("min_overlap_ratio", 0.35);
    overlap_distance_ = declare_parameter<double>(
      "overlap_distance", fine_max_correspondence_distance_);
    map_consistency_distance_ =
      declare_parameter<double>("map_consistency_distance", 0.5);

    startup_delay_ = declare_parameter<double>("startup_delay", 5.0);
    max_roll_pitch_deg_ = declare_parameter<double>("max_roll_pitch_deg", 10.0);
    max_correction_dist_ = declare_parameter<double>("max_correction_dist", 2.0);
    max_yaw_correction_deg_ =
      declare_parameter<double>("max_yaw_correction_deg", 35.0);
    accumulate_frames_ = declare_parameter<int>("accumulate_frames", 10);
    additional_accumulate_frames_ = declare_parameter<int>("additional_accumulate_frames", 10);
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_z_ = declare_parameter<double>("initial_z", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    one_shot_ = declare_parameter<bool>("one_shot", true);
    auto_relocalize_on_start_ =
      declare_parameter<bool>("auto_relocalize_on_start", true);
    cooldown_ = declare_parameter<double>("cooldown", 3.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 10.0);
    odom_max_age_sec_ = declare_parameter<double>("odom_max_age_sec", 0.5);
    tf_wait_sec_ = declare_parameter<double>("tf_wait_sec", 0.2);
    map_to_odom_publish_rate_ = declare_parameter<double>("map_to_odom_publish_rate", 20.0);
    if (!std::isfinite(service_timeout_sec_) || service_timeout_sec_ <= 0.0 ||
        !std::isfinite(odom_max_age_sec_) || odom_max_age_sec_ <= 0.0 ||
        !std::isfinite(tf_wait_sec_) || tf_wait_sec_ < 0.0 ||
        !std::isfinite(map_to_odom_publish_rate_) || map_to_odom_publish_rate_ <= 0.0 ||
        map_frame_.empty() || map_frame_ == odom_frame_ ||
        map_frame_ == base_frame_ || odom_frame_ == base_frame_) {
      throw std::runtime_error("Invalid service timing or frame parameters");
    }

    time_consistency_filter_en_ =
      declare_parameter<bool>("time_consistency_filter_en", true);
    time_consistency_voxel_ =
      declare_parameter<double>("time_consistency_voxel", 0.15);
    time_consistency_min_ratio_ =
      declare_parameter<double>("time_consistency_min_ratio", 0.35);
    time_consistency_min_frames_ =
      declare_parameter<int>("time_consistency_min_frames", 3);
    debug_publish_clouds_ = declare_parameter<bool>("debug_publish_clouds", true);

    if (pcd_file_.empty()) {
      throw std::runtime_error("参数 pcd_file 未设置，需要指向先验 PCD 地图");
    }
    if (z_max_ <= z_min_) {
      throw std::runtime_error("z_max must be greater than z_min");
    }
    if (accumulate_frames_ <= 0 || additional_accumulate_frames_ < 0) {
      throw std::runtime_error(
        "accumulate_frames must be positive and additional_accumulate_frames non-negative");
    }
    if (min_source_points_ <= 0 || min_map_consistent_points_ <= 0 ||
        min_correspondences_ <= 0 || gicp_correspondence_randomness_ < 3 ||
        !std::isfinite(startup_delay_) || startup_delay_ < 0.0 ||
        !std::isfinite(cooldown_) || cooldown_ < 0.0) {
      throw std::runtime_error("Invalid point-count, neighborhood or startup parameters");
    }
    if (fine_map_voxel_ <= 0.0 || fine_source_voxel_ <= 0.0 ||
        coarse_map_voxel_ <= 0.0 || coarse_source_voxel_ <= 0.0) {
      throw std::runtime_error("voxel sizes must be positive");
    }

    load_prior_map();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / map_to_odom_publish_rate_)),
      [this]() { publish_map_to_odom(); });
    RCLCPP_INFO(get_logger(), "map -> odom initialized to identity; not localized yet");

    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    if (debug_publish_clouds_) {
      debug_accumulated_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/accumulated_raw", 1);
      debug_time_filtered_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/time_filtered", 1);
      debug_coarse_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/coarse_aligned", 1);
      debug_map_filtered_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/map_consistent", 1);
      debug_fine_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/reloc/debug/fine_aligned", 1);
    }
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&IcpRelocalizer::cloud_callback, this, std::placeholders::_1));
    request_subscription_ = create_subscription<std_msgs::msg::Empty>(
      request_topic_, 10,
      std::bind(&IcpRelocalizer::request_callback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_callback(msg); });
    service_ = create_service<Relocalize>(
      "/reloc/relocalize",
      [this](std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<Relocalize::Request> request) { service_callback(header, request); });
    auto_pending_ = auto_relocalize_on_start_;
    next_auto_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(startup_delay_));
    progress_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this]() { progress(); });
    RCLCPP_INFO(
      get_logger(),
      "Service /reloc/relocalize ready (MANUAL=0, ODOMETRY=1), auto_start=%s; "
      "accumulate=%d+%d frames, timeout=%.1fs; output=%s [%s]",
      auto_pending_ ? "true" : "false", accumulate_frames_, additional_accumulate_frames_,
      service_timeout_sec_, output_topic_.c_str(), map_frame_.c_str());
  }

private:
  struct RegistrationResult
  {
    bool converged{false};
    double rmse{0.0};
    double elapsed{0.0};
    Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  };

  struct OverlapStats
  {
    int correspondences{0};
    double ratio{0.0};
  };

  struct PipelineResult
  {
    enum Outcome { ACCEPTED, NEED_MORE, REJECTED } outcome{REJECTED};
    std::string message;
    Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f odom_pose{Eigen::Matrix4f::Identity()};
    double rmse{0.0};
    OverlapStats overlap;
    builtin_interfaces::msg::Time stamp;
  };

  static Eigen::Matrix4f pose_matrix(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
    if (!rotation.coeffs().allFinite() || rotation.squaredNorm() < 1e-12 ||
        !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z)) {
      throw std::runtime_error("Invalid pose");
    }
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
    result.block<3, 1>(0, 3) = Eigen::Vector3f(
      pose.position.x, pose.position.y, pose.position.z);
    if (!result.allFinite()) {
      throw std::runtime_error("Pose exceeds representable range");
    }
    return result;
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_) {
      return;
    }
    try {
      const auto pose = pose_matrix(msg->pose.pose);
      const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
      if (stamp.nanoseconds() <= 0 || (have_odom_ && stamp <= odom_stamp_)) {
        return;
      }
      odom_pose_ = pose;
      odom_stamp_ = stamp;
      odom_received_ = SteadyClock::now();
      have_odom_ = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Ignoring odometry: %s", e.what());
    }
  }

  bool begin_job(const Relocalize::Request & request, bool automatic, std::string & error)
  {
    if (active_ || worker_.valid()) {
      error = "busy: another relocalization is collecting or computing";
      return false;
    }
    if (request.mode != Relocalize::Request::MANUAL &&
        request.mode != Relocalize::Request::ODOMETRY) {
      error = "invalid mode: expected MANUAL=0 or ODOMETRY=1";
      return false;
    }
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    if (request.mode == Relocalize::Request::MANUAL) {
      if (!std::isfinite(request.x) || !std::isfinite(request.y) ||
          !std::isfinite(request.z) || !std::isfinite(request.yaw_deg)) {
        error = "manual x/y/z/yaw_deg must be finite";
        return false;
      }
      guess.block<3, 3>(0, 0) = Eigen::AngleAxisf(
        static_cast<float>(std::remainder(request.yaw_deg, 360.0) * M_PI / 180.0),
        Eigen::Vector3f::UnitZ()).toRotationMatrix();
      guess.block<3, 1>(0, 3) = Eigen::Vector3f(request.x, request.y, request.z);
    } else {
      if (!have_odom_ || std::chrono::duration<double>(
          SteadyClock::now() - odom_received_).count() > odom_max_age_sec_ ||
          std::abs((now() - odom_stamp_).seconds()) > odom_max_age_sec_) {
        error = "missing or stale odometry (check topic, frames and use_sim_time)";
        return false;
      }
      guess = map_to_odom_ * odom_pose_;
    }
    // Preserve the configured startup-only seed offset (yaw is radians in YAML).
    if (automatic) {
      Eigen::Matrix4f offset = Eigen::Matrix4f::Identity();
      offset.block<3, 3>(0, 0) = Eigen::AngleAxisf(
        static_cast<float>(initial_yaw_), Eigen::Vector3f::UnitZ()).toRotationMatrix();
      offset.block<3, 1>(0, 3) = Eigen::Vector3f(initial_x_, initial_y_, initial_z_);
      guess = guess * offset;
    }
    if (!guess.allFinite()) {
      error = "initial pose exceeds representable range";
      return false;
    }
    reset_accumulator();
    run_initial_guess_ = guess;
    job_start_ = SteadyClock::now();
    accepted_after_ = now();
    last_cloud_stamp_ = accepted_after_;
    job_automatic_ = automatic;
    active_ = true;
    if (!automatic) {
      auto_pending_ = false;
    }
    RCLCPP_INFO(
      get_logger(), "Relocalization started: %s mode=%u xyz=(%.3f %.3f %.3f) yaw=%.1fdeg",
      automatic ? "startup" : "request", request.mode,
      guess(0, 3), guess(1, 3), guess(2, 3), yaw_deg(guess));
    return true;
  }

  void service_callback(
    const std::shared_ptr<rmw_request_id_t> & header,
    const std::shared_ptr<Relocalize::Request> & request)
  {
    std::string error;
    if (!begin_job(*request, false, error)) {
      Relocalize::Response response;
      response.message = error;
      service_->send_response(*header, response);
      return;
    }
    request_header_ = header;  // rclcpp deferred response: executor remains available.
  }

  void request_callback(const std_msgs::msg::Empty::SharedPtr)
  {
    Relocalize::Request request;
    request.mode = Relocalize::Request::ODOMETRY;
    std::string error;
    if (!begin_job(request, false, error)) {
      RCLCPP_WARN(get_logger(), "/reloc/request rejected: %s", error.c_str());
    }
  }

  void finish_job(Relocalize::Response response)
  {
    response.accumulated_frames = frame_count_;
    response.elapsed_sec = std::chrono::duration<double>(SteadyClock::now() - job_start_).count();
    RCLCPP_INFO(
      get_logger(), "%s: %s frames=%u elapsed=%.3fs",
      response.success ? "ACCEPT" : "REJECT", response.message.c_str(),
      response.accumulated_frames, response.elapsed_sec);
    if (request_header_) {
      try {
        service_->send_response(*request_header_, response);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Could not send service response: %s", e.what());
      }
      request_header_.reset();
    }
    active_ = false;
    if (job_automatic_) {
      auto_pending_ = !response.success || !one_shot_;
      next_auto_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(cooldown_));
    }
    // A timed-out worker still owns the accumulation until its result is drained.
    if (!worker_.valid()) {
      reset_accumulator();
    }
  }

  void progress()
  {
    if (active_ && std::chrono::duration<double>(SteadyClock::now() - job_start_).count()
        >= service_timeout_sec_) {
      Relocalize::Response response;
      response.message = "timeout: collection or registration exceeded service_timeout_sec";
      finish_job(response);
    }
    if (worker_.valid() && worker_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready) {
      PipelineResult result;
      try {
        result = worker_.get();
      } catch (const std::exception & e) {
        result.message = std::string("registration exception: ") + e.what();
      }
      if (!active_) {
        RCLCPP_WARN(get_logger(), "Discarding late registration result");
        reset_accumulator();
      } else if (result.outcome == PipelineResult::NEED_MORE &&
          !accumulation_extended_ && additional_accumulate_frames_ > 0) {
        accumulation_extended_ = true;
        RCLCPP_WARN(
          get_logger(), "有效累积不足: %s; 保留 %d 帧并追加 %d 帧，到 %d 帧重新检验",
          result.message.c_str(), frame_count_, additional_accumulate_frames_,
          accumulate_frames_ + additional_accumulate_frames_);
      } else {
        Relocalize::Response response;
        response.message = result.message;
        if (result.outcome == PipelineResult::ACCEPTED) {
          map_to_odom_ = result.pose * result.odom_pose.inverse();
          response.success = true;
          response.pose.header.frame_id = map_frame_;
          response.pose.header.stamp = result.stamp;
          response.pose.pose = matrix_pose(result.pose);
          response.map_to_odom = correction_transform(result.stamp);
          response.rmse = result.rmse;
          response.overlap_ratio = result.overlap.ratio;
          response.correspondences = result.overlap.correspondences;
          publisher_->publish(response.pose);
          publish_map_to_odom();
        }
        finish_job(response);
      }
    }
    if (auto_pending_ && !active_ && !worker_.valid() && SteadyClock::now() >= next_auto_) {
      Relocalize::Request request;
      request.mode = Relocalize::Request::ODOMETRY;
      std::string error;
      if (!begin_job(request, true, error)) {
        next_auto_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(std::max(cooldown_, 0.1)));
        RCLCPP_WARN(get_logger(), "Startup relocalization waiting: %s", error.c_str());
      }
    }
  }

  PointCloud::Ptr filter_z(const PointCloud::Ptr & input) const
  {
    PointCloud::Ptr z_filtered(new PointCloud());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*z_filtered);
    return z_filtered;
  }

  PointCloud::Ptr downsample(const PointCloud::Ptr & input, double voxel) const
  {
    PointCloud::Ptr downsampled(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(input);
    grid.setLeafSize(
      static_cast<float>(voxel), static_cast<float>(voxel), static_cast<float>(voxel));
    grid.filter(*downsampled);
    return downsampled;
  }

  PointCloud::Ptr filter_and_downsample(const PointCloud::Ptr & input, double voxel) const
  {
    return downsample(filter_z(input), voxel);
  }

  void load_prior_map()
  {
    RCLCPP_INFO(get_logger(), "读取先验地图 %s ...", pcd_file_.c_str());
    PointCloud::Ptr raw(new PointCloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *raw) != 0) {
      throw std::runtime_error("读不了 PCD 文件: " + pcd_file_);
    }

    target_filtered_ = filter_z(raw);
    target_coarse_ = downsample(target_filtered_, coarse_map_voxel_);
    target_fine_ = downsample(target_filtered_, fine_map_voxel_);
    if (target_coarse_->size() < 50 || target_fine_->size() < 50) {
      throw std::runtime_error(
        "先验地图在 z∈[" + std::to_string(z_min_) + ", " +
        std::to_string(z_max_) + "] 内点数不足，检查 pcd_file 和高度带");
    }
    if (static_cast<int>(target_fine_->size()) < gicp_correspondence_randomness_) {
      throw std::runtime_error("Fine target has fewer points than GICP covariance neighbors");
    }
    target_fine_tree_.setInputCloud(target_fine_);
    RCLCPP_INFO(
      get_logger(),
      "先验地图 raw=%zu z_filtered=%zu coarse=%zu(%.2fm) fine=%zu(%.2fm)",
      raw->size(), target_filtered_->size(), target_coarse_->size(), coarse_map_voxel_,
      target_fine_->size(), fine_map_voxel_);
  }

  static geometry_msgs::msg::Pose matrix_pose(const Eigen::Matrix4f & matrix)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
    q.normalize();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  geometry_msgs::msg::TransformStamped correction_transform(
    const builtin_interfaces::msg::Time & stamp) const
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    const auto pose = matrix_pose(map_to_odom_);
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    return tf;
  }

  void publish_map_to_odom()
  {
    broadcaster_->sendTransform(correction_transform(now()));
  }

  void reset_accumulator()
  {
    accumulated_raw_->clear();
    frame_clouds_.clear();
    frame_count_ = 0;
    accumulation_extended_ = false;
  }

  bool lookup(
    const std::string & target_frame, const std::string & source_frame,
    Eigen::Matrix4f & out)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      Eigen::Quaternionf q(
        static_cast<float>(tf.transform.rotation.w),
        static_cast<float>(tf.transform.rotation.x),
        static_cast<float>(tf.transform.rotation.y),
        static_cast<float>(tf.transform.rotation.z));
      out = Eigen::Matrix4f::Identity();
      out.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
      out.block<3, 1>(0, 3) = Eigen::Vector3f(
        static_cast<float>(tf.transform.translation.x),
        static_cast<float>(tf.transform.translation.y),
        static_cast<float>(tf.transform.translation.z));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "拿不到 %s <- %s 的 TF: %s",
        target_frame.c_str(), source_frame.c_str(), e.what());
      return false;
    }
  }

  PointCloud::Ptr time_consistency_filter() const
  {
    PointCloud::Ptr output(new PointCloud());
    if (!time_consistency_filter_en_ || frame_clouds_.size() < 2 ||
        time_consistency_voxel_ <= 0.0) {
      *output = *accumulated_raw_;
      return output;
    }

    std::unordered_map<VoxelKey, int, VoxelKeyHash> frame_counts;
    for (const auto & frame : frame_clouds_) {
      std::unordered_set<VoxelKey, VoxelKeyHash> seen;
      for (const auto & point : frame->points) {
        seen.insert(voxel_key(point, time_consistency_voxel_));
      }
      for (const auto & key : seen) {
        ++frame_counts[key];
      }
    }

    const int ratio_min = static_cast<int>(
      std::ceil(static_cast<double>(frame_clouds_.size()) * time_consistency_min_ratio_));
    const int min_frames = std::clamp(
      std::max(time_consistency_min_frames_, ratio_min), 1,
      static_cast<int>(frame_clouds_.size()));
    output->reserve(accumulated_raw_->size());
    for (const auto & frame : frame_clouds_) {
      for (const auto & point : frame->points) {
        const auto it = frame_counts.find(voxel_key(point, time_consistency_voxel_));
        if (it != frame_counts.end() && it->second >= min_frames) {
          output->push_back(point);
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "时间一致性过滤: %zu -> %zu 点 | voxel=%.2fm min_frames=%d/%zu",
      accumulated_raw_->size(), output->size(), time_consistency_voxel_, min_frames,
      frame_clouds_.size());
    return output;
  }

  RegistrationResult run_coarse_icp(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & guess) const
  {
    const auto started = std::chrono::steady_clock::now();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target_coarse_);
    icp.setMaxCorrespondenceDistance(coarse_max_correspondence_distance_);
    icp.setMaximumIterations(coarse_max_iterations_);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-4);

    PointCloud aligned;
    icp.align(aligned, guess);

    RegistrationResult result;
    result.converged = icp.hasConverged();
    result.transform = icp.getFinalTransformation();
    result.rmse = std::sqrt(icp.getFitnessScore(
      coarse_max_correspondence_distance_ * coarse_max_correspondence_distance_));
    result.elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return result;
  }

  RegistrationResult run_fine_gicp(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & guess) const
  {
    const auto started = std::chrono::steady_clock::now();
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target_fine_);
    gicp.setMaxCorrespondenceDistance(fine_max_correspondence_distance_);
    gicp.setMaximumIterations(fine_max_iterations_);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setEuclideanFitnessEpsilon(1e-5);
    gicp.setCorrespondenceRandomness(gicp_correspondence_randomness_);

    PointCloud aligned;
    gicp.align(aligned, guess);

    RegistrationResult result;
    result.converged = gicp.hasConverged();
    result.transform = gicp.getFinalTransformation();
    result.rmse = std::sqrt(gicp.getFitnessScore(
      fine_max_correspondence_distance_ * fine_max_correspondence_distance_));
    result.elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return result;
  }

  PointCloud::Ptr filter_by_map_consistency(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & coarse_pose) const
  {
    PointCloud::Ptr filtered(new PointCloud());
    filtered->reserve(source->size());

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    const float max_dist2 =
      static_cast<float>(map_consistency_distance_ * map_consistency_distance_);
    for (const auto & point : source->points) {
      const Eigen::Vector4f p_base(point.x, point.y, point.z, 1.0f);
      const Eigen::Vector4f p_map = coarse_pose * p_base;
      pcl::PointXYZ query(p_map.x(), p_map.y(), p_map.z());
      if (target_fine_tree_.nearestKSearch(query, 1, indices, distances) > 0 &&
          distances[0] <= max_dist2) {
        filtered->push_back(point);
      }
    }

    RCLCPP_INFO(
      get_logger(), "地图一致性过滤: %zu -> %zu 点 | distance<=%.2fm",
      source->size(), filtered->size(), map_consistency_distance_);
    return filtered;
  }

  OverlapStats compute_overlap(
    const PointCloud::Ptr & source, const Eigen::Matrix4f & transform,
    double distance) const
  {
    OverlapStats stats;
    if (source->empty()) {
      return stats;
    }

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    const float max_dist2 = static_cast<float>(distance * distance);
    for (const auto & point : source->points) {
      const Eigen::Vector4f p_base(point.x, point.y, point.z, 1.0f);
      const Eigen::Vector4f p_map = transform * p_base;
      pcl::PointXYZ query(p_map.x(), p_map.y(), p_map.z());
      if (target_fine_tree_.nearestKSearch(query, 1, indices, distances) > 0 &&
          distances[0] <= max_dist2) {
        ++stats.correspondences;
      }
    }
    stats.ratio = static_cast<double>(stats.correspondences) /
      static_cast<double>(source->size());
    return stats;
  }

  bool validate_final_result(
    const RegistrationResult & fine, const Eigen::Matrix4f & guess,
    const OverlapStats & overlap, std::string & reason) const
  {
    if (!fine.converged || !fine.transform.allFinite() || !std::isfinite(fine.rmse)) {
      RCLCPP_WARN(get_logger(), "REJECT: Fine GICP 未收敛");
      reason = "fine GICP did not converge";
      return false;
    }
    if (fine.rmse > fine_fitness_threshold_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: Fine GICP 残差 %.3fm > %.3fm",
        fine.rmse, fine_fitness_threshold_);
      reason = "fine GICP RMSE exceeds threshold";
      return false;
    }
    if (overlap.correspondences < min_correspondences_ ||
        overlap.ratio < min_overlap_ratio_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: overlap %.1f%% (%d pts) 不足，阈值 %.1f%% / %d pts",
        overlap.ratio * 100.0, overlap.correspondences,
        min_overlap_ratio_ * 100.0, min_correspondences_);
      reason = "insufficient final correspondences or overlap";
      return false;
    }

    const Eigen::Matrix3f rotation = fine.transform.block<3, 3>(0, 0);
    const double pitch = std::asin(
      std::clamp(-static_cast<double>(rotation(2, 0)), -1.0, 1.0));
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double tilt_deg = std::max(std::abs(roll), std::abs(pitch)) * 180.0 / M_PI;
    if (tilt_deg > max_roll_pitch_deg_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: roll/pitch=%.1f° > %.1f°",
        tilt_deg, max_roll_pitch_deg_);
      reason = "roll/pitch exceeds threshold";
      return false;
    }

    const double correction_dist =
      (fine.transform.block<3, 1>(0, 3) - guess.block<3, 1>(0, 3)).norm();
    const double yaw_correction = std::abs(wrap_deg(yaw_deg(fine.transform) - yaw_deg(guess)));
    if (correction_dist > max_correction_dist_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: 相对初值平移 %.2fm > %.2fm",
        correction_dist, max_correction_dist_);
      reason = "translation correction exceeds threshold";
      return false;
    }
    if (yaw_correction > max_yaw_correction_deg_) {
      RCLCPP_WARN(
        get_logger(), "REJECT: yaw 修正 %.1f° > %.1f°",
        yaw_correction, max_yaw_correction_deg_);
      reason = "yaw correction exceeds threshold";
      return false;
    }
    return true;
  }

  void publish_debug_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloud::Ptr & cloud, const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    if (!debug_publish_clouds_ || !publisher || !cloud) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    publisher->publish(msg);
  }

  void publish_aligned_debug_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloud::Ptr & cloud, const Eigen::Matrix4f & transform,
    const rclcpp::Time & stamp) const
  {
    if (!debug_publish_clouds_ || !publisher || !cloud) {
      return;
    }
    PointCloud::Ptr aligned(new PointCloud());
    pcl::transformPointCloud(*cloud, *aligned, transform);
    publish_debug_cloud(publisher, aligned, map_frame_, stamp);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!active_ || worker_.valid()) {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (stamp <= accepted_after_ || stamp <= last_cloud_stamp_) {
      return;  // Do not collect queued, duplicated or out-of-order scans.
    }
    if (msg->header.frame_id.empty()) {
      return;
    }
    PointCloud::Ptr cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *cloud);
    if (msg->header.frame_id != base_frame_) {
      Eigen::Matrix4f transform;
      if (!lookup(base_frame_, msg->header.frame_id, transform)) {
        return;
      }
      pcl::transformPointCloud(*cloud, *cloud, transform);
    }
    PointCloud::Ptr frame = filter_and_downsample(cloud, fine_source_voxel_);
    if (frame->empty()) {
      return;
    }
    last_cloud_stamp_ = stamp;
    *accumulated_raw_ += *frame;
    frame_clouds_.push_back(frame);
    ++frame_count_;
    const int target_frames = accumulate_frames_ +
      (accumulation_extended_ ? additional_accumulate_frames_ : 0);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "累积点云 %d/%d 帧（%zu 点，稳住别动）",
      frame_count_, target_frames, accumulated_raw_->size());
    if (frame_count_ >= target_frames) {
      // Accumulation is immutable until progress() drains this future.
      worker_ = std::async(std::launch::async, [this, stamp]() {
        return register_cloud(stamp);
      });
    }
  }

  Eigen::Matrix4f reference_odometry(const rclcpp::Time & stamp) const
  {
    const auto deadline = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(tf_wait_sec_));
    std::string error;
    do {
      try {
        // No ROS-clock-based timeout: /clock may be paused.
        const auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp);
        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.position.z = tf.transform.translation.z;
        pose.orientation = tf.transform.rotation;
        return pose_matrix(pose);
      } catch (const tf2::TransformException & e) {
        error = e.what();
      }
      if (!rclcpp::ok() || SteadyClock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (true);
    throw std::runtime_error("missing odom -> base_link at reference scan timestamp: " + error);
  }

  PipelineResult register_cloud(const rclcpp::Time & stamp) const
  {
    PipelineResult result;
    result.stamp = stamp;
    publish_debug_cloud(debug_accumulated_pub_, accumulated_raw_, base_frame_, stamp);
    PointCloud::Ptr stable = time_consistency_filter();
    publish_debug_cloud(debug_time_filtered_pub_, stable, base_frame_, stamp);
    PointCloud::Ptr coarse_source = downsample(stable, coarse_source_voxel_);
    PointCloud::Ptr fine_source = downsample(stable, fine_source_voxel_);
    const int min_fine = std::max(
      min_source_points_, std::max(min_map_consistent_points_, min_correspondences_));
    RCLCPP_INFO(
      get_logger(), "有效累积检验: frames=%d raw=%zu stable=%zu coarse=%zu/%d fine=%zu/%d",
      frame_count_, accumulated_raw_->size(), stable->size(), coarse_source->size(),
      min_source_points_, fine_source->size(), min_fine);
    if (static_cast<int>(stable->size()) < min_source_points_ ||
        static_cast<int>(coarse_source->size()) < min_source_points_ ||
        static_cast<int>(fine_source->size()) < min_fine) {
      result.outcome = PipelineResult::NEED_MORE;
      result.message = "insufficient effective points after temporal/voxel filtering";
      return result;
    }
    result.odom_pose = reference_odometry(stamp);
    const auto coarse = run_coarse_icp(coarse_source, run_initial_guess_);
    publish_aligned_debug_cloud(debug_coarse_pub_, coarse_source, coarse.transform, stamp);
    RCLCPP_INFO(get_logger(), "粗配准: converged=%s fitness=%.3fm time=%.3fs",
      coarse.converged ? "true" : "false", coarse.rmse, coarse.elapsed);
    if (!coarse.converged || !std::isfinite(coarse.rmse) ||
        !coarse.transform.allFinite() || coarse.rmse > coarse_fitness_threshold_) {
      result.message = "coarse ICP failed convergence or RMSE gate";
      return result;
    }
    const auto consistent = filter_by_map_consistency(fine_source, coarse.transform);
    publish_aligned_debug_cloud(debug_map_filtered_pub_, consistent, coarse.transform, stamp);
    if (static_cast<int>(consistent->size()) < min_map_consistent_points_ ||
        static_cast<int>(consistent->size()) < min_correspondences_ ||
        static_cast<int>(consistent->size()) < gicp_correspondence_randomness_) {
      result.message = "insufficient map-consistent points for GICP/final correspondences";
      return result;
    }
    const auto fine = run_fine_gicp(consistent, coarse.transform);
    const auto overlap = compute_overlap(consistent, fine.transform, overlap_distance_);
    RCLCPP_INFO(
      get_logger(), "精配准: fitness=%.3fm overlap=%.1f%%(%d/%zu) time=%.3fs",
      fine.rmse, overlap.ratio * 100.0, overlap.correspondences, consistent->size(), fine.elapsed);
    publish_aligned_debug_cloud(debug_fine_pub_, consistent, fine.transform, stamp);
    if (!validate_final_result(fine, run_initial_guess_, overlap, result.message)) {
      return result;
    }
    result.outcome = PipelineResult::ACCEPTED;
    result.message = "relocalized: map -> odom updated; local odometry remains continuous";
    result.pose = fine.transform;
    result.rmse = fine.rmse;
    result.overlap = overlap;
    return result;
  }

  std::string pcd_file_;
  double coarse_map_voxel_{0.3};
  double coarse_source_voxel_{0.3};
  double fine_map_voxel_{0.1};
  double fine_source_voxel_{0.1};
  double z_min_{-0.5};
  double z_max_{1.5};
  std::string input_topic_;
  std::string request_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string output_topic_;
  double coarse_max_correspondence_distance_{1.5};
  double fine_max_correspondence_distance_{0.5};
  int coarse_max_iterations_{50};
  int fine_max_iterations_{80};
  double coarse_fitness_threshold_{0.7};
  double fine_fitness_threshold_{0.25};
  int gicp_correspondence_randomness_{20};
  int min_source_points_{200};
  int min_map_consistent_points_{200};
  int min_correspondences_{300};
  double min_overlap_ratio_{0.35};
  double overlap_distance_{0.5};
  double map_consistency_distance_{0.5};
  int accumulate_frames_{10};
  int additional_accumulate_frames_{10};
  PointCloud::Ptr accumulated_raw_{new PointCloud()};
  std::vector<PointCloud::Ptr> frame_clouds_;
  int frame_count_{0};
  bool accumulation_extended_{false};
  double startup_delay_{5.0};
  double max_roll_pitch_deg_{10.0};
  double max_correction_dist_{2.0};
  double max_yaw_correction_deg_{35.0};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_z_{0.0};
  double initial_yaw_{0.0};
  bool one_shot_{true};
  bool auto_relocalize_on_start_{true};
  double cooldown_{3.0};

  bool time_consistency_filter_en_{true};
  double time_consistency_voxel_{0.15};
  double time_consistency_min_ratio_{0.35};
  int time_consistency_min_frames_{3};
  bool debug_publish_clouds_{true};

  PointCloud::Ptr target_filtered_;
  PointCloud::Ptr target_coarse_;
  PointCloud::Ptr target_fine_;
  pcl::KdTreeFLANN<pcl::PointXYZ> target_fine_tree_;
  SteadyClock::time_point job_start_{};
  SteadyClock::time_point next_auto_{};
  SteadyClock::time_point odom_received_{};
  Eigen::Matrix4f run_initial_guess_{Eigen::Matrix4f::Identity()};
  bool active_{false};
  bool auto_pending_{false};
  bool job_automatic_{false};
  bool have_odom_{false};
  std::string map_frame_;
  std::string odom_topic_;
  double service_timeout_sec_{10.0};
  double odom_max_age_sec_{0.5};
  double tf_wait_sec_{0.2};
  double map_to_odom_publish_rate_{20.0};
  Eigen::Matrix4f map_to_odom_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f odom_pose_{Eigen::Matrix4f::Identity()};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time accepted_after_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  std::future<PipelineResult> worker_;
  std::shared_ptr<rmw_request_id_t> request_header_;
  rclcpp::Service<Relocalize>::SharedPtr service_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr progress_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_accumulated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_time_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_coarse_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_map_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_fine_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr request_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<IcpRelocalizer>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("icp_relocalizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
