// Copyright 2026 Team SOBITS
// SPDX-License-Identifier: BSD-3-Clause
#ifndef BRIDGE_NODE_HPP_
#define BRIDGE_NODE_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "sobits_viz_rerun/image_codec.hpp"

namespace sobits_viz_rerun
{

/// Republishes a robot's camera, transform and model topics into Rerun, which
/// has no ROS support of its own.
class RerunBridge : public rclcpp::Node
{
public:
  RerunBridge(const rerun::RecordingStream & rec, const rclcpp::NodeOptions & options);

private:
  /// How one camera's data is logged, read from `views.cameras.<name>.`. The
  /// values here are what a camera gets for anything its entry leaves out.
  struct CameraSettings
  {
    double depth_rate_limit_hz = 3.0;
    std::string depth_colormap = "viridis";
    std::optional<rerun::components::Colormap> colormap;
    std::vector<double> depth_range_m{0.0, 0.0};
    std::optional<std::pair<float, float>> range;
    std::string depth_datatype = "u16";
    DepthDatatype datatype = DepthDatatype::kU16;
    bool depth_history = true;
    bool color_history = true;
    double image_plane_distance = 0.3;
    /// Frame ids of this camera's camera_info; empty derives them from its name.
    std::string color_frame;
    std::string depth_frame;
    bool use_compressed = true;
  };

  /// Everything needed to log one camera stream.
  struct CameraStream
  {
    std::string entity_path;        ///< where the images go in the Rerun tree
    std::string info_entity_path;   ///< where the pinhole goes
    bool is_depth = false;
    CameraSettings settings;
    // The intrinsics rarely change, so the last ones logged are remembered and
    // a pinhole is only written again when something actually differs.
    std::optional<std::string> last_frame;
    std::array<double, 9> last_k{};
    uint32_t last_width = 0;
    uint32_t last_height = 0;
    std::mutex mutex;
    /// When this stream last passed the depth rate limit.
    std::optional<rclcpp::Time> last_logged;
    std::mutex rate_mutex;
  };

  /// One camera as a robot descriptor names it, with its topics spelled out.
  struct CameraSpec
  {
    std::string name;
    std::string raw_topic;
    std::string compressed_topic;
    std::string info_topic;
    bool is_depth = false;
  };

  /// One laser scanner as a robot descriptor names it.
  struct LidarSpec
  {
    std::string name;
    std::string scan_topic;
  };

  /// Everything needed to log one laser scanner.
  struct LidarStream
  {
    std::string entity_path;
    double rate_limit_hz = 10.0;
    double radius_m = 0.02;
    rerun::Color color{255, 255, 255};
    std::optional<rclcpp::Time> last_logged;
    std::mutex rate_mutex;
  };

  void declare_parameters();
  void create_subscriptions();
  void subscribe_lidar(const LidarSpec & lidar);
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg, LidarStream * stream);

  /// Read `robot_descriptor`, a `sobits_vla_tools` `.robot.yaml`. Throws when
  /// unset or unreadable: nothing else describes the robot.
  void load_descriptor(std::vector<CameraSpec> & cameras);

  /// Place the following log calls on the `ros_time` timeline.
  void set_time(const builtin_interfaces::msg::Time & stamp);

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg, CameraStream * stream);
  void on_compressed(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, CameraStream * stream);
  void on_compressed_depth(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, CameraStream * stream);
  void on_camera_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, CameraStream * stream);
  void on_tf(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg, bool is_static);
  void on_joint_states(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_joint_command(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg);
  void on_base_command(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void on_urdf(const std_msgs::msg::String::ConstSharedPtr msg);

  template<typename T>
  T declare_or_get(const std::string & name, const T & value);

  /// Declare a camera's settings under `prefix`, or read them back if another
  /// stream of the same camera already did.
  CameraSettings read_camera_settings(const std::string & prefix);

  /// Apply the configured frame prefix policy to a raw ROS frame id.
  std::string normalize_frame(const std::string & frame) const;

  /// True when this depth frame is within the configured rate and should be logged.
  bool depth_rate_allows(CameraStream * stream, const builtin_interfaces::msg::Time & stamp);

  /// Subscribe a camera whose topics are given outright rather than built from
  /// a base, which is how a robot descriptor names them.
  void subscribe_camera_topics(
    const CameraSpec & camera, const std::string & entity_path);

  const rerun::RecordingStream & rec_;

  std::string robot_name_;
  std::string frame_prefix_;
  bool enable_frame_prefix_ = true;
  bool sim_time_ = false;

  /// The robot as its descriptor describes it.
  std::vector<CameraSpec> described_cameras_;
  std::vector<LidarSpec> described_lidars_;
  std::vector<std::unique_ptr<LidarStream>> lidar_streams_;
  std::string described_joint_states_;
  std::string described_odom_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  /// Serialises the callbacks that keep state of their own.
  rclcpp::CallbackGroup::SharedPtr serial_group_;
  std::vector<std::unique_ptr<CameraStream>> streams_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  double tf_rate_limit_hz_ = 20.0;
  double joint_rate_limit_hz_ = 30.0;
  std::optional<rclcpp::Time> joint_last_logged_;

  /// Which of a joint's series some plot tab shows.
  struct JointSeries
  {
    bool position = false;
    bool velocity = false;
    bool effort = false;
    bool command = false;   ///< the same series from the group's command topic
  };
  /// Work out from the descriptor's groups and the `views.joints` tabs which
  /// series each joint is plotted with. Nothing else is logged.
  void resolve_joint_series();
  std::unordered_map<std::string, JointSeries> joint_series_;
  /// A joint group as the descriptor lists it.
  struct DescribedGroup
  {
    std::vector<std::string> joints;
    std::string command_topic;
  };
  std::unordered_map<std::string, DescribedGroup> described_groups_;
  /// Command topics some tab asked to plot, one subscription each.
  std::vector<std::string> command_topics_;
  std::string described_base_command_;

  /// A joint's entity paths, built once instead of on every message.
  struct JointPaths
  {
    JointSeries series;
    std::string position;
    std::string velocity;
    std::string effort;
  };
  std::unordered_map<std::string, JointPaths> joint_paths_;
  std::unordered_map<std::string, JointPaths> command_paths_;
  bool odom_series_logged_ = false;
  /// Which of the base's series the Base tab plots.
  bool base_position_ = false;
  bool base_velocity_ = true;
  bool base_command_ = false;
  bool base_command_series_logged_ = false;
  /// A frame's entity path and when it last passed the rate limit.
  struct TfEntry
  {
    std::string path;
    rclcpp::Time last_logged;
  };
  std::mutex tf_rate_mutex_;
  std::unordered_map<std::string, TfEntry> tf_entries_;
  std::string urdf_path_;
  bool embed_urdf_ = false;
  bool enable_urdf_ = true;
  bool urdf_logged_ = false;
  std::mutex urdf_mutex_;
};

}  // namespace sobits_viz_rerun

#endif  // BRIDGE_NODE_HPP_
