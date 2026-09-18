// Copyright 2026 Team SOBITS
// SPDX-License-Identifier: BSD-3-Clause
#include "bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "sobits_viz_rerun/frame_utils.hpp"
#include "sobits_viz_rerun/image_codec.hpp"

namespace sobits_viz_rerun
{
namespace
{

constexpr char kTimeline[] = "ros_time";

/// Map a `depth_datatype` parameter value onto the codec's enum.
std::optional<DepthDatatype> parse_depth_datatype(const std::string & name)
{
  if (name == "f32") {return DepthDatatype::kF32;}
  if (name == "u16") {return DepthDatatype::kU16;}
  return std::nullopt;
}

/// Map a colormap name from the parameters onto the Rerun enum.
std::optional<rerun::components::Colormap> parse_colormap(const std::string & name)
{
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower.empty() || lower == "none") {return std::nullopt;}
  if (lower == "grayscale" || lower == "greyscale") {
    return rerun::components::Colormap::Grayscale;
  }
  if (lower == "inferno") {return rerun::components::Colormap::Inferno;}
  if (lower == "magma") {return rerun::components::Colormap::Magma;}
  if (lower == "plasma") {return rerun::components::Colormap::Plasma;}
  if (lower == "turbo") {return rerun::components::Colormap::Turbo;}
  if (lower == "viridis") {return rerun::components::Colormap::Viridis;}
  return std::nullopt;
}

/// Report the meshes a URDF names that are not on disk, which otherwise shows
/// up only as an empty robot in the viewer.
void report_urdf_meshes(const rclcpp::Logger & logger, const std::string & urdf)
{
  const std::string token = "file://";
  size_t found = 0;
  size_t missing = 0;
  std::string first_missing;

  for (size_t pos = urdf.find(token); pos != std::string::npos; pos = urdf.find(token, pos + 1)) {
    const size_t start = pos + token.size();
    const size_t end = urdf.find_first_of("\"' ", start);
    if (end == std::string::npos) {break;}
    const std::string path = urdf.substr(start, end - start);
    // An unexpanded `$(find ...)` only survives inside XML comments, which the
    // robot never renders.
    if (path.find("$(") != std::string::npos) {
      continue;
    }
    ++found;
    if (!std::filesystem::exists(path)) {
      ++missing;
      if (first_missing.empty()) {
        first_missing = path;
      }
    }
  }

  if (missing > 0) {
    RCLCPP_WARN(
      logger, "Robot description references %zu mesh files, %zu of which are missing (e.g. %s). "
      "Source the workspace before starting the bridge.",
      found, missing, first_missing.c_str());
  } else {
    RCLCPP_INFO(logger, "Robot description references %zu mesh files, all present.", found);
  }
}

}  // namespace

RerunBridge::RerunBridge(
  const rerun::RecordingStream & rec, const rclcpp::NodeOptions & options)
: rclcpp::Node("rerun_bridge", options), rec_(rec)
{
  declare_parameters();

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  serial_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // ROS uses a right handed frame with Z pointing up, which is not Rerun's
  // default, so the whole scene is anchored to that convention once.
  rec_.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  create_subscriptions();
}

void RerunBridge::declare_parameters()
{
  robot_name_ = declare_parameter<std::string>("robot_name", "sobit_home");
  // Whether the driver's frame ids carry a prefix the URDF's do not. The
  // launch file sets both; empty means "<robot_name>/".
  enable_frame_prefix_ = declare_parameter<bool>("enable_frame_prefix", true);
  frame_prefix_ = declare_parameter<std::string>("frame_prefix", "");
  if (frame_prefix_.empty()) {
    frame_prefix_ = robot_name_ + "/";
  }

  // The launch file chooses where the expanded description is written.
  urdf_path_ = declare_parameter<std::string>("urdf_path", "");
  embed_urdf_ = declare_parameter<bool>("views.scene.embed_urdf", true);
  tf_rate_limit_hz_ = declare_parameter<double>("views.scene.tf_rate_limit_hz", 20.0);
  joint_rate_limit_hz_ = declare_parameter<double>("views.joints.rate_limit_hz", 30.0);

  // `use_sim_time` is declared by rclcpp itself when the node is constructed,
  // but reading it defensively keeps the bridge usable if that ever changes.
  if (!get_parameter("use_sim_time", sim_time_)) {
    sim_time_ = false;
  }
}

void RerunBridge::load_descriptor(std::vector<CameraSpec> & cameras)
{
  const auto path = declare_parameter<std::string>("robot_descriptor", "");
  if (path.empty()) {
    throw std::runtime_error(
            "robot_descriptor is not set. The robot is described by a sobits_vla_tools "
            ".robot.yaml, which names the cameras and topics to bridge.");
  }

  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const std::exception & error) {
    throw std::runtime_error(
            "Could not read the robot descriptor " + path + ": " + error.what());
  }

  // The entity tree and the frame prefix follow the robot's own name for itself.
  if (doc["robot_id"]) {
    robot_name_ = doc["robot_id"].as<std::string>();
    if (frame_prefix_.empty() || frame_prefix_ == "/") {
      frame_prefix_ = robot_name_ + "/";
    }
  }

  if (doc["joint_states_topic"]) {
    described_joint_states_ = doc["joint_states_topic"].as<std::string>();
  }
  if (doc["mobile_base"] && doc["mobile_base"]["odom_topic"]) {
    described_odom_ = doc["mobile_base"]["odom_topic"].as<std::string>();
  }
  if (doc["mobile_base"] && doc["mobile_base"]["command_topic"]) {
    described_base_command_ = doc["mobile_base"]["command_topic"].as<std::string>();
  }

  for (const auto & group : doc["groups"]) {
    if (group["active"] && !group["active"].as<bool>()) {
      continue;
    }
    auto & described = described_groups_[group["name"].as<std::string>()];
    for (const auto & joint : group["joints"]) {
      described.joints.push_back(joint["ros_name"].as<std::string>());
    }
    if (group["command_topic"]) {
      described.command_topic = group["command_topic"].as<std::string>();
    }
  }

  for (const auto & entry : doc["sensors"]["cameras"]) {
    // A descriptor carries every camera the robot has, including ones this
    // session is not meant to look at.
    if (entry["active"] && !entry["active"].as<bool>()) {
      continue;
    }
    CameraSpec camera;
    camera.name = entry["name"].as<std::string>();
    camera.raw_topic = entry["raw_topic"] ? entry["raw_topic"].as<std::string>() : "";
    camera.compressed_topic =
      entry["compressed_topic"] ? entry["compressed_topic"].as<std::string>() : "";
    camera.info_topic = entry["info_topic"] ? entry["info_topic"].as<std::string>() : "";
    camera.is_depth = entry["is_depth"] && entry["is_depth"].as<bool>();
    cameras.push_back(std::move(camera));
  }

  for (const auto & entry : doc["sensors"]["lidars"]) {
    if (entry["active"] && !entry["active"].as<bool>()) {
      continue;
    }
    LidarSpec lidar;
    lidar.name = entry["name"].as<std::string>();
    lidar.scan_topic = entry["scan_topic"] ? entry["scan_topic"].as<std::string>() : "";
    if (!lidar.scan_topic.empty()) {
      described_lidars_.push_back(std::move(lidar));
    }
  }

  RCLCPP_INFO(
    get_logger(), "Read %zu active cameras and %zu lidars for '%s' from %s.",
    cameras.size(), described_lidars_.size(), robot_name_.c_str(), path.c_str());
}

void RerunBridge::create_subscriptions()
{
  load_descriptor(described_cameras_);

  // The layout's switches also decide what is bridged: a view that is off has
  // no reader, so its data is not subscribed to either.
  const bool scene = declare_parameter<bool>("views.scene.enable", true);
  enable_urdf_ = scene;

  for (const auto & camera : described_cameras_) {
    const std::string key = "views.cameras." + camera.name + ".";
    const bool wanted = declare_or_get(key + "enable", true) &&
      (!camera.is_depth || declare_or_get(key + "depth.enable", true));
    if (!wanted) {
      continue;
    }
    subscribe_camera_topics(
      camera, "cameras/" + camera.name + (camera.is_depth ? "/depth" : "/color"));
  }

  // A scan is only ever drawn in the 3D view, so it goes with the scene.
  if (scene) {
    for (const auto & lidar : described_lidars_) {
      if (declare_parameter<bool>("views.lidars." + lidar.name + ".enable", true)) {
        subscribe_lidar(lidar);
      }
    }
  }

  if (scene) {
    // Transforms are the backbone of the 3D view, so they use a deeper queue
    // than the images: dropping one leaves part of the robot frozen.
    auto tf_qos = rclcpp::QoS(100).reliable();
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    subscriptions_.push_back(
      create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf_qos,
        [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {on_tf(msg, false);},
        sub_options));

    subscriptions_.push_back(
      create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
        [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {on_tf(msg, true);},
        sub_options));
  }

  resolve_joint_series();
  if (!joint_series_.empty()) {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = serial_group_;
    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::JointState>(
        described_joint_states_,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_states(msg);},
        sub_options));
    for (const auto & topic : command_topics_) {
      subscriptions_.push_back(
        create_subscription<trajectory_msgs::msg::JointTrajectory>(
          topic, rclcpp::QoS(10),
          [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
            on_joint_command(msg);
          },
          sub_options));
    }
  }

  base_position_ = declare_parameter<bool>("views.base.position", false);
  base_velocity_ = declare_parameter<bool>("views.base.velocity", true);
  base_command_ = declare_parameter<bool>("views.base.command", false);
  if (declare_parameter<bool>("views.base.enable", true) && (base_position_ || base_velocity_)) {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = serial_group_;
    subscriptions_.push_back(
      create_subscription<nav_msgs::msg::Odometry>(
        described_odom_,
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(msg);},
        sub_options));
    if (base_command_ && base_velocity_ && !described_base_command_.empty()) {
      subscriptions_.push_back(
        create_subscription<geometry_msgs::msg::Twist>(
          described_base_command_, rclcpp::QoS(10),
          [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_base_command(msg);},
          sub_options));
    }
  }

  if (scene) {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    // The one topic no descriptor carries. A leading slash means it is already
    // the full name; the node itself runs outside the robot's namespace.
    auto topic = declare_parameter<std::string>(
      "robot_description_topic", "robot_description");
    if (topic.empty() || topic.front() != '/') {
      topic = "/" + robot_name_ + "/" + topic;
    }

    // `robot_state_publisher` latches the description, so a late subscriber
    // still receives it.
    subscriptions_.push_back(
      create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {on_urdf(msg);},
        sub_options));
  }

  RCLCPP_INFO(
    get_logger(), "Bridging %zu subscriptions into Rerun for '%s'.",
    subscriptions_.size(), robot_name_.c_str());
}

void RerunBridge::resolve_joint_series()
{
  using Strings = std::vector<std::string>;
  // A tab says which series it plots; one that says nothing gets the position.
  JointSeries defaults;
  defaults.position = true;

  auto want = [this](const std::string & joint, const JointSeries & series) {
      auto & wanted = joint_series_[joint];
      wanted.position |= series.position;
      wanted.velocity |= series.velocity;
      wanted.effort |= series.effort;
      wanted.command |= series.command;
    };

  // Mirrors blueprint/make_blueprint.py: a tab plots its groups plus `add`
  // minus `exclude`. A group no tab lists is not plotted, so not logged.
  for (const auto & tab : declare_parameter<Strings>("views.joints.tabs", Strings{})) {
    if (tab.empty()) {
      continue;   // how the launch file spells "no tabs", which ROS cannot pass as []
    }
    const std::string prefix = "views.joints." + tab + ".";
    const auto groups = declare_parameter<Strings>(prefix + "groups", Strings{});
    JointSeries series;
    series.position = declare_parameter<bool>(prefix + "position", defaults.position);
    series.velocity = declare_parameter<bool>(prefix + "velocity", defaults.velocity);
    series.effort = declare_parameter<bool>(prefix + "effort", defaults.effort);
    series.command = declare_parameter<bool>(prefix + "command", false);

    const auto excluded = declare_parameter<Strings>(prefix + "exclude_joints", Strings{});
    auto keep = [&excluded](const std::string & joint) {
        return std::find(excluded.begin(), excluded.end(), joint) == excluded.end();
      };
    for (const auto & group : groups) {
      const auto found = described_groups_.find(group);
      if (found == described_groups_.end()) {
        RCLCPP_WARN(
          get_logger(), "Joint tab '%s' names group '%s', which the descriptor lacks.",
          tab.c_str(), group.c_str());
        continue;
      }
      for (const auto & joint : found->second.joints) {
        if (keep(joint)) {
          want(joint, series);
        }
      }
      const auto & topic = found->second.command_topic;
      if (series.command && !topic.empty() &&
        std::find(command_topics_.begin(), command_topics_.end(), topic) == command_topics_.end())
      {
        command_topics_.push_back(topic);
      }
    }
    for (const auto & joint : declare_parameter<Strings>(prefix + "add_joints", Strings{})) {
      if (keep(joint)) {
        want(joint, series);
      }
    }
  }
}

template<typename T>
T RerunBridge::declare_or_get(const std::string & name, const T & value)
{
  if (has_parameter(name)) {
    return get_parameter(name).get_value<T>();
  }
  return declare_parameter<T>(name, value);
}

RerunBridge::CameraSettings RerunBridge::read_camera_settings(const std::string & prefix)
{
  const CameraSettings defaults;
  CameraSettings out;
  out.depth_rate_limit_hz =
    declare_or_get(prefix + "depth.rate_limit_hz", defaults.depth_rate_limit_hz);
  out.depth_colormap = declare_or_get(prefix + "depth.colormap", defaults.depth_colormap);
  out.colormap = parse_colormap(out.depth_colormap);
  if (!out.colormap.has_value() && out.depth_colormap != "none" && !out.depth_colormap.empty()) {
    RCLCPP_WARN(
      get_logger(), "Unknown %sdepth.colormap '%s', letting the viewer choose.",
      prefix.c_str(), out.depth_colormap.c_str());
  }

  out.depth_range_m = declare_or_get(prefix + "depth.colormap_range_m", defaults.depth_range_m);
  if (out.depth_range_m.size() == 2 && out.depth_range_m[1] > out.depth_range_m[0]) {
    out.range = std::make_pair(
      static_cast<float>(out.depth_range_m[0]), static_cast<float>(out.depth_range_m[1]));
  }

  out.depth_datatype = declare_or_get(prefix + "depth.datatype", defaults.depth_datatype);
  const auto datatype = parse_depth_datatype(out.depth_datatype);
  if (datatype.has_value()) {
    out.datatype = datatype.value();
  } else {
    RCLCPP_WARN(
      get_logger(), "Unknown %sdepth.datatype '%s', falling back to f32.",
      prefix.c_str(), out.depth_datatype.c_str());
    out.datatype = DepthDatatype::kF32;
  }
  out.depth_history = declare_or_get(prefix + "depth.history", defaults.depth_history);
  out.color_history = declare_or_get(prefix + "color.history", defaults.color_history);
  out.image_plane_distance =
    declare_or_get(prefix + "frustum_size_m", defaults.image_plane_distance);
  out.color_frame = declare_or_get(prefix + "color.info_frame", defaults.color_frame);
  out.depth_frame = declare_or_get(prefix + "depth.info_frame", defaults.depth_frame);
  out.use_compressed = declare_or_get(prefix + "color.use_compressed", defaults.use_compressed);
  return out;
}

void RerunBridge::subscribe_camera_topics(
  const CameraSpec & camera, const std::string & entity_path)
{
  auto stream = std::make_unique<CameraStream>();
  stream->entity_path = entity_path + "/image";
  stream->info_entity_path = entity_path + "/camera_info";
  stream->is_depth = camera.is_depth;
  stream->settings = read_camera_settings("views.cameras." + camera.name + ".");
  if (camera.is_depth) {
    const auto & s = stream->settings;
    RCLCPP_INFO(
      get_logger(), "%s depth: %.1f Hz, %s, colormap %s over %.1f-%.1f m, %s.",
      camera.name.c_str(), s.depth_rate_limit_hz, s.depth_datatype.c_str(),
      s.depth_colormap.c_str(), s.range ? s.range->first : 0.0f, s.range ? s.range->second : 0.0f,
      s.depth_history ? "with history" : "static");
  }
  CameraStream * raw = stream.get();
  streams_.push_back(std::move(stream));

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  const auto qos = rclcpp::SensorDataQoS();

  const bool compressed = raw->settings.use_compressed && !camera.compressed_topic.empty();
  if (compressed && camera.is_depth) {
    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::CompressedImage>(
        camera.compressed_topic, qos,
        [this, raw](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
          on_compressed_depth(msg, raw);
        },
        sub_options));
  } else if (compressed) {
    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::CompressedImage>(
        camera.compressed_topic, qos,
        [this, raw](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
          on_compressed(msg, raw);
        },
        sub_options));
  } else if (!camera.raw_topic.empty()) {
    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::Image>(
        camera.raw_topic, qos,
        [this, raw](const sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(msg, raw);},
        sub_options));
  }

  if (!camera.info_topic.empty()) {
    // Colour and depth often share one camera_info, so each stream takes only
    // the messages stamped with its own frame or the pinhole flips between them.
    std::string expected = camera.is_depth ? raw->settings.depth_frame : raw->settings.color_frame;
    if (expected.empty()) {
      expected = camera.name + (camera.is_depth ? "_depth_optical_frame" : "_color_optical_frame");
    }
    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::CameraInfo>(
        camera.info_topic, qos,
        [this, raw, expected](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          if (normalize_frame(msg->header.frame_id) == expected) {
            on_camera_info(msg, raw);
          }
        },
        sub_options));
  }
}

void RerunBridge::subscribe_lidar(const LidarSpec & lidar)
{
  const std::string prefix = "views.lidars." + lidar.name + ".";
  auto stream = std::make_unique<LidarStream>();
  stream->entity_path = "lidars/" + lidar.name + "/scan";
  stream->rate_limit_hz = declare_parameter<double>(prefix + "rate_limit_hz", 10.0);
  stream->radius_m = declare_parameter<double>(prefix + "point_radius_m", 0.02);
  const auto color = declare_parameter<std::vector<int64_t>>(
    prefix + "color", std::vector<int64_t>{255, 255, 255});
  if (color.size() == 3) {
    stream->color = rerun::Color(
      static_cast<uint8_t>(color[0]), static_cast<uint8_t>(color[1]),
      static_cast<uint8_t>(color[2]));
  }
  LidarStream * raw = stream.get();
  lidar_streams_.push_back(std::move(stream));

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  subscriptions_.push_back(
    create_subscription<sensor_msgs::msg::LaserScan>(
      lidar.scan_topic, rclcpp::SensorDataQoS(),
      [this, raw](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {on_scan(msg, raw);},
      sub_options));
}

void RerunBridge::on_scan(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr msg, LidarStream * stream)
{
  if (stream->rate_limit_hz > 0.0) {
    const rclcpp::Time now(msg->header.stamp);
    std::lock_guard<std::mutex> lock(stream->rate_mutex);
    if (stream->last_logged.has_value()) {
      const double elapsed = (now - stream->last_logged.value()).seconds();
      if (elapsed >= 0.0 && elapsed < 1.0 / stream->rate_limit_hz) {
        return;
      }
    }
    stream->last_logged = now;
  }
  set_time(msg->header.stamp);

  // Each return becomes a point in the scanner's own frame; the transforms
  // place it. Out-of-range and missing returns are left out.
  std::vector<rerun::Position3D> points;
  points.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float range = msg->ranges[i];
    if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
      continue;
    }
    const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
    points.emplace_back(range * std::cos(angle), range * std::sin(angle), 0.0f);
  }
  rec_.log(
    stream->entity_path,
    rerun::Points3D(points)
    .with_colors(stream->color)
    .with_radii(static_cast<float>(stream->radius_m)),
    rerun::CoordinateFrame(normalize_frame(msg->header.frame_id)));
}

void RerunBridge::set_time(const builtin_interfaces::msg::Time & stamp)
{
  int64_t nanos =
    static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
  if (nanos == 0) {
    nanos = now().nanoseconds();
  }

  if (sim_time_) {
    // A simulated clock starts near zero, so it is shown as an elapsed
    // duration rather than a date in 1970.
    rec_.set_time_duration_nanos(kTimeline, nanos);
  } else {
    rec_.set_time_timestamp_nanos_since_epoch(kTimeline, nanos);
  }
}

std::string RerunBridge::normalize_frame(const std::string & frame) const
{
  if (!enable_frame_prefix_) {
    return frame;
  }
  return strip_frame_prefix(frame, frame_prefix_);
}

void RerunBridge::on_image(
  const sensor_msgs::msg::Image::ConstSharedPtr msg, CameraStream * stream)
{
  set_time(msg->header.stamp);
  const std::string frame = image_plane_frame(normalize_frame(msg->header.frame_id));

  DecodeError error = DecodeError::kUnsupportedEncoding;

  if (stream->is_depth || is_depth_encoding(msg->encoding)) {
    if (!depth_rate_allows(stream, msg->header.stamp)) {
      return;
    }
    const auto & settings = stream->settings;
    auto depth = to_rerun_depth(*msg, settings.colormap, settings.range, settings.datatype, &error);
    if (!depth.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping depth image on %s: %s (encoding '%s')",
        stream->entity_path.c_str(), describe(error), msg->encoding.c_str());
      return;
    }
    if (settings.depth_history) {
      rec_.log(stream->entity_path, depth->archetype, rerun::CoordinateFrame(frame));
    } else {
      rec_.log_static(stream->entity_path, depth->archetype, rerun::CoordinateFrame(frame));
    }
    return;
  }

  auto image = to_rerun_image(*msg, &error);
  if (!image.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping image on %s: %s (encoding '%s')",
      stream->entity_path.c_str(), describe(error), msg->encoding.c_str());
    return;
  }
  if (stream->settings.color_history) {
    rec_.log(stream->entity_path, image->archetype, rerun::CoordinateFrame(frame));
  } else {
    rec_.log_static(stream->entity_path, image->archetype, rerun::CoordinateFrame(frame));
  }
}

void RerunBridge::on_compressed(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, CameraStream * stream)
{
  set_time(msg->header.stamp);
  const std::string frame = image_plane_frame(normalize_frame(msg->header.frame_id));

  DecodeError error = DecodeError::kUnsupportedEncoding;
  auto encoded = to_rerun_encoded(*msg, &error);
  if (!encoded.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping compressed image on %s: %s (format '%s')",
      stream->entity_path.c_str(), describe(error), msg->format.c_str());
    return;
  }
  if (stream->settings.color_history) {
    rec_.log(stream->entity_path, encoded.value(), rerun::CoordinateFrame(frame));
  } else {
    rec_.log_static(stream->entity_path, encoded.value(), rerun::CoordinateFrame(frame));
  }
}

bool RerunBridge::depth_rate_allows(
  CameraStream * stream, const builtin_interfaces::msg::Time & stamp)
{
  const double limit_hz = stream->settings.depth_rate_limit_hz;
  if (limit_hz <= 0.0) {
    return true;
  }
  const rclcpp::Time now(stamp);
  std::lock_guard<std::mutex> lock(stream->rate_mutex);
  if (stream->last_logged.has_value()) {
    const double elapsed = (now - stream->last_logged.value()).seconds();
    // A negative gap means the clock jumped back, as on a simulation restart.
    if (elapsed >= 0.0 && elapsed < 1.0 / limit_hz) {
      return false;
    }
  }
  stream->last_logged = now;
  return true;
}

void RerunBridge::on_compressed_depth(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, CameraStream * stream)
{
  if (!depth_rate_allows(stream, msg->header.stamp)) {
    return;
  }
  set_time(msg->header.stamp);
  const std::string frame = image_plane_frame(normalize_frame(msg->header.frame_id));

  DecodeError error = DecodeError::kUnsupportedEncoding;
  const auto & settings = stream->settings;
  auto depth = to_rerun_compressed_depth(
    *msg, settings.colormap, settings.range, settings.datatype, &error);
  if (!depth.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping compressed depth on %s: %s (format '%s')",
      stream->entity_path.c_str(), describe(error), msg->format.c_str());
    return;
  }
  if (settings.depth_history) {
    rec_.log(stream->entity_path, depth->archetype, rerun::CoordinateFrame(frame));
  } else {
    rec_.log_static(stream->entity_path, depth->archetype, rerun::CoordinateFrame(frame));
  }
}

void RerunBridge::on_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, CameraStream * stream)
{
  const std::string frame = normalize_frame(msg->header.frame_id);

  {
    // Camera info arrives at the full frame rate but almost never changes, so
    // the pinhole is only logged when something about it differs.
    std::lock_guard<std::mutex> lock(stream->mutex);
    // Gazebo's focal lengths wobble in the seventh decimal, so anything this
    // far below a pixel counts as unchanged rather than a new camera.
    const auto near_enough = [](double a, double b) {
        return std::abs(a - b) <= 1e-3;
      };
    const bool unchanged = stream->last_frame.has_value() &&
      stream->last_frame.value() == frame &&
      stream->last_width == msg->width &&
      stream->last_height == msg->height &&
      std::equal(stream->last_k.begin(), stream->last_k.end(), msg->k.begin(), near_enough);
    if (unchanged) {
      return;
    }
    stream->last_frame = frame;
    stream->last_width = msg->width;
    stream->last_height = msg->height;
    std::copy(msg->k.begin(), msg->k.end(), stream->last_k.begin());
  }

  // Rerun reads a 3x3 matrix in column major order, the ROS intrinsics are
  // stored row major.
  const std::array<float, 9> image_from_camera{
    static_cast<float>(msg->k[0]), static_cast<float>(msg->k[3]), static_cast<float>(msg->k[6]),
    static_cast<float>(msg->k[1]), static_cast<float>(msg->k[4]), static_cast<float>(msg->k[7]),
    static_cast<float>(msg->k[2]), static_cast<float>(msg->k[5]), static_cast<float>(msg->k[8])};

  set_time(msg->header.stamp);
  rec_.log(
    stream->info_entity_path,
    rerun::Pinhole(image_from_camera)
    .with_resolution(static_cast<int>(msg->width), static_cast<int>(msg->height))
    .with_image_plane_distance(static_cast<float>(stream->settings.image_plane_distance))
    .with_parent_frame(frame)
    // The images logged for this camera name the same child frame, which is
    // what lets the viewer draw them inside the frustum.
    .with_child_frame(image_plane_frame(frame)));

  RCLCPP_INFO(
    get_logger(), "Logged pinhole for %s: %ux%u in frame '%s'.",
    stream->info_entity_path.c_str(), msg->width, msg->height, frame.c_str());
}

void RerunBridge::on_tf(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg, bool is_static)
{
  for (const auto & transform : msg->transforms) {
    const std::string parent = normalize_frame(transform.header.frame_id);
    const std::string child = normalize_frame(transform.child_frame_id);

    // Without a child frame this lands on the bare `tf/` path, where one entity
    // claims every frame and the viewer gives up on the whole tree.
    if (child.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring a transform from '%s' with an empty child frame.",
        transform.header.frame_id.c_str());
      continue;
    }

    // The URDF loader already registers every fixed joint, and a frame with
    // two definitions makes the viewer refuse to resolve the tree.
    if (is_static && enable_urdf_) {
      continue;
    }

    // Decide before building anything: most transforms are dropped, and the
    // archetype below is the expensive part of handling one.
    const TfEntry * entry = nullptr;
    if (!is_static) {
      const rclcpp::Time stamp(transform.header.stamp);
      std::lock_guard<std::mutex> lock(tf_rate_mutex_);
      auto it = tf_entries_.find(child);
      if (it == tf_entries_.end()) {
        it = tf_entries_.emplace(child, TfEntry{"tf/" + child, stamp}).first;
      } else if (tf_rate_limit_hz_ > 0.0) {
        const double elapsed = (stamp - it->second.last_logged).seconds();
        // A negative gap means the clock jumped back, as on a simulation restart.
        if (elapsed >= 0.0 && elapsed < 1.0 / tf_rate_limit_hz_) {
          continue;
        }
        it->second.last_logged = stamp;
      } else {
        it->second.last_logged = stamp;
      }
      entry = &it->second;
    }

    const auto archetype = rerun::Transform3D()
      .with_translation(
      {static_cast<float>(transform.transform.translation.x),
        static_cast<float>(transform.transform.translation.y),
        static_cast<float>(transform.transform.translation.z)})
      .with_quaternion(
      rerun::Quaternion::from_xyzw(
        static_cast<float>(transform.transform.rotation.x),
        static_cast<float>(transform.transform.rotation.y),
        static_cast<float>(transform.transform.rotation.z),
        static_cast<float>(transform.transform.rotation.w)))
      .with_parent_frame(parent)
      .with_child_frame(child);

    // One entity per child frame: transforms sharing a timestamp would
    // otherwise overwrite each other.
    if (is_static) {
      rec_.log_static("tf_static/" + child, archetype);
    } else {
      set_time(transform.header.stamp);
      rec_.log(entry->path, archetype);
    }
  }
}

void RerunBridge::on_joint_states(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  // The controllers publish far faster than a plot can show, and every sample
  // is kept for the whole session.
  if (joint_rate_limit_hz_ > 0.0) {
    const rclcpp::Time stamp(msg->header.stamp);
    if (joint_last_logged_.has_value()) {
      const double elapsed = (stamp - joint_last_logged_.value()).seconds();
      if (elapsed >= 0.0 && elapsed < 1.0 / joint_rate_limit_hz_) {
        return;
      }
    }
    joint_last_logged_ = stamp;
  }

  set_time(msg->header.stamp);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string & joint = msg->name[i];

    auto entry = joint_paths_.find(joint);
    if (entry == joint_paths_.end()) {
      // First sight of this joint: build its paths and name its series, which
      // is what labels them in the legend. A joint no tab plots stays empty.
      JointPaths paths;
      const auto wanted = joint_series_.find(joint);
      if (wanted != joint_series_.end()) {
        paths.series = wanted->second;
        paths.position = "joints/position/" + joint;
        paths.velocity = "joints/velocity/" + joint;
        paths.effort = "joints/effort/" + joint;
        const auto style = rerun::SeriesLines().with_names(rerun::components::Name(joint));
        if (paths.series.position) {
          rec_.log_static(paths.position, style);
        }
        if (paths.series.velocity && !msg->velocity.empty()) {
          rec_.log_static(paths.velocity, style);
        }
        if (paths.series.effort && !msg->effort.empty()) {
          rec_.log_static(paths.effort, style);
        }
      }
      entry = joint_paths_.emplace(joint, std::move(paths)).first;
    }
    const JointPaths & paths = entry->second;

    if (paths.series.position && i < msg->position.size()) {
      rec_.log(paths.position, rerun::Scalars(msg->position[i]));
    }
    if (paths.series.velocity && i < msg->velocity.size()) {
      rec_.log(paths.velocity, rerun::Scalars(msg->velocity[i]));
    }
    if (paths.series.effort && i < msg->effort.size()) {
      rec_.log(paths.effort, rerun::Scalars(msg->effort[i]));
    }
  }
}

void RerunBridge::on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  set_time(msg->header.stamp);

  const auto & position = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const auto & linear = msg->twist.twist.linear;
  const auto & angular = msg->twist.twist.angular;
  // Forward speed and yaw rate are what the base is actually commanded in;
  // the sideways component is there to show a holonomic drive slipping.
  const double speed = std::hypot(linear.x, linear.y);
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  if (!odom_series_logged_) {
    odom_series_logged_ = true;
    if (base_position_) {
      rec_.log_static("odom/position/x", rerun::SeriesLines().with_names("x (m)"));
      rec_.log_static("odom/position/y", rerun::SeriesLines().with_names("y (m)"));
      rec_.log_static("odom/position/yaw", rerun::SeriesLines().with_names("yaw (rad)"));
    }
    if (base_velocity_) {
      rec_.log_static(
        "odom/velocity/linear_x", rerun::SeriesLines().with_names("linear x (m/s)"));
      rec_.log_static(
        "odom/velocity/linear_y", rerun::SeriesLines().with_names("linear y (m/s)"));
      rec_.log_static("odom/velocity/speed", rerun::SeriesLines().with_names("speed (m/s)"));
      rec_.log_static(
        "odom/velocity/angular_z", rerun::SeriesLines().with_names("yaw rate (rad/s)"));
    }
  }

  if (base_position_) {
    rec_.log("odom/position/x", rerun::Scalars(position.x));
    rec_.log("odom/position/y", rerun::Scalars(position.y));
    rec_.log("odom/position/yaw", rerun::Scalars(yaw));
  }
  if (base_velocity_) {
    rec_.log("odom/velocity/linear_x", rerun::Scalars(linear.x));
    rec_.log("odom/velocity/linear_y", rerun::Scalars(linear.y));
    rec_.log("odom/velocity/speed", rerun::Scalars(speed));
    rec_.log("odom/velocity/angular_z", rerun::Scalars(angular.z));
  }
}

void RerunBridge::on_joint_command(
  const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg)
{
  if (msg->points.empty()) {
    return;
  }
  set_time(msg->header.stamp);
  // The last point is the target the controller is heading for.
  const auto & target = msg->points.back();

  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    const std::string & joint = msg->joint_names[i];
    const auto wanted = joint_series_.find(joint);
    if (wanted == joint_series_.end() || !wanted->second.command) {
      continue;
    }
    auto entry = command_paths_.find(joint);
    if (entry == command_paths_.end()) {
      JointPaths paths;
      paths.series = wanted->second;
      paths.position = "joints/command/position/" + joint;
      paths.velocity = "joints/command/velocity/" + joint;
      paths.effort = "joints/command/effort/" + joint;
      const auto style = rerun::SeriesLines().with_names(rerun::components::Name(joint + " cmd"));
      if (paths.series.position) {
        rec_.log_static(paths.position, style);
      }
      if (paths.series.velocity) {
        rec_.log_static(paths.velocity, style);
      }
      if (paths.series.effort) {
        rec_.log_static(paths.effort, style);
      }
      entry = command_paths_.emplace(joint, std::move(paths)).first;
    }
    const JointPaths & paths = entry->second;
    if (paths.series.position && i < target.positions.size()) {
      rec_.log(paths.position, rerun::Scalars(target.positions[i]));
    }
    if (paths.series.velocity && i < target.velocities.size()) {
      rec_.log(paths.velocity, rerun::Scalars(target.velocities[i]));
    }
    if (paths.series.effort && i < target.effort.size()) {
      rec_.log(paths.effort, rerun::Scalars(target.effort[i]));
    }
  }
}

void RerunBridge::on_base_command(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  // A Twist carries no stamp, so the command lands at the time it arrived.
  set_time(builtin_interfaces::msg::Time());
  if (!base_command_series_logged_) {
    base_command_series_logged_ = true;
    rec_.log_static("odom/command/linear_x", rerun::SeriesLines().with_names("cmd linear x (m/s)"));
    rec_.log_static("odom/command/linear_y", rerun::SeriesLines().with_names("cmd linear y (m/s)"));
    rec_.log_static("odom/command/speed", rerun::SeriesLines().with_names("cmd speed (m/s)"));
    rec_.log_static(
      "odom/command/angular_z", rerun::SeriesLines().with_names("cmd yaw rate (rad/s)"));
  }
  rec_.log("odom/command/linear_x", rerun::Scalars(msg->linear.x));
  rec_.log("odom/command/linear_y", rerun::Scalars(msg->linear.y));
  rec_.log("odom/command/speed", rerun::Scalars(std::hypot(msg->linear.x, msg->linear.y)));
  rec_.log("odom/command/angular_z", rerun::Scalars(msg->angular.z));
}

void RerunBridge::on_urdf(const std_msgs::msg::String::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(urdf_mutex_);
    if (urdf_logged_) {
      return;
    }
    urdf_logged_ = true;
  }

  report_urdf_meshes(get_logger(), msg->data);

  // Written out and imported by path: given the contents directly, Rerun
  // embeds every mesh at once instead of reading them from disk as needed.
  const std::string path = urdf_path_.empty() ?
    (std::filesystem::temp_directory_path() / "sobits_viz_rerun.urdf").string() :
    urdf_path_;

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    RCLCPP_ERROR(
      get_logger(), "Could not write the robot description to %s. The model will not appear.",
      path.c_str());
    return;
  }
  out.write(msg->data.data(), static_cast<std::streamsize>(msg->data.size()));
  out.close();

  if (embed_urdf_) {
    // No entity path prefix on purpose: nesting the model detaches its links
    // from the frames the loader sets up, and it is never drawn.
    rec_.log_file_from_path(path, std::string_view(), true);
    RCLCPP_INFO(
      get_logger(), "Logged robot description (%zu bytes) into the stream.", msg->data.size());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Wrote the robot description to %s. Open it in the viewer alongside the "
      "live data to see the model:  rerun %s", path.c_str(), path.c_str());
  }
}

}  // namespace sobits_viz_rerun
