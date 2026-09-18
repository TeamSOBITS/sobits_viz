// Copyright 2026 Team SOBITS
// SPDX-License-Identifier: BSD-3-Clause
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include "bridge_node.hpp"

namespace
{

/// Find the viewer when PATH does not have it: pip installs it into
/// `~/.local/bin`, which only login shells pick up.
std::string find_viewer_outside_path()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return {};
  }
  const std::string candidate = std::string(home) + "/.local/bin/rerun";
  std::error_code ec;
  if (std::filesystem::exists(candidate, ec)) {
    return candidate;
  }
  return {};
}

/// Start the viewer for `viewer_mode:=spawn` without attaching a sink, so the
/// caller can pair its `GrpcSink` with a file sink.
bool spawn_viewer(const rclcpp::Node & node, uint16_t port)
{
  const auto logger = node.get_logger();

  rerun::SpawnOptions options;
  options.port = port;

  // Prefer whatever is on PATH, but fall back to the pip install location so
  // the node does not depend on how the shell was started.
  const std::string fallback = find_viewer_outside_path();
  auto error = rerun::spawn(options);
  if (error.is_err() && !fallback.empty()) {
    RCLCPP_INFO(logger, "No `rerun` on PATH, using %s instead.", fallback.c_str());
    options.executable_path = fallback;
    error = rerun::spawn(options);
  }
  if (error.is_err()) {
    RCLCPP_ERROR(
      logger, "Could not spawn the Rerun viewer: %s. Is `rerun` on PATH? "
      "Install it with: python3 -m pip install --break-system-packages rerun-sdk==0.37.2",
      error.description.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "Spawned the Rerun viewer on port %u.", options.port);
  return true;
}

/// Point the stream at the viewer the mode names. Nothing is written to disk:
/// this package shows the robot, it does not archive it.
bool configure_sink(
  const rerun::RecordingStream & rec, const rclcpp::Node & node, const std::string & mode)
{
  const auto logger = node.get_logger();
  const auto port = static_cast<uint16_t>(node.get_parameter("grpc_port").as_int());

  if (mode != "spawn" && mode != "connect" && mode != "serve") {
    RCLCPP_ERROR(
      logger, "Unknown viewer_mode '%s'. Expected spawn, connect or serve.", mode.c_str());
    return false;
  }

  if (mode == "spawn" && std::getenv("DISPLAY") == nullptr) {
    RCLCPP_ERROR(
      logger, "viewer_mode 'spawn' needs a display, but DISPLAY is not set. "
      "Use viewer_mode:=web for a browser viewer.");
    return false;
  }

  rerun::Error error;
  if (mode == "serve") {
    // This buffer is what a viewer that attaches later is served from. The
    // default is far too small for several camera streams.
    const std::string buffer = node.get_parameter("server_memory_limit").as_string();
    error = rec.set_sinks(rerun::GrpcServerSink{"0.0.0.0", port, buffer});
    if (!error.is_err()) {
      RCLCPP_INFO(
        logger, "Serving data on port %u. Open it with: rerun rerun+http://<host>:%u/proxy",
        port, port);
    }
  } else {
    std::string url;
    if (mode == "spawn") {
      if (!spawn_viewer(node, port)) {
        return false;
      }
      url = "rerun+http://127.0.0.1:" + std::to_string(port) + "/proxy";
    } else {
      url = node.get_parameter("connect_url").as_string();
    }
    error = rec.set_sinks(rerun::GrpcSink{url});
    if (!error.is_err() && mode == "connect") {
      RCLCPP_INFO(logger, "Streaming to the Rerun viewer at %s.", url.c_str());
    }
  }
  if (error.is_err()) {
    RCLCPP_ERROR(logger, "Could not set up the Rerun sink: %s", error.description.c_str());
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // A short lived node reads the sink parameters before the bridge subscribes.
  // It shares the bridge's name so one parameter set reaches both.
  rclcpp::NodeOptions config_options;
  config_options.use_global_arguments(true);
  auto config = std::make_shared<rclcpp::Node>("rerun_bridge", config_options);
  const auto app_id = config->declare_parameter<std::string>("app_id", "sobit_home");
  const auto mode = config->declare_parameter<std::string>("viewer_mode", "spawn");
  config->declare_parameter<std::string>(
    "connect_url", "rerun+http://127.0.0.1:9876/proxy");
  config->declare_parameter<int>("grpc_port", 9876);
  config->declare_parameter<std::string>("server_memory_limit", "2GiB");
  const auto threads = config->declare_parameter<int>("executor_threads", 4);

  // The SDK sends an entity once its pending rows reach this many bytes. Below a
  // single row, every image, transform and joint sample ships as it is logged.
  const auto flush_bytes = config->declare_parameter<int>("flush_num_bytes", 1000);
  setenv("RERUN_FLUSH_NUM_BYTES", std::to_string(flush_bytes).c_str(), 0);

  const rerun::RecordingStream rec(app_id);
  if (!configure_sink(rec, *config, mode)) {
    rclcpp::shutdown();
    return 1;
  }

  // The configuration node shares its name with the bridge, so it is released
  // before the bridge joins the graph under that name.
  config.reset();

  std::shared_ptr<sobits_viz_rerun::RerunBridge> bridge;
  try {
    bridge = std::make_shared<sobits_viz_rerun::RerunBridge>(rec, rclcpp::NodeOptions());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("rerun_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), static_cast<size_t>(std::max<int64_t>(1, threads)));
  executor.add_node(bridge);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
