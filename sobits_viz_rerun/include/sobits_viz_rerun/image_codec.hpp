// Copyright 2026 Team SOBITS
// SPDX-License-Identifier: BSD-3-Clause
#ifndef SOBITS_VIZ_RERUN__IMAGE_CODEC_HPP_
#define SOBITS_VIZ_RERUN__IMAGE_CODEC_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rerun.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace sobits_viz_rerun
{

/// Why a conversion produced nothing, so the caller can log a useful message.
enum class DecodeError
{
  kUnsupportedEncoding,
  kTruncatedData,
  kEmptyImage,
};

/// The channel type a depth frame is logged as: 32 bit meters or 16 bit
/// millimeters, the latter halving the bytes the viewer stores per frame.
enum class DepthDatatype
{
  kF32,
  kU16,
};

/// A converted color image. Most borrow the ROS message's bytes; padded rows
/// and big-endian 16 bit data need `storage`, which outlives the archetype.
struct ColorImage
{
  rerun::archetypes::Image archetype;
  std::vector<uint8_t> storage;
};

/// A converted depth image and the scale that turns its values into meters.
struct DepthImage
{
  rerun::archetypes::DepthImage archetype;
  std::vector<uint8_t> storage;
};

/// True if `encoding` names a depth format this bridge understands.
bool is_depth_encoding(const std::string & encoding);

/// Convert a color or mono `sensor_msgs/Image`. The result may borrow from
/// `msg`, so both must outlive the `log` call that consumes it.
std::optional<ColorImage> to_rerun_image(
  const sensor_msgs::msg::Image & msg, DecodeError * error = nullptr);

/// Convert a depth `sensor_msgs/Image`, `16UC1` as millimeters and `32FC1` as
/// meters per REP 118. Non-finite pixels become 0, "no measurement".
std::optional<DepthImage> to_rerun_depth(
  const sensor_msgs::msg::Image & msg,
  std::optional<rerun::components::Colormap> colormap = std::nullopt,
  std::optional<std::pair<float, float>> depth_range = std::nullopt,
  DepthDatatype datatype = DepthDatatype::kF32,
  DecodeError * error = nullptr);

/// Convert a `compressedDepth` message: a 12 byte header then a PNG, whose
/// pixels need `depth = quant_a / (raw - quant_b)` to get back to meters.
std::optional<DepthImage> to_rerun_compressed_depth(
  const sensor_msgs::msg::CompressedImage & msg,
  std::optional<rerun::components::Colormap> colormap = std::nullopt,
  std::optional<std::pair<float, float>> depth_range = std::nullopt,
  DepthDatatype datatype = DepthDatatype::kF32,
  DecodeError * error = nullptr);

/// Hand a `sensor_msgs/CompressedImage` to the viewer undecoded. Only JPEG and
/// PNG; `compressedDepth` is rejected, having its own header.
std::optional<rerun::archetypes::EncodedImage> to_rerun_encoded(
  const sensor_msgs::msg::CompressedImage & msg, DecodeError * error = nullptr);

/// Human readable text for `error`, for log messages.
const char * describe(DecodeError error);

}  // namespace sobits_viz_rerun

#endif  // SOBITS_VIZ_RERUN__IMAGE_CODEC_HPP_
