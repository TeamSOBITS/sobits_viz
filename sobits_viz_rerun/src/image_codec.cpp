// Copyright 2026 Team SOBITS
// SPDX-License-Identifier: BSD-3-Clause
#include "sobits_viz_rerun/image_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <utility>

#include <opencv2/imgcodecs.hpp>

namespace sobits_viz_rerun
{
namespace
{

using rerun::encodings::ChannelDatatype;
using rerun::encodings::ColorModel;

/// The header `compressed_depth_image_transport` puts in front of the PNG.
struct ConfigHeader
{
  int32_t format;
  float quant_a;
  float quant_b;
};
static_assert(sizeof(ConfigHeader) == 12, "compressedDepth header must be 12 bytes");

/// How a ROS encoding maps onto a Rerun color image.
struct ColorLayout
{
  ColorModel color_model;
  ChannelDatatype datatype;
  size_t bytes_per_pixel;
};

std::optional<ColorLayout> color_layout(const std::string & encoding)
{
  if (encoding == "rgb8") {return ColorLayout{ColorModel::RGB, ChannelDatatype::U8, 3};}
  if (encoding == "bgr8") {return ColorLayout{ColorModel::BGR, ChannelDatatype::U8, 3};}
  if (encoding == "rgba8") {return ColorLayout{ColorModel::RGBA, ChannelDatatype::U8, 4};}
  if (encoding == "bgra8") {return ColorLayout{ColorModel::BGRA, ChannelDatatype::U8, 4};}
  if (encoding == "mono8" || encoding == "8UC1") {
    return ColorLayout{ColorModel::L, ChannelDatatype::U8, 1};
  }
  if (encoding == "mono16") {return ColorLayout{ColorModel::L, ChannelDatatype::U16, 2};}
  return std::nullopt;
}

/// Copy `msg` row by row into `out`, dropping each row's padding: ROS allows
/// `step` to exceed the bytes a row needs, Rerun wants them packed.
void compact_rows(
  const sensor_msgs::msg::Image & msg, size_t row_bytes, std::vector<uint8_t> & out)
{
  out.resize(row_bytes * msg.height);
  for (size_t row = 0; row < msg.height; ++row) {
    std::memcpy(out.data() + row * row_bytes, msg.data.data() + row * msg.step, row_bytes);
  }
}

/// Swap the two bytes of every 16 bit sample, for big-endian publishers.
void swap_bytes_16(std::vector<uint8_t> & buffer)
{
  for (size_t i = 0; i + 1 < buffer.size(); i += 2) {
    std::swap(buffer[i], buffer[i + 1]);
  }
}

bool set_error(DecodeError * error, DecodeError value)
{
  if (error != nullptr) {
    *error = value;
  }
  return false;
}

/// Meters to millimeters for `DepthDatatype::kU16`: non-finite or non-positive
/// values mean "no measurement" and become 0, otherwise rounded and clamped.
uint16_t meters_to_mm(float meters)
{
  if (!std::isfinite(meters) || meters <= 0.0f) {
    return 0;
  }
  const float mm = std::round(meters * 1000.0f);
  if (mm >= 65535.0f) {return 65535;}
  return static_cast<uint16_t>(mm);
}

/// Reject images whose declared geometry does not match the bytes received.
bool has_usable_geometry(
  const sensor_msgs::msg::Image & msg, size_t row_bytes, DecodeError * error)
{
  if (msg.width == 0 || msg.height == 0) {
    return set_error(error, DecodeError::kEmptyImage);
  }
  if (msg.step < row_bytes || msg.data.size() < static_cast<size_t>(msg.step) * msg.height) {
    return set_error(error, DecodeError::kTruncatedData);
  }
  return true;
}

}  // namespace

bool is_depth_encoding(const std::string & encoding)
{
  return encoding == "16UC1" || encoding == "32FC1";
}

const char * describe(DecodeError error)
{
  switch (error) {
    case DecodeError::kUnsupportedEncoding:
      return "unsupported encoding";
    case DecodeError::kTruncatedData:
      return "image data is shorter than width, height and step imply";
    case DecodeError::kEmptyImage:
      return "image has zero width or height";
  }
  return "unknown error";
}

std::optional<ColorImage> to_rerun_image(
  const sensor_msgs::msg::Image & msg, DecodeError * error)
{
  const auto layout = color_layout(msg.encoding);
  if (!layout.has_value()) {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }

  const size_t row_bytes = static_cast<size_t>(msg.width) * layout->bytes_per_pixel;
  if (!has_usable_geometry(msg, row_bytes, error)) {
    return std::nullopt;
  }

  const rerun::WidthHeight resolution{msg.width, msg.height};
  const bool padded = msg.step != row_bytes;
  const bool needs_swap = msg.is_bigendian && layout->bytes_per_pixel > 1;

  ColorImage result{rerun::archetypes::Image(), {}};
  if (padded || needs_swap) {
    if (padded) {
      compact_rows(msg, row_bytes, result.storage);
    } else {
      result.storage.assign(msg.data.begin(), msg.data.begin() + row_bytes * msg.height);
    }
    if (needs_swap) {
      swap_bytes_16(result.storage);
    }
    result.archetype = rerun::archetypes::Image(
      rerun::Collection<uint8_t>::borrow(result.storage.data(), result.storage.size()),
      resolution, layout->color_model, layout->datatype);
  } else {
    result.archetype = rerun::archetypes::Image(
      rerun::Collection<uint8_t>::borrow(msg.data.data(), row_bytes * msg.height),
      resolution, layout->color_model, layout->datatype);
  }
  return result;
}

std::optional<DepthImage> to_rerun_depth(
  const sensor_msgs::msg::Image & msg,
  std::optional<rerun::components::Colormap> colormap,
  std::optional<std::pair<float, float>> depth_range,
  DepthDatatype datatype,
  DecodeError * error)
{
  if (!is_depth_encoding(msg.encoding)) {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }

  const bool is_float = msg.encoding == "32FC1";
  const size_t bytes_per_pixel = is_float ? 4 : 2;
  const size_t row_bytes = static_cast<size_t>(msg.width) * bytes_per_pixel;
  if (!has_usable_geometry(msg, row_bytes, error)) {
    return std::nullopt;
  }

  const rerun::WidthHeight resolution{msg.width, msg.height};
  const size_t pixel_count = static_cast<size_t>(msg.width) * msg.height;
  const bool want_u16 = datatype == DepthDatatype::kU16;

  DepthImage result{rerun::archetypes::DepthImage(), {}};

  if (is_float && !want_u16) {
    // Always copy: a simulated far clip arrives as +inf, and one such pixel
    // stretches the viewer's depth range over the whole scene.
    result.storage.resize(pixel_count * sizeof(float));
    auto * out = reinterpret_cast<float *>(result.storage.data());
    for (size_t row = 0; row < msg.height; ++row) {
      const auto * in = reinterpret_cast<const float *>(msg.data.data() + row * msg.step);
      for (size_t col = 0; col < msg.width; ++col) {
        const float value = in[col];
        out[row * msg.width + col] = std::isfinite(value) ? value : 0.0f;
      }
    }
    result.archetype = rerun::archetypes::DepthImage(
      rerun::Collection<uint8_t>::borrow(result.storage.data(), result.storage.size()),
      resolution, ChannelDatatype::F32)
      .with_meter(1.0f);
  } else if (is_float) {
    // 32FC1 in, U16 out: convert meters to millimeters into `storage`.
    result.storage.resize(pixel_count * sizeof(uint16_t));
    auto * out = reinterpret_cast<uint16_t *>(result.storage.data());
    for (size_t row = 0; row < msg.height; ++row) {
      const auto * in = reinterpret_cast<const float *>(msg.data.data() + row * msg.step);
      for (size_t col = 0; col < msg.width; ++col) {
        out[row * msg.width + col] = meters_to_mm(in[col]);
      }
    }
    result.archetype = rerun::archetypes::DepthImage(
      rerun::Collection<uint8_t>::borrow(result.storage.data(), result.storage.size()),
      resolution, ChannelDatatype::U16)
      .with_meter(1000.0f);
  } else if (want_u16) {
    // 16UC1 in, U16 out: the existing borrow/zero-copy path, unchanged.
    const bool padded = msg.step != row_bytes;
    if (padded || msg.is_bigendian) {
      if (padded) {
        compact_rows(msg, row_bytes, result.storage);
      } else {
        result.storage.assign(msg.data.begin(), msg.data.begin() + row_bytes * msg.height);
      }
      if (msg.is_bigendian) {
        swap_bytes_16(result.storage);
      }
      result.archetype = rerun::archetypes::DepthImage(
        rerun::Collection<uint8_t>::borrow(result.storage.data(), result.storage.size()),
        resolution, ChannelDatatype::U16)
        .with_meter(1000.0f);
    } else {
      result.archetype = rerun::archetypes::DepthImage(
        rerun::Collection<uint8_t>::borrow(msg.data.data(), row_bytes * msg.height),
        resolution, ChannelDatatype::U16)
        .with_meter(1000.0f);
    }
  } else {
    // 16UC1 in, F32 out: convert millimeters to meters into `storage`.
    result.storage.resize(pixel_count * sizeof(float));
    auto * out = reinterpret_cast<float *>(result.storage.data());
    for (size_t row = 0; row < msg.height; ++row) {
      auto in_row = std::vector<uint8_t>(msg.data.begin() + row * msg.step,
        msg.data.begin() + row * msg.step + row_bytes);
      if (msg.is_bigendian) {
        swap_bytes_16(in_row);
      }
      const auto * in = reinterpret_cast<const uint16_t *>(in_row.data());
      for (size_t col = 0; col < msg.width; ++col) {
        out[row * msg.width + col] = static_cast<float>(in[col]) / 1000.0f;
      }
    }
    result.archetype = rerun::archetypes::DepthImage(
      rerun::Collection<uint8_t>::borrow(result.storage.data(), result.storage.size()),
      resolution, ChannelDatatype::F32)
      .with_meter(1.0f);
  }

  if (colormap.has_value()) {
    result.archetype = std::move(result.archetype).with_colormap(colormap.value());
  }
  if (depth_range.has_value()) {
    // The range is given in metres but the viewer reads it in the pixels'
    // own units, which are millimetres for u16.
    const double scale = datatype == DepthDatatype::kU16 ? 1000.0 : 1.0;
    result.archetype = std::move(result.archetype).with_depth_range(
      rerun::components::ValueRange(std::array<double, 2>{
        depth_range->first * scale, depth_range->second * scale}));
  }
  return result;
}

std::optional<DepthImage> to_rerun_compressed_depth(
  const sensor_msgs::msg::CompressedImage & msg,
  std::optional<rerun::components::Colormap> colormap,
  std::optional<std::pair<float, float>> depth_range,
  DepthDatatype datatype,
  DecodeError * error)
{
  if (msg.format.find("compressedDepth") == std::string::npos) {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }
  // A 16UC1 source is quantized linearly and its PNG can be read as is, but
  // this bridge only ever sees the 32FC1 form, so the rest is not implemented.
  if (msg.format.find("32FC1") == std::string::npos) {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }

  ConfigHeader header{};
  if (msg.data.size() <= sizeof(header)) {
    set_error(error, DecodeError::kTruncatedData);
    return std::nullopt;
  }
  std::memcpy(&header, msg.data.data(), sizeof(header));
  if (!std::isfinite(header.quant_a) || !std::isfinite(header.quant_b) ||
    header.quant_a == 0.0f)
  {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }

  const cv::Mat payload(
    1, static_cast<int>(msg.data.size() - sizeof(header)), CV_8UC1,
    const_cast<uint8_t *>(msg.data.data() + sizeof(header)));
  const cv::Mat raw = cv::imdecode(payload, cv::IMREAD_UNCHANGED);
  if (raw.empty() || raw.type() != CV_16UC1) {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }

  const size_t pixel_count = static_cast<size_t>(raw.cols) * raw.rows;
  DepthImage result{rerun::archetypes::DepthImage(), {}};
  const rerun::WidthHeight resolution{
    static_cast<uint32_t>(raw.cols), static_cast<uint32_t>(raw.rows)};

  // Zero marks a pixel the sender could not measure, and it has to stay zero:
  // running it through the inverse would turn it into a spurious near reading.
  if (datatype == DepthDatatype::kU16) {
    result.storage.resize(pixel_count * sizeof(uint16_t));
    auto * out = reinterpret_cast<uint16_t *>(result.storage.data());
    for (int row = 0; row < raw.rows; ++row) {
      const auto * in = raw.ptr<uint16_t>(row);
      for (int col = 0; col < raw.cols; ++col) {
        const uint16_t value = in[col];
        out[static_cast<size_t>(row) * raw.cols + col] = (value == 0) ? 0 :
          meters_to_mm(header.quant_a / (static_cast<float>(value) - header.quant_b));
      }
    }
    result.archetype = rerun::archetypes::DepthImage(
      rerun::Collection<uint8_t>::borrow(result.storage.data(), result.storage.size()),
      resolution, ChannelDatatype::U16)
      .with_meter(1000.0f);
  } else {
    result.storage.resize(pixel_count * sizeof(float));
    auto * out = reinterpret_cast<float *>(result.storage.data());
    for (int row = 0; row < raw.rows; ++row) {
      const auto * in = raw.ptr<uint16_t>(row);
      for (int col = 0; col < raw.cols; ++col) {
        const uint16_t value = in[col];
        out[static_cast<size_t>(row) * raw.cols + col] =
          (value == 0) ? 0.0f : header.quant_a / (static_cast<float>(value) - header.quant_b);
      }
    }
    result.archetype = rerun::archetypes::DepthImage(
      rerun::Collection<uint8_t>::borrow(result.storage.data(), result.storage.size()),
      resolution, ChannelDatatype::F32)
      .with_meter(1.0f);
  }

  if (colormap.has_value()) {
    result.archetype = std::move(result.archetype).with_colormap(colormap.value());
  }
  if (depth_range.has_value()) {
    // The range is given in metres but the viewer reads it in the pixels'
    // own units, which are millimetres for u16.
    const double scale = datatype == DepthDatatype::kU16 ? 1000.0 : 1.0;
    result.archetype = std::move(result.archetype).with_depth_range(
      rerun::components::ValueRange(std::array<double, 2>{
        depth_range->first * scale, depth_range->second * scale}));
  }
  return result;
}

std::optional<rerun::archetypes::EncodedImage> to_rerun_encoded(
  const sensor_msgs::msg::CompressedImage & msg, DecodeError * error)
{
  if (msg.data.empty()) {
    set_error(error, DecodeError::kEmptyImage);
    return std::nullopt;
  }

  // `format` looks like "rgb8; jpeg compressed bgr8" or just "jpeg".
  const std::string & format = msg.format;
  std::string media_type;
  // `compressedDepth` also names a codec, but carries an extra ROS header
  // rather than being a plain PNG, so rule it out before looking for one.
  if (format.find("compressedDepth") != std::string::npos) {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }
  if (format.find("jpeg") != std::string::npos || format.find("jpg") != std::string::npos) {
    media_type = "image/jpeg";
  } else if (format.find("png") != std::string::npos) {
    media_type = "image/png";
  } else {
    set_error(error, DecodeError::kUnsupportedEncoding);
    return std::nullopt;
  }

  return rerun::archetypes::EncodedImage::from_bytes(
    rerun::Collection<uint8_t>::borrow(msg.data.data(), msg.data.size()), media_type);
}

}  // namespace sobits_viz_rerun
