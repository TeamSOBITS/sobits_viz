// Copyright 2026 Team SOBITS
// SPDX-License-Identifier: BSD-3-Clause
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include "sobits_viz_rerun/frame_utils.hpp"
#include "sobits_viz_rerun/image_codec.hpp"

namespace
{

using sobits_viz_rerun::DecodeError;

/// Build an image message with `step` bytes per row, filled with a ramp.
sensor_msgs::msg::Image make_image(
  const std::string & encoding, uint32_t width, uint32_t height, size_t step)
{
  sensor_msgs::msg::Image msg;
  msg.encoding = encoding;
  msg.width = width;
  msg.height = height;
  msg.step = static_cast<uint32_t>(step);
  msg.is_bigendian = 0;
  msg.data.resize(step * height);
  for (size_t i = 0; i < msg.data.size(); ++i) {
    msg.data[i] = static_cast<uint8_t>(i % 251);
  }
  return msg;
}

}  // namespace

TEST(ImageCodec, DecodesPackedColorEncodings)
{
  for (const auto & encoding : {"rgb8", "bgr8", "mono8", "8UC1", "rgba8", "bgra8", "mono16"}) {
    const size_t bpp =
      (std::string(encoding) == "rgb8" || std::string(encoding) == "bgr8") ? 3 :
      (std::string(encoding) == "rgba8" || std::string(encoding) == "bgra8") ? 4 :
      (std::string(encoding) == "mono16") ? 2 : 1;
    auto msg = make_image(encoding, 4, 3, 4 * bpp);
    DecodeError error{};
    const auto result = sobits_viz_rerun::to_rerun_image(msg, &error);
    ASSERT_TRUE(result.has_value()) << encoding;
    // Packed rows are forwarded without an extra buffer.
    EXPECT_TRUE(result->storage.empty()) << encoding;
  }
}

TEST(ImageCodec, RepacksPaddedRows)
{
  // 4 pixels of rgb8 need 12 bytes, this publisher pads each row to 16.
  auto msg = make_image("rgb8", 4, 3, 16);
  const auto result = sobits_viz_rerun::to_rerun_image(msg);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->storage.size(), 4u * 3u * 3u);
  // The first row keeps its pixels and loses only the padding.
  EXPECT_EQ(result->storage[0], msg.data[0]);
  EXPECT_EQ(result->storage[11], msg.data[11]);
  // The second row starts where the padding of the first one was skipped.
  EXPECT_EQ(result->storage[12], msg.data[16]);
}

TEST(ImageCodec, RejectsUnknownEncoding)
{
  auto msg = make_image("yuv422_yuy2", 4, 2, 16);
  DecodeError error{};
  EXPECT_FALSE(sobits_viz_rerun::to_rerun_image(msg, &error).has_value());
  EXPECT_EQ(error, DecodeError::kUnsupportedEncoding);
}

TEST(ImageCodec, RejectsTruncatedData)
{
  auto msg = make_image("rgb8", 4, 3, 12);
  msg.data.resize(10);
  DecodeError error{};
  EXPECT_FALSE(sobits_viz_rerun::to_rerun_image(msg, &error).has_value());
  EXPECT_EQ(error, DecodeError::kTruncatedData);
}

TEST(ImageCodec, RejectsEmptyImage)
{
  auto msg = make_image("rgb8", 0, 0, 0);
  DecodeError error{};
  EXPECT_FALSE(sobits_viz_rerun::to_rerun_image(msg, &error).has_value());
  EXPECT_EQ(error, DecodeError::kEmptyImage);
}

TEST(ImageCodec, RecognizesDepthEncodings)
{
  EXPECT_TRUE(sobits_viz_rerun::is_depth_encoding("16UC1"));
  EXPECT_TRUE(sobits_viz_rerun::is_depth_encoding("32FC1"));
  EXPECT_FALSE(sobits_viz_rerun::is_depth_encoding("rgb8"));
}

TEST(ImageCodec, DecodesMillimeterDepth)
{
  auto msg = make_image("16UC1", 4, 3, 8);
  DecodeError error{};
  const auto result = sobits_viz_rerun::to_rerun_depth(
    msg, std::nullopt, std::nullopt, sobits_viz_rerun::DepthDatatype::kU16, &error);
  ASSERT_TRUE(result.has_value());
  // Packed little endian 16 bit depth is forwarded without a copy.
  EXPECT_TRUE(result->storage.empty());
}

TEST(ImageCodec, ConvertsMillimeterDepthToMeters)
{
  // 16UC1 in with the kF32 default now converts, rather than passing the
  // millimeters through as U16.
  auto msg = make_image("16UC1", 2, 1, 4);
  msg.data = {0xD2, 0x04, 0x00, 0x00};  // 1234 mm, 0 mm
  const auto result = sobits_viz_rerun::to_rerun_depth(msg);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->storage.size(), 2u * sizeof(float));
  const auto * out = reinterpret_cast<const float *>(result->storage.data());
  EXPECT_FLOAT_EQ(out[0], 1.234f);
  EXPECT_FLOAT_EQ(out[1], 0.0f);
}

TEST(ImageCodec, ReplacesNonFiniteFloatDepthWithZero)
{
  sensor_msgs::msg::Image msg;
  msg.encoding = "32FC1";
  msg.width = 3;
  msg.height = 1;
  msg.step = 12;
  msg.data.resize(12);
  const std::vector<float> pixels{
    1.5f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()};
  std::memcpy(msg.data.data(), pixels.data(), msg.data.size());

  const auto result = sobits_viz_rerun::to_rerun_depth(msg);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->storage.size(), 12u);
  const auto * out = reinterpret_cast<const float *>(result->storage.data());
  EXPECT_FLOAT_EQ(out[0], 1.5f);
  // Infinity and NaN both mean "no measurement" and become zero.
  EXPECT_FLOAT_EQ(out[1], 0.0f);
  EXPECT_FLOAT_EQ(out[2], 0.0f);
}

TEST(ImageCodec, ConvertsFloatDepthToMillimeters)
{
  sensor_msgs::msg::Image msg;
  msg.encoding = "32FC1";
  msg.width = 2;
  msg.height = 1;
  msg.step = 8;
  msg.data.resize(8);
  const std::vector<float> pixels{
    1.2345f, std::numeric_limits<float>::infinity()};
  std::memcpy(msg.data.data(), pixels.data(), msg.data.size());

  const auto result = sobits_viz_rerun::to_rerun_depth(
    msg, std::nullopt, std::nullopt, sobits_viz_rerun::DepthDatatype::kU16);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->storage.size(), 2u * sizeof(uint16_t));
  const auto * out = reinterpret_cast<const uint16_t *>(result->storage.data());
  // 1.2345 m rounds to 1235 mm; a non-finite pixel has no measurement.
  EXPECT_EQ(out[0], 1235);
  EXPECT_EQ(out[1], 0);
}

TEST(ImageCodec, SwapsBigEndianDepth)
{
  auto msg = make_image("16UC1", 2, 1, 4);
  msg.data = {0x01, 0x02, 0x03, 0x04};
  msg.is_bigendian = 1;
  const auto result = sobits_viz_rerun::to_rerun_depth(
    msg, std::nullopt, std::nullopt, sobits_viz_rerun::DepthDatatype::kU16);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->storage.size(), 4u);
  EXPECT_EQ(result->storage[0], 0x02);
  EXPECT_EQ(result->storage[1], 0x01);
}

TEST(ImageCodec, AcceptsJpegAndPngCompressedImages)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.data = {0xFF, 0xD8, 0xFF};

  msg.format = "rgb8; jpeg compressed bgr8";
  EXPECT_TRUE(sobits_viz_rerun::to_rerun_encoded(msg).has_value());

  msg.format = "png";
  EXPECT_TRUE(sobits_viz_rerun::to_rerun_encoded(msg).has_value());

  // `compressedDepth` prefixes the payload with a ROS specific header.
  DecodeError error{};
  msg.format = "16UC1; compressedDepth png";
  EXPECT_FALSE(sobits_viz_rerun::to_rerun_encoded(msg, &error).has_value());
  EXPECT_EQ(error, DecodeError::kUnsupportedEncoding);
}

TEST(FrameUtils, StripsOnlyALeadingPrefix)
{
  EXPECT_EQ(
    sobits_viz_rerun::strip_frame_prefix(
      "sobit_home/hand_left_camera_optical_frame", "sobit_home/"),
    "hand_left_camera_optical_frame");
  // Already bare frames, as the simulator and the URDF publish them.
  EXPECT_EQ(
    sobits_viz_rerun::strip_frame_prefix("head_camera_color_optical_frame", "sobit_home/"),
    "head_camera_color_optical_frame");
  // The prefix only counts at the start.
  EXPECT_EQ(
    sobits_viz_rerun::strip_frame_prefix("map/sobit_home/base", "sobit_home/"),
    "map/sobit_home/base");
  EXPECT_EQ(sobits_viz_rerun::strip_frame_prefix("base_link", ""), "base_link");
}

TEST(FrameUtils, NamesTheImagePlaneFrame)
{
  EXPECT_EQ(
    sobits_viz_rerun::image_plane_frame("head_camera_color_optical_frame"),
    "head_camera_color_optical_frame_image_plane");
}

/// Build a `compressedDepth` message from 16 bit quantized pixels.
static sensor_msgs::msg::CompressedImage make_compressed_depth(
  const std::vector<uint16_t> & pixels, int width, int height, float quant_a, float quant_b)
{
  const cv::Mat raw(height, width, CV_16UC1, const_cast<uint16_t *>(pixels.data()));
  std::vector<uint8_t> png;
  cv::imencode(".png", raw, png);

  sensor_msgs::msg::CompressedImage msg;
  msg.format = "32FC1; compressedDepth png";
  msg.data.resize(12 + png.size());
  const int32_t format = 0;
  std::memcpy(msg.data.data() + 0, &format, 4);
  std::memcpy(msg.data.data() + 4, &quant_a, 4);
  std::memcpy(msg.data.data() + 8, &quant_b, 4);
  std::memcpy(msg.data.data() + 12, png.data(), png.size());
  return msg;
}

TEST(ImageCodec, DecodesCompressedDepthToMeters)
{
  // depth = quant_a / (raw - quant_b), the inverse the ROS transport applies.
  const float quant_a = 10100.0f;
  const float quant_b = -1009.0f;
  const std::vector<uint16_t> pixels{2000, 4000, 0};
  const auto msg = make_compressed_depth(pixels, 3, 1, quant_a, quant_b);

  DecodeError error{};
  const auto result = sobits_viz_rerun::to_rerun_compressed_depth(
    msg, std::nullopt, std::nullopt, sobits_viz_rerun::DepthDatatype::kF32, &error);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->storage.size(), 3u * sizeof(float));

  const auto * out = reinterpret_cast<const float *>(result->storage.data());
  EXPECT_NEAR(out[0], quant_a / (2000.0f - quant_b), 1e-3f);
  EXPECT_NEAR(out[1], quant_a / (4000.0f - quant_b), 1e-3f);
  // Zero means the sender had no measurement, and must not become a near hit.
  EXPECT_FLOAT_EQ(out[2], 0.0f);
}

TEST(ImageCodec, DecodesCompressedDepthToMillimeters)
{
  const float quant_a = 10100.0f;
  const float quant_b = -1009.0f;
  const std::vector<uint16_t> pixels{2000, 4000, 0};
  const auto msg = make_compressed_depth(pixels, 3, 1, quant_a, quant_b);

  DecodeError error{};
  const auto result = sobits_viz_rerun::to_rerun_compressed_depth(
    msg, std::nullopt, std::nullopt, sobits_viz_rerun::DepthDatatype::kU16, &error);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->storage.size(), 3u * sizeof(uint16_t));

  const auto * out = reinterpret_cast<const uint16_t *>(result->storage.data());
  const auto expect_mm = [](float raw, float a, float b) {
      return static_cast<uint16_t>(std::round(a / (raw - b) * 1000.0f));
    };
  EXPECT_EQ(out[0], expect_mm(2000.0f, quant_a, quant_b));
  EXPECT_EQ(out[1], expect_mm(4000.0f, quant_a, quant_b));
  EXPECT_EQ(out[2], 0);
}

TEST(ImageCodec, RejectsCompressedDepthWithoutHeader)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.format = "32FC1; compressedDepth png";
  msg.data.resize(8);

  DecodeError error{};
  EXPECT_FALSE(
    sobits_viz_rerun::to_rerun_compressed_depth(
      msg, std::nullopt, std::nullopt, sobits_viz_rerun::DepthDatatype::kF32, &error)
    .has_value());
  EXPECT_EQ(error, DecodeError::kTruncatedData);
}

TEST(ImageCodec, RejectsPlainCompressedAsDepth)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.format = "rgb8; jpeg compressed bgr8";
  msg.data.resize(64, 1);

  DecodeError error{};
  EXPECT_FALSE(
    sobits_viz_rerun::to_rerun_compressed_depth(
      msg, std::nullopt, std::nullopt, sobits_viz_rerun::DepthDatatype::kF32, &error)
    .has_value());
  EXPECT_EQ(error, DecodeError::kUnsupportedEncoding);
}
