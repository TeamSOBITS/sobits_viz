// Copyright 2026 Team SOBITS
// SPDX-License-Identifier: BSD-3-Clause
#ifndef SOBITS_VIZ_RERUN__FRAME_UTILS_HPP_
#define SOBITS_VIZ_RERUN__FRAME_UTILS_HPP_

#include <string>

namespace sobits_viz_rerun
{

/// Remove a leading `prefix` from `frame`, if present, so the URDF's bare link
/// names and a driver's namespaced ones land on the same frame.
inline std::string strip_frame_prefix(const std::string & frame, const std::string & prefix)
{
  if (prefix.empty() || frame.size() <= prefix.size()) {
    return frame;
  }
  if (frame.compare(0, prefix.size(), prefix) == 0) {
    return frame.substr(prefix.size());
  }
  return frame;
}

/// Name of the 2D image plane frame belonging to camera frame `frame`. The
/// pinhole's `child_frame` and the image's `CoordinateFrame` must agree on it.
inline std::string image_plane_frame(const std::string & frame)
{
  return frame + "_image_plane";
}

}  // namespace sobits_viz_rerun

#endif  // SOBITS_VIZ_RERUN__FRAME_UTILS_HPP_
