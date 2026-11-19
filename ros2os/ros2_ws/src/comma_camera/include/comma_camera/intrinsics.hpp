// Nominal camera intrinsics of comma devices, from openpilot/common/transformations/camera.py.
#pragma once

#include <optional>
#include <string>

namespace comma_camera {

// Focal length in pixels for a camera ("narrowRoad", "wideRoad", "cabin") whose
// ISP output is width x height. openpilot models every camera as a pinhole with
// the principal point at the image centre and no distortion (the wide-road and
// cabin lenses are in fact strongly distorted fisheyes).
inline std::optional<double> nominal_focal_length(const std::string &camera, int width, int height) {
  const bool narrow = camera == "narrowRoad";
  if (width == 1928 && height == 1208) {  // OX03C10 / AR0231 (comma three, comma 3X)
    return narrow ? 2648.0 : 567.0;
  }
  if (width == 1344 && height == 760) {   // OS04C10, 2x binned (comma four)
    return narrow ? 1522.0 * 3.0 / 4.0 : 567.0 / 4.0 * 3.0;
  }
  return std::nullopt;
}

}  // namespace comma_camera
