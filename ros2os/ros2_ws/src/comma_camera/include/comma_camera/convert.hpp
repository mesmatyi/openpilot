// NV12 frame conversion helpers (no ROS / VisionIPC dependency).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace comma_camera {

// A view of an NV12 frame as camerad produces it: full-resolution Y plane
// followed by an interleaved half-resolution UV plane, both using `stride`.
struct Nv12View {
  const uint8_t *y = nullptr;
  const uint8_t *uv = nullptr;
  int width = 0;
  int height = 0;
  int stride = 0;
};

enum class Encoding { NV12, MONO8, RGB8, BGR8 };

// Accepts the sensor_msgs/image_encodings names "nv12", "mono8", "rgb8", "bgr8"; throws std::invalid_argument.
Encoding parse_encoding(const std::string &name);
const char *encoding_name(Encoding e);

// Output geometry for a frame of width x height after `downscale` (1 or 2).
int scaled_dim(int dim, int downscale);
size_t output_step(Encoding e, int out_width);
size_t output_size(Encoding e, int out_width, int out_height);

// True if the frame can be converted with this downscale (even dimensions, /4 when halving).
bool valid_geometry(int width, int height, int downscale);

// Converts `src` into a tightly packed buffer of output_size() bytes. Colour
// conversion uses full-range BT.601 (JFIF), the convention openpilot's UI uses
// for camerad frames. downscale == 2 averages 2x2 luma blocks.
void convert(const Nv12View &src, Encoding enc, int downscale, uint8_t *dst);

// Planar I420 copy for JPEG encoding.
struct I420 {
  int width = 0, height = 0;
  std::vector<uint8_t> y, u, v;
};
void to_i420(const Nv12View &src, int downscale, I420 &out);

}  // namespace comma_camera
