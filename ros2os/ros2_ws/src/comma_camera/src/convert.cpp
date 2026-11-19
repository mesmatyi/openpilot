#include "comma_camera/convert.hpp"

#include <algorithm>
#include <stdexcept>

namespace comma_camera {

Encoding parse_encoding(const std::string &name) {
  if (name == "nv12") return Encoding::NV12;
  if (name == "mono8") return Encoding::MONO8;
  if (name == "rgb8") return Encoding::RGB8;
  if (name == "bgr8") return Encoding::BGR8;
  throw std::invalid_argument("unsupported encoding '" + name + "' (use nv12, mono8, rgb8 or bgr8)");
}

const char *encoding_name(Encoding e) {
  switch (e) {
    case Encoding::NV12: return "nv12";
    case Encoding::MONO8: return "mono8";
    case Encoding::RGB8: return "rgb8";
    case Encoding::BGR8: return "bgr8";
  }
  return "";
}

int scaled_dim(int dim, int downscale) { return dim / downscale; }

size_t output_step(Encoding e, int w) {
  return (e == Encoding::RGB8 || e == Encoding::BGR8) ? static_cast<size_t>(w) * 3 : static_cast<size_t>(w);
}

size_t output_size(Encoding e, int w, int h) {
  if (e == Encoding::NV12) return static_cast<size_t>(w) * h * 3 / 2;
  return output_step(e, w) * h;
}

bool valid_geometry(int width, int height, int downscale) {
  if (downscale != 1 && downscale != 2) return false;
  const int m = 2 * downscale;
  return width > 0 && height > 0 && width % m == 0 && height % m == 0;
}

namespace {

inline uint8_t clamp8(int v) { return static_cast<uint8_t>(std::min(255, std::max(0, v))); }

// full-range BT.601 in Q10: R = Y + 1.402 V, G = Y - 0.344136 U - 0.714136 V, B = Y + 1.772 U
inline void yuv_to_rgb(int y, int u, int v, uint8_t &r, uint8_t &g, uint8_t &b) {
  u -= 128;
  v -= 128;
  const int y10 = y << 10;
  r = clamp8((y10 + 1436 * v + 512) >> 10);
  g = clamp8((y10 - 352 * u - 731 * v + 512) >> 10);
  b = clamp8((y10 + 1815 * u + 512) >> 10);
}

inline int luma(const Nv12View &s, int x, int y, int ds) {
  if (ds == 1) return s.y[y * s.stride + x];
  const uint8_t *r0 = s.y + (2 * y) * s.stride + 2 * x;
  const uint8_t *r1 = r0 + s.stride;
  return (r0[0] + r0[1] + r1[0] + r1[1] + 2) >> 2;
}

// chroma sample covering output pixel (x, y)
inline const uint8_t *chroma(const Nv12View &s, int x, int y, int ds) {
  const int cx = (x * ds) / 2, cy = (y * ds) / 2;
  return s.uv + cy * s.stride + 2 * cx;
}

}  // namespace

void convert(const Nv12View &s, Encoding enc, int ds, uint8_t *dst) {
  const int w = scaled_dim(s.width, ds), h = scaled_dim(s.height, ds);
  switch (enc) {
    case Encoding::NV12: {
      for (int y = 0; y < h; y++) {
        uint8_t *row = dst + static_cast<size_t>(y) * w;
        if (ds == 1) {
          std::copy_n(s.y + static_cast<size_t>(y) * s.stride, w, row);
        } else {
          for (int x = 0; x < w; x++) row[x] = static_cast<uint8_t>(luma(s, x, y, ds));
        }
      }
      uint8_t *uv_out = dst + static_cast<size_t>(w) * h;
      for (int y = 0; y < h / 2; y++) {
        uint8_t *row = uv_out + static_cast<size_t>(y) * w;
        const uint8_t *src = s.uv + static_cast<size_t>(y * ds) * s.stride;
        if (ds == 1) {
          std::copy_n(src, w, row);
        } else {
          for (int x = 0; x < w / 2; x++) {
            row[2 * x] = src[4 * x];
            row[2 * x + 1] = src[4 * x + 1];
          }
        }
      }
      break;
    }
    case Encoding::MONO8:
      for (int y = 0; y < h; y++) {
        uint8_t *row = dst + static_cast<size_t>(y) * w;
        if (ds == 1) {
          std::copy_n(s.y + static_cast<size_t>(y) * s.stride, w, row);
        } else {
          for (int x = 0; x < w; x++) row[x] = static_cast<uint8_t>(luma(s, x, y, ds));
        }
      }
      break;
    case Encoding::RGB8:
    case Encoding::BGR8: {
      const bool bgr = enc == Encoding::BGR8;
      for (int y = 0; y < h; y++) {
        uint8_t *row = dst + static_cast<size_t>(y) * w * 3;
        for (int x = 0; x < w; x++) {
          const uint8_t *c = chroma(s, x, y, ds);
          uint8_t r, g, b;
          yuv_to_rgb(luma(s, x, y, ds), c[0], c[1], r, g, b);
          row[3 * x + 0] = bgr ? b : r;
          row[3 * x + 1] = g;
          row[3 * x + 2] = bgr ? r : b;
        }
      }
      break;
    }
  }
}

void to_i420(const Nv12View &s, int ds, I420 &out) {
  out.width = scaled_dim(s.width, ds);
  out.height = scaled_dim(s.height, ds);
  const int w = out.width, h = out.height, cw = w / 2, ch = h / 2;
  out.y.resize(static_cast<size_t>(w) * h);
  out.u.resize(static_cast<size_t>(cw) * ch);
  out.v.resize(static_cast<size_t>(cw) * ch);
  for (int y = 0; y < h; y++) {
    uint8_t *row = out.y.data() + static_cast<size_t>(y) * w;
    if (ds == 1) {
      std::copy_n(s.y + static_cast<size_t>(y) * s.stride, w, row);
    } else {
      for (int x = 0; x < w; x++) row[x] = static_cast<uint8_t>(luma(s, x, y, ds));
    }
  }
  for (int y = 0; y < ch; y++) {
    const uint8_t *src = s.uv + static_cast<size_t>(y * ds) * s.stride;
    uint8_t *u = out.u.data() + static_cast<size_t>(y) * cw;
    uint8_t *v = out.v.data() + static_cast<size_t>(y) * cw;
    for (int x = 0; x < cw; x++) {
      u[x] = src[2 * ds * x];
      v[x] = src[2 * ds * x + 1];
    }
  }
}

}  // namespace comma_camera
