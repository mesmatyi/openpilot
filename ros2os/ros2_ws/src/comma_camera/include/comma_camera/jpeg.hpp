// JPEG encoding of I420 frames with libjpeg-turbo (TurboJPEG API).
#pragma once

#include <cstdint>
#include <vector>

#include "comma_camera/convert.hpp"

namespace comma_camera {

class JpegEncoder {
 public:
  JpegEncoder();
  ~JpegEncoder();
  JpegEncoder(const JpegEncoder &) = delete;
  JpegEncoder &operator=(const JpegEncoder &) = delete;

  // Encodes 4:2:0 planes directly (no RGB round trip); returns false on error.
  bool encode(const I420 &img, int quality, std::vector<uint8_t> &out);

 private:
  void *handle_ = nullptr;
  unsigned char *buf_ = nullptr;
  unsigned long buf_size_ = 0;
};

}  // namespace comma_camera
