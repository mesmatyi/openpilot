#include "comma_camera/jpeg.hpp"

#include <turbojpeg.h>

#include <algorithm>
#include <stdexcept>

namespace comma_camera {

JpegEncoder::JpegEncoder() {
  handle_ = tjInitCompress();
  if (handle_ == nullptr) throw std::runtime_error("tjInitCompress failed");
}

JpegEncoder::~JpegEncoder() {
  if (buf_ != nullptr) tjFree(buf_);
  if (handle_ != nullptr) tjDestroy(static_cast<tjhandle>(handle_));
}

bool JpegEncoder::encode(const I420 &img, int quality, std::vector<uint8_t> &out) {
  const unsigned char *planes[3] = {img.y.data(), img.u.data(), img.v.data()};
  const int strides[3] = {img.width, img.width / 2, img.width / 2};
  // TurboJPEG grows buf_ as needed and we keep it between frames
  unsigned long size = buf_size_;
  if (tjCompressFromYUVPlanes(static_cast<tjhandle>(handle_), planes, img.width, strides, img.height, TJSAMP_420, &buf_,
                              &size, quality, TJFLAG_FASTDCT) != 0) {
    return false;
  }
  buf_size_ = std::max(buf_size_, size);
  out.assign(buf_, buf_ + size);
  return true;
}

}  // namespace comma_camera
