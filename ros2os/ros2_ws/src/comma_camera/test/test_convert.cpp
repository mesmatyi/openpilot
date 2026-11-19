#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

#include "comma_camera/convert.hpp"
#include "comma_camera/intrinsics.hpp"
#include "comma_camera/jpeg.hpp"

using namespace comma_camera;

namespace {

// NV12 test frame with row padding (stride > width), like VisionIPC buffers.
struct Frame {
  int w, h, stride;
  std::vector<uint8_t> buf;
  Frame(int w_, int h_, int pad) : w(w_), h(h_), stride(w_ + pad), buf(static_cast<size_t>(stride) * h_ * 3 / 2, 0xEE) {}
  uint8_t *y() { return buf.data(); }
  uint8_t *uv() { return buf.data() + static_cast<size_t>(stride) * h; }
  Nv12View view() { return {y(), uv(), w, h, stride}; }
  void fill(uint8_t Y, uint8_t U, uint8_t V) {
    for (int r = 0; r < h; r++)
      for (int c = 0; c < w; c++) y()[r * stride + c] = Y;
    for (int r = 0; r < h / 2; r++)
      for (int c = 0; c < w / 2; c++) {
        uv()[r * stride + 2 * c] = U;
        uv()[r * stride + 2 * c + 1] = V;
      }
  }
};

}  // namespace

TEST(Convert, EncodingNames) {
  for (auto name : {"nv12", "mono8", "rgb8", "bgr8"}) EXPECT_STREQ(encoding_name(parse_encoding(name)), name);
  EXPECT_THROW(parse_encoding("yuv422"), std::invalid_argument);
}

TEST(Convert, Geometry) {
  EXPECT_TRUE(valid_geometry(1928, 1208, 1));
  EXPECT_TRUE(valid_geometry(1928, 1208, 2));
  EXPECT_TRUE(valid_geometry(1344, 760, 2));
  EXPECT_FALSE(valid_geometry(1930, 1206, 2));
  EXPECT_FALSE(valid_geometry(8, 8, 3));
  EXPECT_EQ(output_size(Encoding::NV12, 8, 4), 48u);
  EXPECT_EQ(output_size(Encoding::BGR8, 8, 4), 96u);
  EXPECT_EQ(output_step(Encoding::RGB8, 8), 24u);
}

TEST(Convert, Nv12RepacksStride) {
  Frame f(8, 4, 24);
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 8; c++) f.y()[r * f.stride + c] = static_cast<uint8_t>(r * 8 + c);
  for (int r = 0; r < 2; r++)
    for (int c = 0; c < 8; c++) f.uv()[r * f.stride + c] = static_cast<uint8_t>(100 + r * 8 + c);
  std::vector<uint8_t> out(output_size(Encoding::NV12, 8, 4));
  convert(f.view(), Encoding::NV12, 1, out.data());
  for (int i = 0; i < 32; i++) EXPECT_EQ(out[i], i);
  for (int i = 0; i < 16; i++) EXPECT_EQ(out[32 + i], 100 + i);
}

TEST(Convert, MonoDownscaleAverages) {
  Frame f(8, 8, 8);
  f.fill(0, 128, 128);
  f.y()[0] = 100;
  f.y()[1] = 200;
  f.y()[f.stride] = 0;
  f.y()[f.stride + 1] = 100;
  std::vector<uint8_t> out(output_size(Encoding::MONO8, 4, 4));
  convert(f.view(), Encoding::MONO8, 2, out.data());
  EXPECT_EQ(out[0], 100);  // (100 + 200 + 0 + 100 + 2) / 4
  EXPECT_EQ(out[1], 0);
}

TEST(Convert, ColourFullRangeBt601) {
  Frame f(4, 4, 0);
  std::vector<uint8_t> out(output_size(Encoding::RGB8, 4, 4));

  f.fill(128, 128, 128);  // neutral grey
  convert(f.view(), Encoding::RGB8, 1, out.data());
  EXPECT_EQ(out[0], 128);
  EXPECT_EQ(out[1], 128);
  EXPECT_EQ(out[2], 128);

  f.fill(76, 85, 255);  // JFIF encoding of pure red
  convert(f.view(), Encoding::RGB8, 1, out.data());
  EXPECT_NEAR(out[0], 255, 2);
  EXPECT_NEAR(out[1], 0, 2);
  EXPECT_NEAR(out[2], 0, 2);

  std::vector<uint8_t> bgr(out.size());
  convert(f.view(), Encoding::BGR8, 1, bgr.data());
  EXPECT_EQ(bgr[0], out[2]);
  EXPECT_EQ(bgr[1], out[1]);
  EXPECT_EQ(bgr[2], out[0]);

  f.fill(255, 128, 128);  // clamps instead of wrapping
  convert(f.view(), Encoding::RGB8, 2, out.data());
  EXPECT_EQ(out[0], 255);
}

TEST(Convert, I420Split) {
  Frame f(8, 4, 8);
  f.fill(10, 20, 30);
  f.uv()[2] = 21;  // second chroma sample U
  f.uv()[3] = 31;
  I420 img;
  to_i420(f.view(), 1, img);
  EXPECT_EQ(img.width, 8);
  EXPECT_EQ(img.y.size(), 32u);
  EXPECT_EQ(img.u.size(), 8u);
  EXPECT_EQ(img.u[0], 20);
  EXPECT_EQ(img.v[0], 30);
  EXPECT_EQ(img.u[1], 21);
  EXPECT_EQ(img.v[1], 31);
  to_i420(f.view(), 2, img);
  EXPECT_EQ(img.width, 4);
  EXPECT_EQ(img.u.size(), 2u);
  EXPECT_EQ(img.u[0], 20);
}

TEST(Jpeg, EncodesValidJfif) {
  Frame f(64, 32, 16);
  f.fill(128, 100, 160);
  I420 img;
  to_i420(f.view(), 1, img);
  JpegEncoder enc;
  std::vector<uint8_t> out;
  ASSERT_TRUE(enc.encode(img, 80, out));
  ASSERT_GT(out.size(), 4u);
  EXPECT_EQ(out[0], 0xFF);
  EXPECT_EQ(out[1], 0xD8);
  EXPECT_EQ(out[out.size() - 2], 0xFF);
  EXPECT_EQ(out[out.size() - 1], 0xD9);
  ASSERT_TRUE(enc.encode(img, 80, out));  // buffer reuse
}

TEST(Intrinsics, MatchOpenpilotTable) {
  EXPECT_DOUBLE_EQ(*nominal_focal_length("narrowRoad", 1928, 1208), 2648.0);
  EXPECT_DOUBLE_EQ(*nominal_focal_length("cabin", 1928, 1208), 567.0);
  EXPECT_DOUBLE_EQ(*nominal_focal_length("narrowRoad", 1344, 760), 1141.5);
  EXPECT_DOUBLE_EQ(*nominal_focal_length("wideRoad", 1344, 760), 425.25);
  EXPECT_FALSE(nominal_focal_length("wideRoad", 640, 480).has_value());
}
