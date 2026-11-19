// ROS 2 camera driver for comma devices.
//
// openpilot's camerad owns the Qualcomm Spectra ISP and the image sensors and
// exposes the processed NV12 frames through VisionIPC (shared ION buffers).
// This node maps those buffers zero-copy and publishes, per camera:
//
//   camera/<name>/image_raw             sensor_msgs/Image           (encoding: nv12 | mono8 | rgb8 | bgr8)
//   camera/<name>/image_raw/compressed  sensor_msgs/CompressedImage (JPEG straight from the YUV planes)
//   camera/<name>/camera_info           sensor_msgs/CameraInfo      (openpilot's nominal pinhole intrinsics)
//
// with <name> in {narrowRoad, wideRoad, cabin} and optical frames
// {narrow_road_optical, wide_road_optical, cabin_optical}, matching tools/mcap.
// Work is only done for outputs that have subscribers.

#include <time.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "msgq/visionipc/visionipc_client.h"

#include "comma_camera/convert.hpp"
#include "comma_camera/intrinsics.hpp"
#include "comma_camera/jpeg.hpp"

namespace comma_camera {

namespace {

struct CameraSpec {
  const char *name;
  int stream_type;  // openpilot/cereal/visionstream.h
  const char *frame_id;
};

constexpr CameraSpec CAMERAS[] = {
  {"narrowRoad", 0, "narrow_road_optical"},  // VISION_STREAM_NARROW_ROAD
  {"wideRoad", 2, "wide_road_optical"},      // VISION_STREAM_WIDE_ROAD
  {"cabin", 1, "cabin_optical"},             // VISION_STREAM_CABIN
};

int64_t to_ns(const timespec &t) { return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec; }

// camerad timestamps are CLOCK_BOOTTIME; ROS stamps are CLOCK_REALTIME
int64_t boottime_to_realtime_ns() {
  timespec r0{}, b{}, r1{};
  clock_gettime(CLOCK_REALTIME, &r0);
  clock_gettime(CLOCK_BOOTTIME, &b);
  clock_gettime(CLOCK_REALTIME, &r1);
  return (to_ns(r0) + to_ns(r1)) / 2 - to_ns(b);
}

builtin_interfaces::msg::Time to_stamp(int64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

template <typename PubT>
bool has_subscribers(const PubT &pub) {
  return pub->get_subscription_count() + pub->get_intra_process_subscription_count() > 0;
}

}  // namespace

class CameraNode : public rclcpp::Node {
 public:
  explicit CameraNode(const rclcpp::NodeOptions &options) : Node("camera", options) {
    server_ = declare_parameter<std::string>("vipc_server", "camerad");
    const auto names = declare_parameter<std::vector<std::string>>("cameras", {"narrowRoad", "wideRoad", "cabin"});
    encoding_ = parse_encoding(declare_parameter<std::string>("encoding", "bgr8"));
    downscale_ = static_cast<int>(declare_parameter<int64_t>("downscale", 1));
    jpeg_quality_ = static_cast<int>(declare_parameter<int64_t>("jpeg_quality", 80));
    jpeg_rate_divider_ = static_cast<int>(declare_parameter<int64_t>("jpeg_rate_divider", 1));
    const auto stamp = declare_parameter<std::string>("stamp", "sof");
    if (downscale_ != 1 && downscale_ != 2) throw std::invalid_argument("downscale must be 1 or 2");
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) throw std::invalid_argument("jpeg_quality must be 1..100");
    if (jpeg_rate_divider_ < 1) throw std::invalid_argument("jpeg_rate_divider must be >= 1");
    if (stamp != "sof" && stamp != "eof") throw std::invalid_argument("stamp must be 'sof' or 'eof'");
    stamp_eof_ = stamp == "eof";

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    for (const auto &name : names) {
      const CameraSpec *spec = nullptr;
      for (const auto &c : CAMERAS) {
        if (name == c.name) spec = &c;
      }
      if (spec == nullptr) throw std::invalid_argument("unknown camera '" + name + "' (narrowRoad, wideRoad, cabin)");
      auto s = std::make_unique<Stream>();
      s->spec = *spec;
      s->focal_override = declare_parameter<double>(name + ".focal_length", 0.0);
      const std::string base = std::string("camera/") + spec->name;
      s->image_pub = create_publisher<sensor_msgs::msg::Image>(base + "/image_raw", qos);
      s->jpeg_pub = create_publisher<sensor_msgs::msg::CompressedImage>(base + "/image_raw/compressed", qos);
      s->info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(base + "/camera_info", qos);
      streams_.push_back(std::move(s));
    }
    for (auto &s : streams_) {
      Stream *ptr = s.get();
      s->thread = std::thread([this, ptr] { run(*ptr); });
    }
    RCLCPP_INFO(get_logger(), "publishing %zu camera(s) from VisionIPC server '%s', encoding %s, downscale %d",
                streams_.size(), server_.c_str(), encoding_name(encoding_), downscale_);
  }

  ~CameraNode() override {
    stop_ = true;
    for (auto &s : streams_) {
      if (s->thread.joinable()) s->thread.join();
    }
  }

 private:
  struct Stream {
    CameraSpec spec{};
    double focal_override = 0.0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    std::thread thread;
    I420 i420;
    std::unique_ptr<JpegEncoder> jpeg;
    bool warned_geometry = false;
    bool warned_intrinsics = false;
    uint64_t frames = 0;
  };

  void run(Stream &s) {
    VisionIpcClient client(server_, static_cast<VisionStreamType>(s.spec.stream_type), false);
    s.jpeg = std::make_unique<JpegEncoder>();
    bool waiting_logged = false;
    while (!stop_ && rclcpp::ok()) {
      if (!client.connected) {
        if (!client.connect(false)) {
          if (!waiting_logged) {
            RCLCPP_INFO(get_logger(), "%s: waiting for camerad", s.spec.name);
            waiting_logged = true;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }
        waiting_logged = false;
        RCLCPP_INFO(get_logger(), "%s: connected (%zux%zu, %d buffers)", s.spec.name,
                    static_cast<size_t>(client.buffers[0].width), static_cast<size_t>(client.buffers[0].height),
                    static_cast<int>(client.num_buffers));
      }
      VisionIpcBufExtra extra{};
      VisionBuf *buf = client.recv(&extra, 100);
      if (buf == nullptr) continue;
      try {
        publish(s, *buf, extra);
      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s: %s", s.spec.name, e.what());
      }
    }
  }

  void publish(Stream &s, const VisionBuf &buf, const VisionIpcBufExtra &extra) {
    const Nv12View view{buf.y, buf.uv, static_cast<int>(buf.width), static_cast<int>(buf.height),
                        static_cast<int>(buf.stride)};
    if (!valid_geometry(view.width, view.height, downscale_)) {
      if (!s.warned_geometry) {
        RCLCPP_ERROR(get_logger(), "%s: cannot convert %dx%d with downscale %d", s.spec.name, view.width, view.height,
                     downscale_);
        s.warned_geometry = true;
      }
      return;
    }
    const int w = scaled_dim(view.width, downscale_), h = scaled_dim(view.height, downscale_);
    const uint64_t ts = stamp_eof_ ? extra.timestamp_eof : extra.timestamp_sof;
    std_msgs::msg::Header header;
    header.stamp = to_stamp(static_cast<int64_t>(ts) + boottime_to_realtime_ns());
    header.frame_id = s.spec.frame_id;
    s.frames++;

    if (has_subscribers(s.info_pub)) {
      s.info_pub->publish(camera_info(s, header, view.width, view.height, w, h));
    }

    if (has_subscribers(s.image_pub)) {
      auto img = std::make_unique<sensor_msgs::msg::Image>();
      img->header = header;
      img->width = w;
      img->height = h;
      img->encoding = encoding_name(encoding_);
      img->is_bigendian = 0;
      img->step = static_cast<uint32_t>(output_step(encoding_, w));
      img->data.resize(output_size(encoding_, w, h));
      convert(view, encoding_, downscale_, img->data.data());
      s.image_pub->publish(std::move(img));
    }

    if (has_subscribers(s.jpeg_pub) && extra.frame_id % static_cast<uint32_t>(jpeg_rate_divider_) == 0) {
      to_i420(view, downscale_, s.i420);
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header = header;
      msg->format = "rgb8; jpeg compressed bgr8";  // same as tools/mcap exports; decodes with image_transport
      if (s.jpeg->encode(s.i420, jpeg_quality_, msg->data)) {
        s.jpeg_pub->publish(std::move(msg));
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s: JPEG encoding failed", s.spec.name);
      }
    }
  }

  sensor_msgs::msg::CameraInfo camera_info(Stream &s, const std_msgs::msg::Header &header, int full_w, int full_h,
                                           int w, int h) {
    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.width = w;
    info.height = h;
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    double f = s.focal_override;
    if (f <= 0.0) f = nominal_focal_length(s.spec.name, full_w, full_h).value_or(0.0);
    if (f <= 0.0 && !s.warned_intrinsics) {
      RCLCPP_WARN(get_logger(), "%s: no nominal intrinsics for %dx%d; set %s.focal_length (publishing an uncalibrated "
                  "CameraInfo)", s.spec.name, full_w, full_h, s.spec.name);
      s.warned_intrinsics = true;
    }
    f /= downscale_;
    const double cx = w / 2.0, cy = h / 2.0;  // openpilot puts the principal point at the image centre
    info.k = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  std::string server_;
  Encoding encoding_ = Encoding::BGR8;
  int downscale_ = 1;
  int jpeg_quality_ = 80;
  int jpeg_rate_divider_ = 1;
  bool stamp_eof_ = false;
  std::atomic<bool> stop_{false};
  std::vector<std::unique_ptr<Stream>> streams_;
};

}  // namespace comma_camera

RCLCPP_COMPONENTS_REGISTER_NODE(comma_camera::CameraNode)
