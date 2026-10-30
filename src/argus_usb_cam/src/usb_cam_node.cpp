// file: usb_cam_node.cpp
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

// Bounded FIFO shared by two threads. When full, push() drops the oldest item.
template <typename T>
class FrameQueue {
public:
  explicit FrameQueue(size_t capacity) : capacity_(capacity) {}

  // Returns true if an older item was dropped to make room.
  bool push(T item) {
    bool dropped = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (items_.size() >= capacity_) {
        items_.pop_front();
        dropped = true;
      }
      items_.push_back(std::move(item));
    }
    cv_.notify_one();
    return dropped;
  }

  // Blocks until an item is available. Returns false once stop() is called.
  bool pop(T & out) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]{ return stopped_ || !items_.empty(); });
    if (stopped_) return false;
    out = std::move(items_.front());
    items_.pop_front();
    return true;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    cv_.notify_all();
  }

private:
  size_t capacity_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<T> items_;
  bool stopped_{false};
};

class UsbCamNode : public rclcpp::Node {
public:
  UsbCamNode()
  : Node("usb_cam_cpp"),
    system_clock_(RCL_SYSTEM_TIME),
    // Frames held while decoding or publish() is slow. 100 is 5 s at 20 Hz;
    // publish() has been seen to stall for over 2 s with OpenVINS on every module.
    queue_size_(static_cast<size_t>(std::max<int64_t>(1, declare_parameter<int>("queue_size", 100)))),
    raw_queue_(queue_size_),
    msg_queue_(queue_size_) {
    // ---- Parameters
    device_       = declare_parameter<std::string>("device", "/dev/video0");
    // Published size. 640x400 is a 2x scale of the OV9281's full 1280x800 sensor.
    width_        = declare_parameter<int>("width", 640);
    height_       = declare_parameter<int>("height", 400);
    capture_width_  = declare_parameter<int>("capture_width", 1280);
    capture_height_ = declare_parameter<int>("capture_height", 800);
    fourcc_str_   = declare_parameter<std::string>("fourcc", "MJPG"); // MJPG, YUYV, Y800
    open_fps_     = declare_parameter<double>("open_fps", 100.0);     // request at camera
    target_fps_   = declare_parameter<double>("target_fps", 20.0);    // publish rate
    grayscale_    = declare_parameter<bool>("grayscale", true);       // mono8 vs bgr8
    frame_id_     = declare_parameter<std::string>("camera_frame_id", "cam0");

    // Time alignment params
    timeshift_cam_imu_      = declare_parameter<double>("timeshift_cam_imu", 0.0); // added to cam stamp
    exposure_seconds_       = declare_parameter<double>("exposure_seconds", 0.0);  // if known
    use_mid_exposure_stamp_ = declare_parameter<bool>("use_mid_exposure_stamp", false);
    buffers_                = declare_parameter<int>("buffers", 1);                // small buffer to cut latency

    // ---- OpenCV capture
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera: %s", device_.c_str());
      throw std::runtime_error("VideoCapture open failed");
    }

    // Request format
    const int fourcc = cv::VideoWriter::fourcc(
      fourcc_str_.size()>0?fourcc_str_[0]:'M',
      fourcc_str_.size()>1?fourcc_str_[1]:'J',
      fourcc_str_.size()>2?fourcc_str_[2]:'P',
      fourcc_str_.size()>3?fourcc_str_[3]:'G'
    );
    cap_.set(cv::CAP_PROP_FOURCC, fourcc);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  capture_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height_);
    cap_.set(cv::CAP_PROP_FPS, open_fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, buffers_);
    // Hand back the MJPEG bytes undecoded, so decoding runs on its own thread
    // and a slow decode cannot make the capture thread miss frames.
    raw_mjpeg_ = fourcc_str_ == "MJPG" && cap_.set(cv::CAP_PROP_CONVERT_RGB, 0);

    // Log actual settings
    const double aw = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    const double ah = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double afps = cap_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(get_logger(),
      "Opened %s as %.0fx%.0f %s, publishing %dx%d, camera ~%.1f fps, publishing at %.1f Hz | buffers=%d | queue=%zu | decode on %s thread | Δt(cam->imu)=%.6f s",
      device_.c_str(), aw, ah, fourcc_str_.c_str(), width_, height_, afps, target_fps_, buffers_, queue_size_,
      raw_mjpeg_ ? "its own" : "the capture", timeshift_cam_imu_);

    // Publisher (sensor-data friendly)
    rclcpp::SensorDataQoS qos;
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("cam0/image_raw", qos);

    // Capture, decode, and publish run on separate threads joined by queues,
    // so a slow decode or publish() delays frames instead of skipping them.
    publish_thread_ = std::thread([this]{ publishLoop(); });
    decode_thread_ = std::thread([this]{ decodeLoop(); });
    capture_thread_ = std::thread([this]{ captureLoop(); });
  }

  ~UsbCamNode() override {
    stop_ = true;
    raw_queue_.stop();
    msg_queue_.stop();
    if (capture_thread_.joinable()) capture_thread_.join();
    if (decode_thread_.joinable()) decode_thread_.join();
    if (publish_thread_.joinable()) publish_thread_.join();
    cap_.release();
  }

private:
  struct RawFrame {
    rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME};
    cv::Mat data;  // MJPEG bytes when raw_mjpeg_, otherwise the decoded image
  };

  static double ms(std::chrono::steady_clock::duration d) {
    return std::chrono::duration<double, std::milli>(d).count();
  }

  void captureLoop() {
    using steady = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<steady::duration>(
      std::chrono::duration<double>(1.0 / std::max(0.1, target_fps_)));
    auto next_t = steady::now() + period;
    while (!stop_) {
      std::this_thread::sleep_until(next_t);
      const auto woke = steady::now();
      const auto scheduled = next_t;
      next_t += period;
      if (stop_) break;

      // Stamp as close to dequeue as possible: midpoint around grab()
      const rclcpp::Time t_before = system_clock_.now();
      if (!cap_.grab()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to grab frame");
        continue;
      }
      const rclcpp::Time t_after = system_clock_.now();
      const auto grabbed = steady::now();

      RawFrame frame;
      frame.stamp = rclcpp::Time(
        (t_before.nanoseconds() + t_after.nanoseconds()) / 2, RCL_SYSTEM_TIME);
      cv::Mat retrieved;
      const bool ok = cap_.retrieve(retrieved);
      // retrieve() can point into the driver buffer that the next grab() reuses.
      if (ok) frame.data = retrieved.clone();
      const auto done = steady::now();

      // After a long stall, restart the schedule instead of grabbing back to back.
      if (next_t < done) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Capture overran its %.0f ms slot: woke %.1f ms late, grab %.1f, retrieve %.1f ms",
          ms(period), ms(woke - scheduled), ms(grabbed - woke), ms(done - grabbed));
        next_t = done + period;
      }
      if (!ok || frame.data.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to retrieve frame");
        continue;
      }
      if (raw_queue_.push(std::move(frame))) {
        ++raw_dropped_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Decode queue full (%zu); %llu oldest frames dropped so far",
          queue_size_, static_cast<unsigned long long>(raw_dropped_.load()));
      }
    }
  }

  void decodeLoop() {
    RawFrame raw;
    while (raw_queue_.pop(raw)) {
      cv::Mat frame;
      // Raw MJPEG comes back as one row of bytes. Anything else is already decoded.
      if (raw.data.rows == 1 && raw.data.type() == CV_8UC1) {
        frame = cv::imdecode(raw.data, cv::IMREAD_COLOR);
        if (frame.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to decode MJPEG frame");
          continue;
        }
      } else {
        frame = raw.data;
      }
      auto msg = toMessage(frame, raw.stamp);
      if (msg_queue_.push(std::move(msg))) {
        ++msg_dropped_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Camera queue full (%zu); %llu oldest frames dropped so far",
          queue_size_, static_cast<unsigned long long>(msg_dropped_.load()));
      }
    }
  }

  void publishLoop() {
    sensor_msgs::msg::Image::UniquePtr msg;
    while (msg_queue_.pop(msg)) {
      try {
        image_pub_->publish(std::move(msg));
      } catch (const std::exception & ex) {
        if (!rclcpp::ok()) return;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Image publish failed: %s", ex.what());
      }
    }
  }

  sensor_msgs::msg::Image::UniquePtr toMessage(cv::Mat & frame, rclcpp::Time t_stamp) {
    // 640x480 is a different window of this sensor. Scale the full 1280x800 frame.
    if (frame.cols != width_ || frame.rows != height_) {
      cv::resize(frame, frame, cv::Size(width_, height_), 0, 0, cv::INTER_AREA);
    }

    // Mid-exposure correction (optional) + Kalibr shift (once)
    if (use_mid_exposure_stamp_ && exposure_seconds_ > 0.0) {
      t_stamp = t_stamp + rclcpp::Duration::from_seconds(exposure_seconds_ * 0.5);
    }
    if (timeshift_cam_imu_ != 0.0) {
      t_stamp = t_stamp + rclcpp::Duration::from_seconds(timeshift_cam_imu_);
    }

    // Convert to desired encoding
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    if (grayscale_) {
      if (frame.channels() == 3)       cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
      else if (frame.channels() == 4)  cv::cvtColor(frame, frame, cv::COLOR_BGRA2GRAY);
      cv_bridge::CvImage({}, "mono8", frame).toImageMsg(*msg);
    } else {
      if (frame.channels() == 1)       cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
      else if (frame.channels() == 4)  cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
      cv_bridge::CvImage({}, "bgr8", frame).toImageMsg(*msg);
    }

    msg->header.stamp = t_stamp;
    msg->header.frame_id = frame_id_;
    return msg;
  }

  // Params
  std::string device_, fourcc_str_, frame_id_;
  int width_{640}, height_{400}, capture_width_{1280}, capture_height_{800}, buffers_{1};
  double open_fps_{100.0}, target_fps_{20.0};
  bool grayscale_{true};
  double timeshift_cam_imu_{0.0};
  double exposure_seconds_{0.0};
  bool   use_mid_exposure_stamp_{false};
  bool   raw_mjpeg_{false};

  // State
  cv::VideoCapture cap_;
  rclcpp::Clock system_clock_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  size_t queue_size_;
  FrameQueue<RawFrame> raw_queue_;
  FrameQueue<sensor_msgs::msg::Image::UniquePtr> msg_queue_;
  std::atomic<uint64_t> raw_dropped_{0};
  std::atomic<uint64_t> msg_dropped_{0};
  std::atomic<bool> stop_{false};
  std::thread capture_thread_;
  std::thread decode_thread_;
  std::thread publish_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<UsbCamNode>());
  rclcpp::shutdown();
  return 0;
}
