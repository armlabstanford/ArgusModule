// LEDManager: pulse status LEDs while sensor topics are arriving.
//   LED 1 (1 Hz): <ns>/cam0/image_raw and <ns>/imu0 both recent
//   LED 2 (2 Hz): <ns>/odomimu recent
// Each pulse is on for `on_time_s`. An LED stays off when its topics go quiet
// for longer than `timeout_s`.
// Pins are BCM GPIO numbers on `gpio_chip`; -1 disables that LED.

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/gpio.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;

class GpioLine {
public:
  GpioLine() = default;
  GpioLine(const GpioLine &) = delete;
  GpioLine & operator=(const GpioLine &) = delete;
  ~GpioLine() { release(); }

  bool open(const std::string & chip_path, int offset, std::string & error) {
    int chip_fd = ::open(chip_path.c_str(), O_RDWR | O_CLOEXEC);
    if (chip_fd < 0) {
      error = "cannot open " + chip_path + ": " + std::strerror(errno);
      return false;
    }
    gpio_v2_line_request request{};
    request.offsets[0] = static_cast<__u32>(offset);
    request.num_lines = 1;
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    std::strncpy(request.consumer, "led_manager", sizeof(request.consumer) - 1);
    int rc = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request);
    int saved_errno = errno;
    ::close(chip_fd);
    if (rc < 0) {
      error = "cannot claim GPIO " + std::to_string(offset) + ": " + std::strerror(saved_errno);
      return false;
    }
    fd_ = request.fd;
    set(false);
    return true;
  }

  void set(bool on) {
    if (fd_ < 0) {
      return;
    }
    gpio_v2_line_values values{};
    values.mask = 1;
    values.bits = on ? 1 : 0;
    ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &values);
  }

  void release() {
    if (fd_ >= 0) {
      set(false);
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_ = -1;
};

class LEDManager : public rclcpp::Node {
public:
  LEDManager()
  : rclcpp::Node("led_manager") {
    const auto chip = declare_parameter<std::string>("gpio_chip", "/dev/gpiochip0");
    const int led1_gpio = declare_parameter<int>("led1_gpio", -1);
    const int led2_gpio = declare_parameter<int>("led2_gpio", -1);
    timeout_ = seconds(declare_parameter<double>("timeout_s", 1.0));
    on_time_ = seconds(declare_parameter<double>("on_time_s", 0.1));
    led1_period_ = seconds(1.0 / declare_parameter<double>("led1_hz", 1.0));
    led2_period_ = seconds(1.0 / declare_parameter<double>("led2_hz", 2.0));
    start_ = Clock::now();

    open_led(led1_, chip, led1_gpio, "led1_gpio");
    open_led(led2_, chip, led2_gpio, "led2_gpio");

    // Generic subscriptions only note arrival; images are never deserialized.
    // Best effort accepts both best-effort and reliable publishers.
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cam_sub_ = watch("cam0/image_raw", "sensor_msgs/msg/Image", qos, last_cam_);
    imu_sub_ = watch("imu0", "sensor_msgs/msg/Imu", qos, last_imu_);
    odom_sub_ = watch("odomimu", "nav_msgs/msg/Odometry", qos, last_odom_);

    // Fine enough to time a short pulse.
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { update(); });

    RCLCPP_INFO(get_logger(),
      "LED 1 (GPIO %d): %s/cam0/image_raw + %s/imu0 | LED 2 (GPIO %d): %s/odomimu",
      led1_gpio, get_namespace(), get_namespace(), led2_gpio, get_namespace());
  }

private:
  void open_led(GpioLine & line, const std::string & chip, int gpio, const char * param) {
    if (gpio < 0) {
      RCLCPP_WARN(get_logger(), "%s is not set; that LED is disabled", param);
      return;
    }
    std::string error;
    if (!line.open(chip, gpio, error)) {
      RCLCPP_ERROR(get_logger(), "%s: %s", param, error.c_str());
    }
  }

  rclcpp::GenericSubscription::SharedPtr watch(
    const std::string & topic, const std::string & type,
    const rclcpp::QoS & qos, Clock::time_point & last)
  {
    return create_generic_subscription(
      topic, type, qos,
      [&last](std::shared_ptr<rclcpp::SerializedMessage>) { last = Clock::now(); });
  }

  static Clock::duration seconds(double value) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(value));
  }

  bool recent(Clock::time_point last, Clock::time_point now) const {
    return last != Clock::time_point{} && now - last <= timeout_;
  }

  bool pulse_on(Clock::time_point now, Clock::duration period) const {
    return (now - start_) % period < on_time_;
  }

  void update() {
    const auto now = Clock::now();
    const bool led1_active = recent(last_cam_, now) && recent(last_imu_, now);
    const bool led2_active = recent(last_odom_, now);
    led1_.set(led1_active && pulse_on(now, led1_period_));
    led2_.set(led2_active && pulse_on(now, led2_period_));
  }

  GpioLine led1_;
  GpioLine led2_;
  Clock::duration timeout_{};
  Clock::duration on_time_{};
  Clock::duration led1_period_{};
  Clock::duration led2_period_{};
  Clock::time_point start_{};
  Clock::time_point last_cam_{};
  Clock::time_point last_imu_{};
  Clock::time_point last_odom_{};
  rclcpp::GenericSubscription::SharedPtr cam_sub_;
  rclcpp::GenericSubscription::SharedPtr imu_sub_;
  rclcpp::GenericSubscription::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDManager>());
  rclcpp::shutdown();
  return 0;
}
