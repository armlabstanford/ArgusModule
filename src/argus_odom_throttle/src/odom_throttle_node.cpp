// Republish the newest OpenVINS odometry at a fixed rate.
// OpenVINS publishes odomimu at IMU rate (~250 Hz); the IK server only needs
// ~50 Hz, and every extra message costs Wi-Fi airtime.
//   in:  <ns>/odomimu            (nav_msgs/Odometry)
//   out: <ns>/odomimu_throttled  (nav_msgs/Odometry, best effort, keep last 1)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <memory>

class OdomThrottle : public rclcpp::Node {
public:
  OdomThrottle()
  : rclcpp::Node("odom_throttle") {
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    const auto input = declare_parameter<std::string>("input_topic", "odomimu");
    const auto output = declare_parameter<std::string>("output_topic", "odomimu_throttled");

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    pub_ = create_publisher<nav_msgs::msg::Odometry>(output, qos);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input, qos,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) { latest_ = std::move(msg); });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      [this]() {
        if (latest_) {
          pub_->publish(std::move(latest_));
        }
      });

    RCLCPP_INFO(get_logger(), "%s/%s -> %s/%s at %.1f Hz",
      get_namespace(), input.c_str(), get_namespace(), output.c_str(), rate_hz);
  }

private:
  nav_msgs::msg::Odometry::UniquePtr latest_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomThrottle>());
  rclcpp::shutdown();
  return 0;
}
