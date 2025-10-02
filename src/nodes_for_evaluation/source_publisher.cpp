#include <chrono>
#include <custom_msg/msg/header_extra_stamp.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class SourcePublisher : public rclcpp::Node {
public:
  SourcePublisher() : Node("source_publisher"), index_(0) {
    publisher_ = this->create_publisher<custom_msg::msg::HeaderExtraStamp>(
        "start_topic", 1000);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&SourcePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    // Wait for 2 seconds on first call
    static auto start_time = this->now();
    auto elapsed = this->now() - start_time;
    if (elapsed.seconds() < 2.0) {
      return;
    }

    auto message = custom_msg::msg::HeaderExtraStamp();
    message.header.stamp = this->now();
    message.header.frame_id = std::to_string(index_);

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "SourcePublisher: %u", index_);
    index_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msg::msg::HeaderExtraStamp>::SharedPtr publisher_;
  uint32_t index_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SourcePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
