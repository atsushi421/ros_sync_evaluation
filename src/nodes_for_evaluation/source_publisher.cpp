#include <chrono>
#include <custom_msg/msg/header_extra_stamp.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

class SourcePublisher : public rclcpp::Node {
public:
  explicit SourcePublisher(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("source_publisher", options), index_(0) {
    publisher_ = this->create_publisher<custom_msg::msg::HeaderExtraStamp>(
        "start_topic", 1000);
    timer_ = this->create_wall_timer(
        25ms, std::bind(&SourcePublisher::timer_callback, this));
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

RCLCPP_COMPONENTS_REGISTER_NODE(SourcePublisher)
