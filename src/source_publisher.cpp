#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

class SourcePublisher : public rclcpp::Node {
public:
  SourcePublisher() : Node("source_publisher"), index_(0) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::PointStamped>("start_topic", 1000);
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

    auto message = geometry_msgs::msg::PointStamped();
    message.header.stamp = this->now();
    message.header.frame_id = std::to_string(index_);
    // Add dummy data
    message.point.x = 1.0;
    message.point.y = 2.0;
    message.point.z = 3.0;

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "SourcePublisher: %u", index_);
    index_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  uint32_t index_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SourcePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
