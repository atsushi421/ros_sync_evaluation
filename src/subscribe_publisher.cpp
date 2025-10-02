#include <chrono>
#include <memory>
#include <pmu_analyzer.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class SubscribePublisher : public rclcpp::Node {
public:
  SubscribePublisher()
      : Node("subscribe_publisher"), rng_(std::random_device{}()),
        dist_(10000, 10000000) {
    this->declare_parameter("topic_id", 1);
    topic_id_ = this->get_parameter("topic_id").as_int();
    std::string topic_name = "topic" + std::to_string(topic_id_);

    publisher_ =
        this->create_publisher<std_msgs::msg::Header>(topic_name, 1000);
    subscriber_ = this->create_subscription<std_msgs::msg::Header>(
        "start_topic", 1000,
        std::bind(&SubscribePublisher::start_callback, this,
                  std::placeholders::_1));

    // Initialize PMU analyzer for elapsed time measurement
    session_name_ =
        "subscriber_publisher" + std::to_string(topic_id_) + "_published";
    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SubscribePublisher() {
    // Close PMU analyzer session
    pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_);
  }

private:
  void pseudo_processing(int iterations) {
    volatile double result = 0.0;
    for (int i = 0; i < iterations; ++i) {
      result += std::sin(i) * std::cos(i);
    }
  }

  void start_callback(const std_msgs::msg::Header::SharedPtr msg) {
    int index = std::stoi(msg->frame_id);
    RCLCPP_INFO(this->get_logger(), "SubscribePublisher %d: %u", topic_id_,
                index);

    // Random pseudo processing (10000-10000000 iterations)
    int iterations = dist_(rng_);
    pseudo_processing(iterations);

    publisher_->publish(*msg);
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(session_name_, 0, true, index);
  }

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscriber_;
  int topic_id_;
  std::string session_name_;
  std::mt19937 rng_;
  std::uniform_int_distribution<int> dist_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscribePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
