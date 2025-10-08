#include <chrono>
#include <custom_msg/msg/header_extra_stamp.hpp>
#include <memory>
#include <pmu_analyzer.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

long long to_microseconds(const builtin_interfaces::msg::Time &t) {
  return static_cast<long long>(t.sec) * 1000000LL +
         static_cast<long long>(t.nanosec) / 1000LL;
}

class SubscribePublisher : public rclcpp::Node {
public:
  explicit SubscribePublisher(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("subscribe_publisher", options), rng_(std::random_device{}()),
        dist_(1000, 50000) {
    this->declare_parameter("topic_id", 1);
    topic_id_ = this->get_parameter("topic_id").as_int();
    std::string topic_name = "topic" + std::to_string(topic_id_);
    session_name_ =
        "subscribe_publisher_published_" + std::to_string(topic_id_);

    publisher_ = this->create_publisher<custom_msg::msg::HeaderExtraStamp>(
        topic_name, 1000);
    subscriber_ = this->create_subscription<custom_msg::msg::HeaderExtraStamp>(
        "start_topic", 1000,
        std::bind(&SubscribePublisher::start_callback, this,
                  std::placeholders::_1));

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SubscribePublisher() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  void wait_microsec(uint64_t usec) {
    using clock = std::chrono::steady_clock;
    const auto start = clock::now();

    while (true) {
      auto now = clock::now();
      auto diff =
          std::chrono::duration_cast<std::chrono::microseconds>(now - start)
              .count();

      if (diff >= static_cast<long long>(usec))
        break;
    }
  }

  void start_callback(const custom_msg::msg::HeaderExtraStamp::SharedPtr msg) {
    int index = std::stoi(msg->header.frame_id);
    RCLCPP_INFO(this->get_logger(), "SubscribePublisher %d: %u", topic_id_,
                index);

    // Random pseudo processing (1000-50000 us)
    int iterations = dist_(rng_);
    wait_microsec(iterations);

    msg->header.stamp = this->now();
    publisher_->publish(*msg);
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(session_name_, 0, true,
                                         to_microseconds(msg->header.stamp));
  }

  rclcpp::Publisher<custom_msg::msg::HeaderExtraStamp>::SharedPtr publisher_;
  rclcpp::Subscription<custom_msg::msg::HeaderExtraStamp>::SharedPtr
      subscriber_;
  int topic_id_;
  std::string session_name_;
  std::mt19937 rng_;
  std::uniform_int_distribution<int> dist_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(SubscribePublisher)
