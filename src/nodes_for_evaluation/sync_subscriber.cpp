#include <custom_msg/msg/header_extra_stamp.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pmu_analyzer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

long long to_microseconds(const builtin_interfaces::msg::Time &t) {
  return static_cast<long long>(t.sec) * 1000000LL +
         static_cast<long long>(t.nanosec) / 1000LL;
}

// Template specializations for different numbers of publishers
template <int N> class SyncSubscriber;

// Specialization for 1 publisher (no synchronization needed, just a simple
// subscriber)
template <> class SyncSubscriber<1> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ = this->create_subscription<custom_msg::msg::HeaderExtraStamp>(
        "topic1", 1000,
        std::bind(&SyncSubscriber<1>::callback, this, std::placeholders::_1));

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(session_name_, 0, true,
                                         to_microseconds(msg1->extra_stamp));
    RCLCPP_INFO(this->get_logger(),
                "SyncSubscriber: index = %s, sync_cb_index_ = %zu",
                msg1->header.frame_id.c_str(), sync_cb_index_);
    sync_cb_index_++;
  }

  rclcpp::Subscription<custom_msg::msg::HeaderExtraStamp>::SharedPtr sub1_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 2 publishers
template <> class SyncSubscriber<2> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic1");
    sub2_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic2");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(1000), *sub1_, *sub2_);
    sync_->registerCallback(std::bind(&SyncSubscriber<2>::callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      SyncPolicy;

  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg2) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::max(to_microseconds(msg1->extra_stamp),
                 to_microseconds(msg2->extra_stamp)));
    RCLCPP_INFO(this->get_logger(),
                "SyncSubscriber: msg1_index = %s, msg2_index = %s, "
                "sync_cb_index_ = %zu",
                msg1->header.frame_id.c_str(), msg2->header.frame_id.c_str(),
                sync_cb_index_);
    sync_cb_index_++;
  }

  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub1_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 3 publishers
template <> class SyncSubscriber<3> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic1");
    sub2_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic2");
    sub3_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic3");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(1000), *sub1_, *sub2_, *sub3_);
    sync_->registerCallback(
        std::bind(&SyncSubscriber<3>::callback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp>
      SyncPolicy;

  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg2,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg3) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::max({to_microseconds(msg1->extra_stamp),
                  to_microseconds(msg2->extra_stamp),
                  to_microseconds(msg3->extra_stamp)}));
    RCLCPP_INFO(this->get_logger(),
                "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index "
                "= %s, sync_cb_index_ = %zu",
                msg1->header.frame_id.c_str(), msg2->header.frame_id.c_str(),
                msg3->header.frame_id.c_str(), sync_cb_index_);
    sync_cb_index_++;
  }

  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub1_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub2_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub3_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 4 publishers
template <> class SyncSubscriber<4> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic1");
    sub2_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic2");
    sub3_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic3");
    sub4_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic4");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_);
    sync_->registerCallback(std::bind(
        &SyncSubscriber<4>::callback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      SyncPolicy;

  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg2,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg3,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg4) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::max({to_microseconds(msg1->extra_stamp),
                  to_microseconds(msg2->extra_stamp),
                  to_microseconds(msg3->extra_stamp),
                  to_microseconds(msg4->extra_stamp)}));
    RCLCPP_INFO(
        this->get_logger(),
        "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index = "
        "%s, msg4_index = %s, sync_cb_index_ = %zu",
        msg1->header.frame_id.c_str(), msg2->header.frame_id.c_str(),
        msg3->header.frame_id.c_str(), msg4->header.frame_id.c_str(),
        sync_cb_index_);
    sync_cb_index_++;
  }

  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub1_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub2_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub3_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub4_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Wrapper component that creates the appropriate template instance
class SyncSubscriberComponent : public rclcpp::Node {
public:
  explicit SyncSubscriberComponent(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber_wrapper", options) {
    // This node is just a placeholder - the real work is done by registering
    // the template specializations
    RCLCPP_WARN(this->get_logger(),
                "SyncSubscriberComponent is a placeholder. Use "
                "SyncSubscriber1, SyncSubscriber2, SyncSubscriber3, or "
                "SyncSubscriber4 instead.");
  }
};

// Register each template specialization as a separate component
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<1>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<2>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<3>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<4>)
