#include <custom_msg/msg/header_extra_stamp.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
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
    // Declare parameter for sync policy (not used for single publisher, but declared for consistency)
    this->declare_parameter<std::string>("sync_policy", "exact");

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
    // Declare and get sync policy parameter
    this->declare_parameter<std::string>("sync_policy", "exact");
    this->declare_parameter<double>("max_interval_duration", 100.0);
    std::string sync_policy = this->get_parameter("sync_policy").as_string();
    double max_interval_ms = this->get_parameter("max_interval_duration").as_double();

    sub1_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic1");
    sub2_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic2");

    if (sync_policy == "approximate") {
      sync_approx_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
          ApproxSyncPolicy(1000), *sub1_, *sub2_);
      sync_approx_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_interval_ms * 1e6)));
      sync_approx_->registerCallback(std::bind(&SyncSubscriber<2>::callback, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
      RCLCPP_INFO(this->get_logger(), "Using ApproximateTime sync policy (max_interval: %.1fms)", max_interval_ms);
    } else {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
          ExactSyncPolicy(1000), *sub1_, *sub2_);
      sync_exact_->registerCallback(std::bind(&SyncSubscriber<2>::callback, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
      RCLCPP_INFO(this->get_logger(), "Using ExactTime sync policy");
    }

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ApproxSyncPolicy;

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
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_approx_;
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
    // Declare and get sync policy parameter
    this->declare_parameter<std::string>("sync_policy", "exact");
    this->declare_parameter<double>("max_interval_duration", 100.0);
    std::string sync_policy = this->get_parameter("sync_policy").as_string();
    double max_interval_ms = this->get_parameter("max_interval_duration").as_double();

    sub1_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic1");
    sub2_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic2");
    sub3_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic3");

    if (sync_policy == "approximate") {
      sync_approx_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
          ApproxSyncPolicy(1000), *sub1_, *sub2_, *sub3_);
      sync_approx_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_interval_ms * 1e6)));
      sync_approx_->registerCallback(
          std::bind(&SyncSubscriber<3>::callback, this, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3));
      RCLCPP_INFO(this->get_logger(), "Using ApproximateTime sync policy (max_interval: %.1fms)", max_interval_ms);
    } else {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
          ExactSyncPolicy(1000), *sub1_, *sub2_, *sub3_);
      sync_exact_->registerCallback(
          std::bind(&SyncSubscriber<3>::callback, this, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3));
      RCLCPP_INFO(this->get_logger(), "Using ExactTime sync policy");
    }

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp>
      ApproxSyncPolicy;

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
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_approx_;
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
    // Declare and get sync policy parameter
    this->declare_parameter<std::string>("sync_policy", "exact");
    this->declare_parameter<double>("max_interval_duration", 100.0);
    std::string sync_policy = this->get_parameter("sync_policy").as_string();
    double max_interval_ms = this->get_parameter("max_interval_duration").as_double();

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

    if (sync_policy == "approximate") {
      sync_approx_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
          ApproxSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_);
      sync_approx_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_interval_ms * 1e6)));
      sync_approx_->registerCallback(std::bind(
          &SyncSubscriber<4>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
      RCLCPP_INFO(this->get_logger(), "Using ApproximateTime sync policy (max_interval: %.1fms)", max_interval_ms);
    } else {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
          ExactSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_);
      sync_exact_->registerCallback(std::bind(
          &SyncSubscriber<4>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
      RCLCPP_INFO(this->get_logger(), "Using ExactTime sync policy");
    }

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ApproxSyncPolicy;

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
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_approx_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 5 publishers
template <> class SyncSubscriber<5> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    // Declare and get sync policy parameter
    this->declare_parameter<std::string>("sync_policy", "exact");
    this->declare_parameter<double>("max_interval_duration", 100.0);
    std::string sync_policy = this->get_parameter("sync_policy").as_string();
    double max_interval_ms = this->get_parameter("max_interval_duration").as_double();

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
    sub5_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic5");

    if (sync_policy == "approximate") {
      sync_approx_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
          ApproxSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_);
      sync_approx_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_interval_ms * 1e6)));
      sync_approx_->registerCallback(std::bind(
          &SyncSubscriber<5>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5));
      RCLCPP_INFO(this->get_logger(), "Using ApproximateTime sync policy (max_interval: %.1fms)", max_interval_ms);
    } else {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
          ExactSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_);
      sync_exact_->registerCallback(std::bind(
          &SyncSubscriber<5>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5));
      RCLCPP_INFO(this->get_logger(), "Using ExactTime sync policy");
    }

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp>
      ApproxSyncPolicy;

  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg2,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg3,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg4,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg5) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::max({to_microseconds(msg1->extra_stamp),
                  to_microseconds(msg2->extra_stamp),
                  to_microseconds(msg3->extra_stamp),
                  to_microseconds(msg4->extra_stamp),
                  to_microseconds(msg5->extra_stamp)}));
    RCLCPP_INFO(
        this->get_logger(),
        "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index = "
        "%s, msg4_index = %s, msg5_index = %s, sync_cb_index_ = %zu",
        msg1->header.frame_id.c_str(), msg2->header.frame_id.c_str(),
        msg3->header.frame_id.c_str(), msg4->header.frame_id.c_str(),
        msg5->header.frame_id.c_str(), sync_cb_index_);
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
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub5_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_approx_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 6 publishers
template <> class SyncSubscriber<6> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    // Declare and get sync policy parameter
    this->declare_parameter<std::string>("sync_policy", "exact");
    this->declare_parameter<double>("max_interval_duration", 100.0);
    std::string sync_policy = this->get_parameter("sync_policy").as_string();
    double max_interval_ms = this->get_parameter("max_interval_duration").as_double();

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
    sub5_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic5");
    sub6_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic6");

    if (sync_policy == "approximate") {
      sync_approx_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
          ApproxSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_, *sub6_);
      sync_approx_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_interval_ms * 1e6)));
      sync_approx_->registerCallback(std::bind(
          &SyncSubscriber<6>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5, std::placeholders::_6));
      RCLCPP_INFO(this->get_logger(), "Using ApproximateTime sync policy (max_interval: %.1fms)", max_interval_ms);
    } else {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
          ExactSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_, *sub6_);
      sync_exact_->registerCallback(std::bind(
          &SyncSubscriber<6>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5, std::placeholders::_6));
      RCLCPP_INFO(this->get_logger(), "Using ExactTime sync policy");
    }

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ApproxSyncPolicy;

  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg2,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg3,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg4,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg5,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg6) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::max({to_microseconds(msg1->extra_stamp),
                  to_microseconds(msg2->extra_stamp),
                  to_microseconds(msg3->extra_stamp),
                  to_microseconds(msg4->extra_stamp),
                  to_microseconds(msg5->extra_stamp),
                  to_microseconds(msg6->extra_stamp)}));
    RCLCPP_INFO(
        this->get_logger(),
        "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index = "
        "%s, msg4_index = %s, msg5_index = %s, msg6_index = %s, "
        "sync_cb_index_ = %zu",
        msg1->header.frame_id.c_str(), msg2->header.frame_id.c_str(),
        msg3->header.frame_id.c_str(), msg4->header.frame_id.c_str(),
        msg5->header.frame_id.c_str(), msg6->header.frame_id.c_str(),
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
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub5_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub6_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_approx_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 7 publishers
template <> class SyncSubscriber<7> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    // Declare and get sync policy parameter
    this->declare_parameter<std::string>("sync_policy", "exact");
    this->declare_parameter<double>("max_interval_duration", 100.0);
    std::string sync_policy = this->get_parameter("sync_policy").as_string();
    double max_interval_ms = this->get_parameter("max_interval_duration").as_double();

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
    sub5_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic5");
    sub6_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic6");
    sub7_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic7");

    if (sync_policy == "approximate") {
      sync_approx_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
          ApproxSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_, *sub6_,
          *sub7_);
      sync_approx_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_interval_ms * 1e6)));
      sync_approx_->registerCallback(std::bind(
          &SyncSubscriber<7>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5, std::placeholders::_6, std::placeholders::_7));
      RCLCPP_INFO(this->get_logger(), "Using ApproximateTime sync policy (max_interval: %.1fms)", max_interval_ms);
    } else {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
          ExactSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_, *sub6_,
          *sub7_);
      sync_exact_->registerCallback(std::bind(
          &SyncSubscriber<7>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5, std::placeholders::_6, std::placeholders::_7));
      RCLCPP_INFO(this->get_logger(), "Using ExactTime sync policy");
    }

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp>
      ApproxSyncPolicy;

  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg2,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg3,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg4,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg5,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg6,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg7) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::max({to_microseconds(msg1->extra_stamp),
                  to_microseconds(msg2->extra_stamp),
                  to_microseconds(msg3->extra_stamp),
                  to_microseconds(msg4->extra_stamp),
                  to_microseconds(msg5->extra_stamp),
                  to_microseconds(msg6->extra_stamp),
                  to_microseconds(msg7->extra_stamp)}));
    RCLCPP_INFO(
        this->get_logger(),
        "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index = "
        "%s, msg4_index = %s, msg5_index = %s, msg6_index = %s, "
        "msg7_index = %s, sync_cb_index_ = %zu",
        msg1->header.frame_id.c_str(), msg2->header.frame_id.c_str(),
        msg3->header.frame_id.c_str(), msg4->header.frame_id.c_str(),
        msg5->header.frame_id.c_str(), msg6->header.frame_id.c_str(),
        msg7->header.frame_id.c_str(), sync_cb_index_);
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
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub5_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub6_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub7_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_approx_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 8 publishers
template <> class SyncSubscriber<8> : public rclcpp::Node {
public:
  explicit SyncSubscriber(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("sync_subscriber", options), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    // Declare and get sync policy parameter
    this->declare_parameter<std::string>("sync_policy", "exact");
    this->declare_parameter<double>("max_interval_duration", 100.0);
    std::string sync_policy = this->get_parameter("sync_policy").as_string();
    double max_interval_ms = this->get_parameter("max_interval_duration").as_double();

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
    sub5_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic5");
    sub6_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic6");
    sub7_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic7");
    sub8_ = std::make_shared<
        message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>(
        this, "topic8");

    if (sync_policy == "approximate") {
      sync_approx_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
          ApproxSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_, *sub6_,
          *sub7_, *sub8_);
      sync_approx_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_interval_ms * 1e6)));
      sync_approx_->registerCallback(std::bind(
          &SyncSubscriber<8>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
          std::placeholders::_8));
      RCLCPP_INFO(this->get_logger(), "Using ApproximateTime sync policy (max_interval: %.1fms)", max_interval_ms);
    } else {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
          ExactSyncPolicy(1000), *sub1_, *sub2_, *sub3_, *sub4_, *sub5_, *sub6_,
          *sub7_, *sub8_);
      sync_exact_->registerCallback(std::bind(
          &SyncSubscriber<8>::callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
          std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
          std::placeholders::_8));
      RCLCPP_INFO(this->get_logger(), "Using ExactTime sync policy");
    }

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  typedef message_filters::sync_policies::ExactTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp,
      custom_msg::msg::HeaderExtraStamp, custom_msg::msg::HeaderExtraStamp>
      ApproxSyncPolicy;

  void callback(const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg1,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg2,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg3,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg4,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg5,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg6,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg7,
                const custom_msg::msg::HeaderExtraStamp::ConstSharedPtr &msg8) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::max({to_microseconds(msg1->extra_stamp),
                  to_microseconds(msg2->extra_stamp),
                  to_microseconds(msg3->extra_stamp),
                  to_microseconds(msg4->extra_stamp),
                  to_microseconds(msg5->extra_stamp),
                  to_microseconds(msg6->extra_stamp),
                  to_microseconds(msg7->extra_stamp),
                  to_microseconds(msg8->extra_stamp)}));
    RCLCPP_INFO(
        this->get_logger(),
        "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index = "
        "%s, msg4_index = %s, msg5_index = %s, msg6_index = %s, "
        "msg7_index = %s, msg8_index = %s, sync_cb_index_ = %zu",
        msg1->header.frame_id.c_str(), msg2->header.frame_id.c_str(),
        msg3->header.frame_id.c_str(), msg4->header.frame_id.c_str(),
        msg5->header.frame_id.c_str(), msg6->header.frame_id.c_str(),
        msg7->header.frame_id.c_str(), msg8->header.frame_id.c_str(),
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
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub5_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub6_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub7_;
  std::shared_ptr<
      message_filters::Subscriber<custom_msg::msg::HeaderExtraStamp>>
      sub8_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_approx_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Register each template specialization as a separate component
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<1>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<2>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<3>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<4>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<5>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<6>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<7>)
RCLCPP_COMPONENTS_REGISTER_NODE(SyncSubscriber<8>)
