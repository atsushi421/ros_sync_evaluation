#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pmu_analyzer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>

// Template specializations for different numbers of publishers
template <int N> class SyncSubscriber;

// Specialization for 1 publisher (no synchronization needed, just a simple
// subscriber)
template <> class SyncSubscriber<1> : public rclcpp::Node {
public:
  SyncSubscriber()
      : Node("sync_subscriber"), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ = this->create_subscription<std_msgs::msg::Header>(
        "topic1", 1000,
        std::bind(&SyncSubscriber<1>::callback, this, std::placeholders::_1));

    pmu_analyzer::ELAPSED_TIME_INIT(session_name_);
  }

  ~SyncSubscriber() { pmu_analyzer::ELAPSED_TIME_CLOSE(session_name_); }

private:
  void callback(const std_msgs::msg::Header::ConstSharedPtr &msg1) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(session_name_, 0, true,
                                         std::stoi(msg1->frame_id));
    RCLCPP_INFO(this->get_logger(),
                "SyncSubscriber: index = %s, sync_cb_index_ = %zu",
                msg1->frame_id.c_str(), sync_cb_index_);
    sync_cb_index_++;
  }

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub1_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 2 publishers
template <> class SyncSubscriber<2> : public rclcpp::Node {
public:
  SyncSubscriber()
      : Node("sync_subscriber"), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
            this, "topic1");
    sub2_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
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
  typedef message_filters::sync_policies::ExactTime<std_msgs::msg::Header,
                                                    std_msgs::msg::Header>
      SyncPolicy;

  void callback(const std_msgs::msg::Header::ConstSharedPtr &msg1,
                const std_msgs::msg::Header::ConstSharedPtr &msg2) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::stoi(std::max(msg1->frame_id, msg2->frame_id)));
    RCLCPP_INFO(this->get_logger(),
                "SyncSubscriber: msg1_index = %s, msg2_index = %s, "
                "sync_cb_index_ = %zu",
                msg1->frame_id.c_str(), msg2->frame_id.c_str(), sync_cb_index_);
    sync_cb_index_++;
  }

  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub1_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 3 publishers
template <> class SyncSubscriber<3> : public rclcpp::Node {
public:
  SyncSubscriber()
      : Node("sync_subscriber"), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
            this, "topic1");
    sub2_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
            this, "topic2");
    sub3_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
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
      std_msgs::msg::Header, std_msgs::msg::Header, std_msgs::msg::Header>
      SyncPolicy;

  void callback(const std_msgs::msg::Header::ConstSharedPtr &msg1,
                const std_msgs::msg::Header::ConstSharedPtr &msg2,
                const std_msgs::msg::Header::ConstSharedPtr &msg3) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::stoi(std::max({msg1->frame_id, msg2->frame_id, msg3->frame_id})));
    RCLCPP_INFO(this->get_logger(),
                "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index "
                "= %s, sync_cb_index_ = %zu",
                msg1->frame_id.c_str(), msg2->frame_id.c_str(),
                msg3->frame_id.c_str(), sync_cb_index_);
    sync_cb_index_++;
  }

  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub1_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub2_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub3_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  size_t sync_cb_index_;
  std::string session_name_;
};

// Specialization for 4 publishers
template <> class SyncSubscriber<4> : public rclcpp::Node {
public:
  SyncSubscriber()
      : Node("sync_subscriber"), sync_cb_index_(0),
        session_name_("sync_subscriber_subscribed") {
    sub1_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
            this, "topic1");
    sub2_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
            this, "topic2");
    sub3_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
            this, "topic3");
    sub4_ =
        std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(
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
      std_msgs::msg::Header, std_msgs::msg::Header, std_msgs::msg::Header,
      std_msgs::msg::Header>
      SyncPolicy;

  void callback(const std_msgs::msg::Header::ConstSharedPtr &msg1,
                const std_msgs::msg::Header::ConstSharedPtr &msg2,
                const std_msgs::msg::Header::ConstSharedPtr &msg3,
                const std_msgs::msg::Header::ConstSharedPtr &msg4) {
    pmu_analyzer::ELAPSED_TIME_TIMESTAMP(
        session_name_, 0, true,
        std::stoi(std::max(
            {msg1->frame_id, msg2->frame_id, msg3->frame_id, msg4->frame_id})));
    RCLCPP_INFO(
        this->get_logger(),
        "SyncSubscriber: msg1_index = %s, msg2_index = %s, msg3_index = "
        "%s, msg4_index = %s, sync_cb_index_ = %zu",
        msg1->frame_id.c_str(), msg2->frame_id.c_str(), msg3->frame_id.c_str(),
        msg4->frame_id.c_str(), sync_cb_index_);
    sync_cb_index_++;
  }

  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub1_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub2_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub3_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub4_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  size_t sync_cb_index_;
  std::string session_name_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Get number of publishers from command line argument or ROS parameter
  int num_publishers = 2; // default

  if (argc > 1) {
    num_publishers = std::atoi(argv[1]);
  }

  std::shared_ptr<rclcpp::Node> node;

  switch (num_publishers) {
  case 1:
    node = std::make_shared<SyncSubscriber<1>>();
    break;
  case 2:
    node = std::make_shared<SyncSubscriber<2>>();
    break;
  case 3:
    node = std::make_shared<SyncSubscriber<3>>();
    break;
  case 4:
    node = std::make_shared<SyncSubscriber<4>>();
    break;
  default:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Unsupported number of publishers: %d. Supported: 1-4",
                 num_publishers);
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
