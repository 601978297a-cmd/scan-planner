#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

using SteadyClock = std::chrono::steady_clock;
using PointCloud = sensor_msgs::msg::PointCloud2;

struct PendingCloud
{
  PointCloud::ConstSharedPtr message;
  SteadyClock::time_point received_at;
};

class SensorPoseAdapter : public rclcpp::Node
{
public:
  SensorPoseAdapter()
  : Node("sensor_pose_adapter"),
    target_frame_(declare_parameter<std::string>("target_frame", "map")),
    source_frame_(declare_parameter<std::string>("source_frame", "rslidar_front")),
    cloud_topic_(declare_parameter<std::string>("cloud_topic", "/rslidar_points_front")),
    output_topic_(declare_parameter<std::string>("output_topic", "/scan/sensor_pose")),
    cloud_stamp_topic_(
      declare_parameter<std::string>("cloud_stamp_topic", "/scan/front_cloud_stamp")),
    synced_cloud_topic_(
      declare_parameter<std::string>("synced_cloud_topic", "/scan/front_cloud_synced")),
    max_tf_wait_sec_(declare_parameter<double>("max_tf_wait_sec", 0.5)),
    retry_period_sec_(declare_parameter<double>("retry_period_sec", 0.02)),
    max_queue_size_(declare_parameter<int64_t>("max_queue_size", 16)),
    tf_buffer_(get_clock())
  {
    if (max_tf_wait_sec_ < 0.0) {
      throw std::invalid_argument("max_tf_wait_sec must be nonnegative");
    }
    if (retry_period_sec_ <= 0.0) {
      throw std::invalid_argument("retry_period_sec must be positive");
    }
    if (max_queue_size_ <= 0) {
      throw std::invalid_argument("max_queue_size must be positive");
    }

    cloud_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    match_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      tf_buffer_, this, true);

    const auto reliable_pair_qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, reliable_pair_qos);
    cloud_stamp_pub_ = create_publisher<std_msgs::msg::Header>(
      cloud_stamp_topic_, reliable_pair_qos);
    synced_cloud_pub_ = create_publisher<PointCloud>(
      synced_cloud_topic_, reliable_pair_qos);

    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;
    cloud_sub_ = create_subscription<PointCloud>(
      cloud_topic_,
      reliable_pair_qos,
      std::bind(&SensorPoseAdapter::cloud_callback, this, std::placeholders::_1),
      cloud_options);

    retry_timer_ = create_wall_timer(
      std::chrono::duration<double>(retry_period_sec_),
      std::bind(&SensorPoseAdapter::process_pending_clouds, this),
      match_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Synchronizing %s with TF %s->%s; publishing %s and %s",
      cloud_topic_.c_str(),
      target_frame_.c_str(),
      source_frame_.c_str(),
      output_topic_.c_str(),
      synced_cloud_topic_.c_str());
  }

private:
  void cloud_callback(PointCloud::ConstSharedPtr message)
  {
    std::shared_ptr<PendingCloud> dropped;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (pending_clouds_.size() >= static_cast<std::size_t>(max_queue_size_)) {
        dropped = pending_clouds_.front();
        pending_clouds_.pop_front();
      }
      pending_clouds_.push_back(
        std::make_shared<PendingCloud>(PendingCloud{std::move(message), SteadyClock::now()}));
    }

    if (dropped) {
      warn_drop(*dropped->message, "queue full");
    }
  }

  void process_pending_clouds()
  {
    while (rclcpp::ok()) {
      std::shared_ptr<PendingCloud> pending;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_clouds_.empty()) {
          return;
        }
        pending = pending_clouds_.front();
      }

      if (std::chrono::duration<double>(SteadyClock::now() - pending->received_at).count() >
        max_tf_wait_sec_)
      {
        if (remove_if_front(pending)) {
          warn_drop(*pending->message, "TF wait timeout");
        }
        continue;
      }

      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_.lookupTransform(
          target_frame_,
          source_frame_,
          rclcpp::Time(pending->message->header.stamp),
          rclcpp::Duration::from_seconds(0.0));
      } catch (const tf2::TransformException & exception) {
        const auto & stamp = pending->message->header.stamp;
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "TF lookup pending for cloud at %d.%09u: %s",
          stamp.sec,
          stamp.nanosec,
          exception.what());
        return;
      }

      if (!remove_if_front(pending)) {
        continue;
      }
      publish_synced_pair(pending->message, transform);
    }
  }

  bool remove_if_front(const std::shared_ptr<PendingCloud> & pending)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (pending_clouds_.empty() || pending_clouds_.front() != pending) {
      return false;
    }
    pending_clouds_.pop_front();
    return true;
  }

  void publish_synced_pair(
    const PointCloud::ConstSharedPtr & cloud,
    const geometry_msgs::msg::TransformStamped & transform)
  {
    std_msgs::msg::Header cloud_stamp;
    cloud_stamp.stamp = cloud->header.stamp;
    cloud_stamp.frame_id = cloud->header.frame_id;

    nav_msgs::msg::Odometry pose;
    pose.header.stamp = cloud->header.stamp;
    pose.header.frame_id = target_frame_;
    pose.child_frame_id = source_frame_;
    pose.pose.pose.position.x = transform.transform.translation.x;
    pose.pose.pose.position.y = transform.transform.translation.y;
    pose.pose.pose.position.z = transform.transform.translation.z;
    pose.pose.pose.orientation = transform.transform.rotation;

    cloud_stamp_pub_->publish(cloud_stamp);
    pose_pub_->publish(pose);
    synced_cloud_pub_->publish(*cloud);
  }

  void warn_drop(const PointCloud & cloud, const char * reason)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Dropping cloud at %d.%09u: %s",
      cloud.header.stamp.sec,
      cloud.header.stamp.nanosec,
      reason);
  }

  const std::string target_frame_;
  const std::string source_frame_;
  const std::string cloud_topic_;
  const std::string output_topic_;
  const std::string cloud_stamp_topic_;
  const std::string synced_cloud_topic_;
  const double max_tf_wait_sec_;
  const double retry_period_sec_;
  const int64_t max_queue_size_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr match_callback_group_;
  rclcpp::Subscription<PointCloud>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr cloud_stamp_pub_;
  rclcpp::Publisher<PointCloud>::SharedPtr synced_cloud_pub_;

  std::mutex queue_mutex_;
  std::deque<std::shared_ptr<PendingCloud>> pending_clouds_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorPoseAdapter>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
