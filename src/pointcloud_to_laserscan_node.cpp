/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  input_qos_reliability_ = this->declare_parameter("input_qos_reliability", "best_effort");
  input_qos_durability_ = this->declare_parameter("input_qos_durability", "volatile");

  auto to_lower = [](std::string value) {
      std::transform(
        value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      return value;
    };
  input_qos_reliability_ = to_lower(input_qos_reliability_);
  input_qos_durability_ = to_lower(input_qos_durability_);
  if (input_qos_reliability_ != "best_effort" && input_qos_reliability_ != "reliable") {
    RCLCPP_WARN(
      this->get_logger(),
      "Unsupported input_qos_reliability='%s', falling back to 'best_effort'",
      input_qos_reliability_.c_str());
    input_qos_reliability_ = "best_effort";
  }
  if (input_qos_durability_ != "volatile" && input_qos_durability_ != "transient_local") {
    RCLCPP_WARN(
      this->get_logger(),
      "Unsupported input_qos_durability='%s', falling back to 'volatile'",
      input_qos_durability_.c_str());
    input_qos_durability_ = "volatile";
  }
  always_subscribe_ = this->declare_parameter("always_subscribe", false);
  auto_resubscribe_on_stall_ = this->declare_parameter("auto_resubscribe_on_stall", false);
  exit_on_cloud_stall_ = this->declare_parameter("exit_on_cloud_stall", true);
  diagnostics_timeout_sec_ = this->declare_parameter("diagnostics_timeout_sec", 2.0);
  resubscribe_cooldown_sec_ = this->declare_parameter("resubscribe_cooldown_sec", 15.0);
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  last_publish_ns_.store(this->now().nanoseconds());
  last_cloud_ns_.store(this->now().nanoseconds());

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PointCloudToLaserScanNode::diagnosticsTimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "always_subscribe=%s, auto_resubscribe_on_stall=%s, exit_on_cloud_stall=%s, diagnostics_timeout=%.1fs, "
    "resubscribe_cooldown=%.1fs, queue_size=%d, input_qos_reliability=%s, "
    "input_qos_durability=%s",
    always_subscribe_ ? "true" : "false",
    auto_resubscribe_on_stall_ ? "true" : "false",
    exit_on_cloud_stall_ ? "true" : "false",
    diagnostics_timeout_sec_,
    resubscribe_cooldown_sec_,
    input_queue_size_,
    input_qos_reliability_.c_str(),
    input_qos_durability_.c_str());

  if (always_subscribe_) {
    resubscribe();
  } else {
    subscription_listener_thread_ = std::thread(
      std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
  }
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  if (subscription_listener_thread_.joinable()) {
    subscription_listener_thread_.join();
  }
}

 rclcpp::QoS PointCloudToLaserScanNode::makeInputQos() const
 {
   rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(input_queue_size_))};
   if (input_qos_reliability_ == "reliable") {
     qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
   } else {
     qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
   }

   if (input_qos_durability_ == "transient_local") {
     qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
   } else {
     qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
   }
   return qos;
 }

void PointCloudToLaserScanNode::resubscribe()
{
  sub_.unsubscribe();

  const auto qos = makeInputQos();
  sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
  last_resubscribe_ns_.store(this->now().nanoseconds());
  RCLCPP_INFO(this->get_logger(), "cloud_in subscription (re)established");
}

void PointCloudToLaserScanNode::diagnosticsTimerCallback()
{
  if (diagnostics_timeout_sec_ <= 0.0) {
    return;
  }

  if (!always_subscribe_ && !sub_.getSubscriber()) {
    return;
  }

  const int64_t now_ns = this->now().nanoseconds();
  const double scan_age_sec =
    static_cast<double>(now_ns - last_publish_ns_.load()) / 1e9;
  const double cloud_age_sec =
    static_cast<double>(now_ns - last_cloud_ns_.load()) / 1e9;

  if (scan_age_sec <= diagnostics_timeout_sec_) {
    return;
  }

  if (cloud_age_sec > diagnostics_timeout_sec_) {
    const std::string cloud_topic = sub_.getSubscriber() ?
      sub_.getSubscriber()->get_topic_name() :
      this->get_node_topics_interface()->resolve_topic_name("cloud_in");
    const size_t cloud_publishers = this->count_publishers(cloud_topic);
    const double resubscribe_age_sec =
      static_cast<double>(now_ns - last_resubscribe_ns_.load()) / 1e9;
    const bool cooldown_ok = resubscribe_age_sec >= resubscribe_cooldown_sec_;
    const bool can_resubscribe =
      always_subscribe_ && auto_resubscribe_on_stall_ && cloud_publishers > 0 && cooldown_ok;
    const bool should_exit = exit_on_cloud_stall_ && cloud_publishers > 0;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No cloud received for %.2fs (last scan %.2fs). topic='%s', publishers=%zu, "
      "subscriber_active=%s, auto_resubscribe=%s, cooldown_ok=%s",
      cloud_age_sec,
      scan_age_sec,
      cloud_topic.c_str(),
      cloud_publishers,
      sub_.getSubscriber() ? "true" : "false",
      auto_resubscribe_on_stall_ ? "true" : "false",
      cooldown_ok ? "true" : "false");

    if (should_exit) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Cloud stall detected with active publishers on '%s'. Exiting process to let launch respawn.",
        cloud_topic.c_str());
      std::_Exit(EXIT_FAILURE);
    }

    if (can_resubscribe) {
      RCLCPP_WARN(this->get_logger(), "Stall detected with active publishers; resubscribing to cloud_in");
      resubscribe();
    }
  } else {
    // Clouds are arriving but scans are not published
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Clouds arriving (last %.2fs ago) but no LaserScan for %.2fs. "
      "Check point filter params (height/range/angle) or TF.",
      cloud_age_sec, scan_age_sec);
  }
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        const auto qos = makeInputQos();
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Received pointcloud with %d data points and frame_id=%s",
    cloud_msg->width * cloud_msg->height, cloud_msg->header.frame_id.c_str());

  last_cloud_ns_.store(this->now().nanoseconds());

  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  pub_->publish(std::move(scan_msg));
  last_publish_ns_.store(this->now().nanoseconds());
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
