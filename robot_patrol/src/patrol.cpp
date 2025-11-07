#include "robot_patrol/patrol.hpp"
#include <cmath>

Patrol::Patrol()
    : Node("robot_patrol_node"), min_distance(0.0), max_distance(0.0),
      angle_increment_(0.03157376870512962), index(0), case_index(0),
      direction__update_lock(false), direction__yaw_shift_alart(false),
      direction_(0.0), direction__yaw_(0.0), yaw_error_(0.0), roll(0.0),
      pitch(0.0), current_yaw_(0.0), callback1_done_(false) {

  twist_msg.linear.x = 0.1;

  // Create a reentrant callback group
  reentrant_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mutually_exclusive_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Set up subscription options to use the reentrant callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = reentrant_group_;

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot_1/odom", 10,
      std::bind(&Patrol::odometry_callback, this, std::placeholders::_1),
      sub_options);

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
  laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/fastbot_1/scan", qos,
      std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1),
      sub_options);

  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/fastbot_1/cmd_vel", 10);

  // Timer
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100), // timer interval
                              std::bind(&Patrol::timer_callback, this),
                              rclcpp::CallbackGroup::SharedPtr(
                                  mutually_exclusive_group_)); // timer callback

  RCLCPP_INFO(this->get_logger(), "robot_patrol_node Ready...");
}

Patrol::~Patrol() {}

void Patrol::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Extract yaw from quaternion
  tf2::Quaternion q(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, current_yaw_);
  callback1_done_ = true;
  condition_.notify_one();
}

void Patrol::laserscan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
  std::unique_lock<std::mutex> lock(mutex_);

  min_distance = *std::min_element(laser_msg->ranges.begin() + 99,
                                   laser_msg->ranges.begin() + 99);

  condition_.wait(lock, [this] { return callback1_done_; });

  if (min_distance > 0.35 &&
      !direction__update_lock) { // Just forward if nothing happened
    twist_msg.angular.z = 0.0;
  } else if (min_distance <= 0.35 &&
             !direction__update_lock) { // Obstacles front detected, update
                                        // status just once
    direction__update_lock =
        true; // Temporarily pause the update for the it, index and direction_
              // until the current yaw error is eliminated

    it = std::max_element(
        laser_msg->ranges.begin() + 50, laser_msg->ranges.begin() + 150,
        [](float a, float b) {
          // Replace 'inf' values with '-inf' for comparison purposes
          return (std::isfinite(a) ? a
                                   : -std::numeric_limits<float>::infinity()) <
                 (std::isfinite(b) ? b
                                   : -std::numeric_limits<float>::infinity());
        });
    index = std::distance(laser_msg->ranges.begin() + 50, it);

    RCLCPP_INFO(this->get_logger(), "Maximum distancd: %f at %d", *it, index);

    direction__yaw_ = (index - 50) * angle_increment_ +
                      current_yaw_; // direction_yaw_ from global current_yaw_,
                                    // fixed each time
    direction_ =
        direction__yaw_ - current_yaw_; // The angle from the front X axis
    if (std::abs(direction__yaw_) >
        M_PI) { // If the abs of direction__yaw_ is larger than pi then it means
                // the robot is going to rotate pass the shifting yaw
      direction__yaw_shift_alart = true;
    }

    RCLCPP_INFO(this->get_logger(), "Direction yaw: %f, Current yaw: %f",
                direction__yaw_, current_yaw_);
  } else { // Turning and dealing with the shifting at the edge

    if (direction__yaw_shift_alart && std::abs(current_yaw_) > 2.9) {
      if (current_yaw_ * direction__yaw_ <= 0) {
        if (direction__yaw_ >= 0) {
          direction__yaw_ = std::fmod(direction__yaw_ + M_PI, 2 * M_PI) - M_PI;
          direction__yaw_shift_alart = false;
        } else {
          direction__yaw_ = direction__yaw_ + 2 * M_PI;
          direction__yaw_shift_alart = false;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Close to the shift point! ");
    }

    yaw_error_ = direction__yaw_ - current_yaw_;

    // RCLCPP_INFO(this->get_logger(), "DY: %f, C: %f, YE: %f", direction__yaw_,
    //            current_yaw_, yaw_error_);

    // If the yaw error is significant, rotate towards the target
    if (std::abs(yaw_error_) > 0.2) // 0.05 radians threshold for orientation
    {
      twist_msg.angular.z = direction_ / 2;
    } else {
      direction__update_lock =
          false; // Release the lock for updating the yaw error calculation
      twist_msg.angular.z = 0.0;
    }
  }
}

void Patrol::timer_callback() { twist_publisher_->publish(twist_msg); }