#ifndef PATROL_HPP
#define PATROL_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

class Patrol : public rclcpp::Node {
public:
  Patrol();
  ~Patrol();

private:
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_;

  // Timer
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;

  // Laser Scan Part
  void
  laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;

  std::vector<float>::iterator it;
  float min_distance, max_distance;
  const float angle_increment_;
  int index, case_index;
  bool direction__update_lock, direction__yaw_shift_alart;

  // Odometry Part
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  double direction_, direction__yaw_, yaw_error_;

  // Twist Publisher Part
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  geometry_msgs::msg::Twist twist_msg{};

  double roll, pitch, current_yaw_;

  std::mutex mutex_;
  std::condition_variable condition_;
  bool callback1_done_;
};

#endif