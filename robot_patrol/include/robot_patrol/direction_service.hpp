#ifndef DIRECTION_SERVICE_HPP
#define DIRECTION_SERVICE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include "direction_service_msg/srv/get_direction.hpp"

class DirectionService : public rclcpp::Node {
public:
  DirectionService();
  ~DirectionService();

private:
  rclcpp::Service<direction_service_msg::srv::GetDirection>::SharedPtr
      get_direction_service_;
  void direction_service_callback(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<direction_service_msg::srv::GetDirection::Request>
          request,
      const std::shared_ptr<direction_service_msg::srv::GetDirection::Response>
          response);

  float total_dist_sec_right{0.0}, total_dist_sec_front{0.0},
      total_dist_sec_left{0.0}, max_sum{0.0};
};

#endif