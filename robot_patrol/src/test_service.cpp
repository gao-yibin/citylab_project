#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "direction_service_msg/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service_client") {
    service_client_ =
        this->create_client<direction_service_msg::srv::GetDirection>(
            "/direction_service");

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos,
        std::bind(&TestService::laserscan_callback, this,
                  std::placeholders::_1));
  }

  void send_request() {
    // Create an empty Trigger request
    auto request =
        std::make_shared<direction_service_msg::srv::GetDirection::Request>();

    // Send the request asynchronously
    auto result_future = service_client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();
      // Log the service response
      RCLCPP_INFO(this->get_logger(), "Direction: %s",
                  response->direction.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

private:
  void
  laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    request->laser_data = *laser_msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Client<direction_service_msg::srv::GetDirection>::SharedPtr
      service_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<TestService>();

  client_node->send_request();

  // rclcpp::spin(client_node);

  rclcpp::shutdown();
  return 0;
}