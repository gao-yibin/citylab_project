#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "direction_service_msg/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
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

    constexpr std::chrono::milliseconds TIMER_PERIOD{100};
    timer_ = this->create_wall_timer(TIMER_PERIOD,
                                     std::bind(&TestService::timer_cb, this));
  }

  void timer_cb() {
    auto request =
        std::make_shared<direction_service_msg::srv::GetDirection::Request>();
    std::call_once(wait_once_flag_, [this] {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return callback1_done_; });
    });

    RCLCPP_INFO(this->get_logger(), "angle_incrememt: %f",
                laser_data.angle_increment);
    request->laser_data = laser_data;

    // Fire the request asynchronously
    auto result_future = service_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Request sent. ");

    // Attach a continuation that runs on the node’s executor
    if (result_future.get()) {
      // Log the service response
      auto response = std::make_shared<
          direction_service_msg::srv::GetDirection::Response>();
      RCLCPP_INFO(this->get_logger(), "Direction: %s",
                  response->direction.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Waiting for the response");
    }
  }

private:
  void
  laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    laser_data = *laser_msg;
    RCLCPP_INFO(this->get_logger(), "angle_incrememt: %f",
                laser_msg->angle_increment);
    RCLCPP_INFO(this->get_logger(), "angle_incrememt: %f",
                laser_data.angle_increment);

    std::call_once(set_once_flag_, [this] {
      std::lock_guard<std::mutex> lock(mutex_);
      callback1_done_ = true;
      condition_.notify_one();
    });
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Client<direction_service_msg::srv::GetDirection>::SharedPtr
      service_client_;
  sensor_msgs::msg::LaserScan laser_data{};

  rclcpp::TimerBase::SharedPtr timer_;

  // Guard lock
  std::mutex mutex_;
  std::condition_variable condition_;
  bool callback1_done_{false};

  std::once_flag wait_once_flag_;
  std::once_flag set_once_flag_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<TestService>();

  rclcpp::spin(client_node);

  rclcpp::shutdown();
  return 0;
}