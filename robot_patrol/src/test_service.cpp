#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>

#include "direction_service_msg/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using GetDirection = direction_service_msg::srv::GetDirection;

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service_client") {
    // ---------- Service client ----------
    client_ = this->create_client<GetDirection>("/direction_service");

    // ---------- LaserScan subscriber ----------
    // We only need the newest scan → keep depth = 1, best‑effort is fine for a
    // stream
    auto sub_qos = rclcpp::QoS(1)
                       .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                       .durability(rclcpp::DurabilityPolicy::Volatile);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", sub_qos,
        std::bind(&TestService::laserCallback, this, std::placeholders::_1));

    // ---------- Periodic timer ----------
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&TestService::timerCallback, this));
  }

private:
  // ----- Callback that stores the *latest* scan -----
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    latest_scan_ = *msg;        // copy once (vector allocations are cheap)
    new_scan_available_ = true; // flag for the timer
    cv_.notify_one();           // wake the timer if it is waiting
  }

  // ----- Timer that builds and sends the service request -----
  void timerCallback() {
    // Wait for the first scan only (non‑blocking after it arrives)
    {
      std::unique_lock<std::mutex> lk(data_mtx_);
      if (!new_scan_available_) {
        // Block *at most* 50 ms – we don’t want to stall the executor forever
        cv_.wait_for(lk, std::chrono::milliseconds(50),
                     [this] { return new_scan_available_; });
      }
      // If we still have nothing, just skip this cycle
      if (!new_scan_available_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No laser scan received yet. Skipping request");
        return;
      }
      // Move the data out of the protected region (copy is fine)
      request_scan_ = latest_scan_;
      new_scan_available_ = false; // reset flag for the next cycle
    }

    // ------- Build request -------
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data =
        request_scan_; // copy into the request (ROS2 does deep copy)

    // ------- Make sure the service exists -------
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Direction service not available. Will retry next timer tick");
      return;
    }

    // ------- Send request asynchronously -------
    auto future = client_->async_send_request(request);
    // Try to retrieve the result without blocking longer than, say, 10 ms
    if (future.wait_for(std::chrono::milliseconds(10)) ==
        std::future_status::ready) {
      auto response_ptr = future.get();
      RCLCPP_INFO(this->get_logger(), "Direction response: %s",
                  response_ptr->direction.c_str());
    } else {
      // Not ready yet – we simply skip this tick; the next timer will try again
      RCLCPP_DEBUG(this->get_logger(),
                   "Service response not ready yet. Will retry on next timer");
    }

    // NOTE: we deliberately **do NOT block** here. The lambda above will be
    // invoked when the response arrives, allowing the timer thread to continue.
  }

  // ----- Member variables -----
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data protection
  std::mutex data_mtx_;
  std::condition_variable cv_;
  sensor_msgs::msg::LaserScan latest_scan_;  // protected by data_mtx_
  sensor_msgs::msg::LaserScan request_scan_; // copy prepared for the request
  bool new_scan_available_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestService>();

  // MultiThreadedExecutor is fine – it lets the timer, subscription,
  // and the service‑response lambda run on separate threads.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}