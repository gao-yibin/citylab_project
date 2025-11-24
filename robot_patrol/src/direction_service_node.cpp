#include "robot_patrol/direction_service.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto direction_service_server = std::make_shared<DirectionService>();

  rclcpp::spin(direction_service_server);

  rclcpp::shutdown();
  return 0;
}