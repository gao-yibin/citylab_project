#include "robot_patrol/patrol.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto robot_patrol_node = std::make_shared<Patrol>();

  // Use MultiThreadedExecutor with 3 threads
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    3);
  executor.add_node(robot_patrol_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}