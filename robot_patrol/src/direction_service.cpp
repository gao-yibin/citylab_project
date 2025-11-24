#include "robot_patrol/direction_service.hpp"
#include <algorithm>

DirectionService::DirectionService() : Node("direction_service_server") {
  get_direction_service_ =
      this->create_service<direction_service_msg::srv::GetDirection>(
          "/direction_service",
          std::bind(&DirectionService::direction_service_callback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));
}

DirectionService::~DirectionService() {}

void DirectionService::direction_service_callback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<direction_service_msg::srv::GetDirection::Request>
        request,
    const std::shared_ptr<direction_service_msg::srv::GetDirection::Response>
        response) {
  if (*std::min_element(request->laser_data.ranges.begin() + 100,
                        request->laser_data.ranges.begin() + 100) < 0.35) {

    total_dist_sec_right = std::accumulate(
        request->laser_data.ranges.begin() + 50,
        request->laser_data.ranges.begin() + 83, 0.0f,
        [](float acc, float val) -> float {
          return std::isfinite(val) ? acc + val : acc; // ignore inf/nan
        });
    total_dist_sec_front =
        std::accumulate(request->laser_data.ranges.begin() + 84,
                        request->laser_data.ranges.begin() + 127, 0.0f,
                        [](float acc, float val) -> float {
                          return std::isfinite(val) ? acc + val : acc;
                        });
    total_dist_sec_left =
        std::accumulate(request->laser_data.ranges.begin() + 128,
                        request->laser_data.ranges.begin() + 150, 0.0f,
                        [](float acc, float val) -> float {
                          return std::isfinite(val) ? acc + val : acc;
                        });
    max_sum = std::max(
        {total_dist_sec_right, total_dist_sec_front, total_dist_sec_left});

    if (max_sum == total_dist_sec_right) {
      response->direction = "right";
    } else if (max_sum == total_dist_sec_front) {
      response->direction = "front";
    } else {
      response->direction = "left";
    }
  }
}