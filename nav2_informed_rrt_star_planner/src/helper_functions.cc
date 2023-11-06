/**
 * @file helper_functions.cc
 *
 * @brief Helper functions
 *
 * @author Monica Perez Serrano
 * Contact: monicapserrano@outlook.com
 *
 */

#include "nav2_informed_rrt_star_planner/helper_functions.h"

geometry_msgs::msg::Quaternion yaw2quat(const double yaw_rad){
    geometry_msgs::msg::Quaternion q{};
  q.w = cos(yaw_rad * 0.5);
  q.z = sin(yaw_rad * 0.5);
  return q;
}

double quat2yaw(const geometry_msgs::msg::Quaternion& q){
  const double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}
    
std::vector<geometry_msgs::msg::Point> toPoint2D(
    const std::vector<geometry_msgs::msg::Point32>& polygon_points){
    std::vector<geometry_msgs::msg::Point> points;
  for(auto it=polygon_points.begin(); it!=polygon_points.end(); it ++){
    points.push_back(geometry_msgs::build<geometry_msgs::msg::Point>()
                      .x(it->x).y(it->y).z(it->z));
  }
  return points;
}