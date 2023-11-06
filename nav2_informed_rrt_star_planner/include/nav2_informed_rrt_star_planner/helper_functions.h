/**
 * @file helper_functions.h
 *
 * @brief Helper functions
 *
 * @author Monica Perez Serrano
 * Contact: monicapserrano@outlook.com
 *
 */

#pragma once

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

geometry_msgs::msg::Quaternion yaw2quat(const double yaw_rad);
double quat2yaw(const geometry_msgs::msg::Quaternion& q);
    
std::vector<geometry_msgs::msg::Point> toPoint2D(
    const std::vector<geometry_msgs::msg::Point32>& polygon_points);