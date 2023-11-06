/**
 * @file informed_rrt_star_planner.cc
 *
 * @brief ROS2 Global Planner Plugin of Informed RRT* using OMPL library
 *
 * @author Monica Perez Serrano
 * Contact: monicapserrano@outlook.com
 *
 */

#include "nav2_informed_rrt_star_planner/informed_rrt_star_planner.h"
#include "nav2_informed_rrt_star_planner/helper_functions.h"

#include <cmath>
#include <string>
#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>


PLUGINLIB_EXPORT_CLASS(nav2::ompl::planner::informed_rrt_star::InformedRRTStarPlanner,
                       nav2_core::GlobalPlanner)


namespace ob = ompl::base;
namespace og = ompl::geometric;

using std::placeholders::_1;

namespace nav2::ompl::planner::informed_rrt_star{

void InformedRRTStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  RCLCPP_DEBUG(node_->get_logger(), "[InformedRRTStar] Calling configure.");
  name_ = name;
  padded_footprint_ = toPoint2D(costmap_ros->getRobotFootprintPolygon().points);
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".solving_time_s", rclcpp::ParameterValue(
      5.0));
  node_->get_parameter(name_ + ".solving_time_s", solving_time_s_);
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap =
      std::make_shared<nav2_costmap_2d::Costmap2D>(*costmap_ros->getCostmap());
  impl_.configure(costmap_ros, toPoint2D(costmap_ros->getRobotFootprintPolygon().points));
}

nav_msgs::msg::Path InformedRRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  RCLCPP_DEBUG(node_->get_logger(), "[InformedRRTStar] Start frame_id %s.",
    start.header.frame_id.c_str());
  RCLCPP_DEBUG(node_->get_logger(), "[InformedRRTStar] Goal frame_id %s.",
    goal.header.frame_id.c_str());

  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), 
        "[InformedRRTStar] Planner will only except start position from %s frame",
        global_frame_.c_str());
    return nav_msgs::msg::Path{};
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(),
        "[InformedRRTStar] Planner will only except goal position from %s frame",
        global_frame_.c_str());
    return nav_msgs::msg::Path{};
  }
  const Pose2D start_pose{start.pose.position.x, start.pose.position.y,
                          quat2yaw(start.pose.orientation)};
  const Pose2D goal_pose{goal.pose.position.x, goal.pose.position.y,
                         quat2yaw(goal.pose.orientation)};
  std::vector<ob::State*> path = impl_.solve(start_pose, goal_pose, solving_time_s_);

  nav_msgs::msg::Path global_path{};

  for (std::size_t path_idx = 0; path_idx < path.size(); path_idx++){
    const ob::SE2StateSpace::StateType* se2state = path[path_idx]
                                                  ->as<ob::SE2StateSpace::StateType>();
    geometry_msgs::msg::PoseStamped pose_stamped_msg{};
    pose_stamped_msg.header.stamp = node_->now();
    pose_stamped_msg.pose.position.x = se2state->getX();
    pose_stamped_msg.pose.position.y = se2state->getY();
    pose_stamped_msg.pose.orientation = yaw2quat(se2state->getYaw());
    global_path.poses.push_back(pose_stamped_msg);
  }
  
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  return global_path;
}
}

