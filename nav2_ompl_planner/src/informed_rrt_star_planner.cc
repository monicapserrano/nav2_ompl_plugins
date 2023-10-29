/**
 * @file informed_rrt_star_planner.cc
 *
 * @brief ROS2 Global Planner Plugin of Informed RRT* using OMPL library
 *
 * @author Monica Perez Serrano
 * Contact: monicapserrano@outlook.com
 *
 */

#include "nav2_ompl_planner/informed_rrt_star_planner.h"

#include <cmath>
#include <string>
#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>


PLUGINLIB_EXPORT_CLASS(nav2_ompl_planner::InformedRRTStarPlanner, nav2_core::GlobalPlanner)


namespace ob = ompl::base;
namespace og = ompl::geometric;

using std::placeholders::_1;

namespace nav2_ompl_planner{

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

bool InformedRRTStarPlanner::isStateValid(
      const ob::SpaceInformation *space_information, const ob::State *state){
    const double kFreeSpaceCost = 0;
    const ob::SE2StateSpace::StateType* se2state = 
      state->as<ob::SE2StateSpace::StateType>();
    nav2_costmap_2d::FootprintCollisionChecker footprint_collision_checker(costmap_);
    double cost = footprint_collision_checker.footprintCostAtPose(
      se2state->getX(), se2state->getY(),
      se2state->getYaw(), padded_footprint_);
    const bool result = cost == kFreeSpaceCost
      || (cost < kFreeSpaceCost && ALLOW_PLANNING_IN_UNKNOWN_SPACE);
    return result;
}

void InformedRRTStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{

  std::cout << "[InformedRRTStar] Calling configure." << std::endl;
  node_ = parent.lock();
  name_ = name;
  tf_ = tf; // Not used for now
  padded_footprint_ = toPoint2D(costmap_ros->getRobotFootprintPolygon().points);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".solving_time_s", rclcpp::ParameterValue(
      5.0));
  node_->get_parameter(name_ + ".solving_time_s", solving_time_s_);
}

nav_msgs::msg::Path InformedRRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  RCLCPP_DEBUG(node_->get_logger(), "[InformedRRTStar] Start frame_id %s.",
    start.header.frame_id.c_str());
  RCLCPP_DEBUG(node_->get_logger(), "[InformedRRTStar] Goal frame_id %s.",
    goal.header.frame_id.c_str());

  // Checking if the goal and start state is in the global frame
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
  
  mu.lock();

  RCLCPP_DEBUG(node_->get_logger(), "[InformedRRTStar] Computing OMPL path.");
  ob::StateSpacePtr se2_state_space(new ob::SE2StateSpace());
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, costmap_->getOriginX());
	bounds.setHigh(0, costmap_->getOriginX() + costmap_->getSizeInMetersX());
	bounds.setLow(1, costmap_->getOriginY());
	bounds.setHigh(1, costmap_->getOriginY() + costmap_->getSizeInMetersY());
  se2_state_space->as<ob::SE2StateSpace>()->setBounds(bounds);

  ob::SpaceInformationPtr space_information(new ob::SpaceInformation(se2_state_space));
  space_information->setStateValidityChecker(std::bind(
    &InformedRRTStarPlanner::isStateValid, this, space_information.get(), _1));

  ob::ScopedState<> start_state(se2_state_space), goal_state(se2_state_space);
  start_state = {start.pose.position.x, start.pose.position.y,
                 quat2yaw(start.pose.orientation), 0.0};
  goal_state = {goal.pose.position.x, goal.pose.position.y,
                quat2yaw(goal.pose.orientation)};

  ob::OptimizationObjectivePtr length_objective(
    new ob::PathLengthOptimizationObjective(space_information));

  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(space_information));
  pdef->setStartAndGoalStates(start_state, goal_state);
  pdef->setOptimizationObjective(length_objective);
  ob::PlannerPtr planner(new og::InformedRRTstar(space_information));
  planner->setProblemDefinition(pdef);
  planner->setup();
  ob::PlannerStatus solved = planner->solve(solving_time_s_);

  if(!solved){
    RCLCPP_ERROR(
      node_->get_logger(), "[InformedRRTStar] The planner failed to find a feasible "
                           "time in a computing time of %f seconds.", solving_time_s_);
    return nav_msgs::msg::Path{};
  }
  RCLCPP_INFO(node_->get_logger(), "[InformedRRTStar] Ompl finished planning.");

  ob::PathPtr path = pdef->getSolutionPath();
	og::PathGeometric* geometric_path = pdef->getSolutionPath()->as<og::PathGeometric>();
  og::PathSimplifier smooth_path(space_information, pdef->getGoal(), length_objective);
  smooth_path.smoothBSpline(*geometric_path);

  nav_msgs::msg::Path global_path{};

  for (std::size_t path_idx = 0; path_idx < geometric_path->getStateCount(); path_idx++){
    const ob::SE2StateSpace::StateType* se2state = geometric_path->getState(path_idx)
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
  
  mu.unlock();

  return global_path;
}
}

