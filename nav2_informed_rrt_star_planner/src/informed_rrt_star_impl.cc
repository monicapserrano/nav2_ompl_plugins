/**
 * @file informed_rrt_star_impl.cc
 *
 * @brief Implementation of Informed RRT* using OMPL library
 *
 * @author Monica Perez Serrano
 * Contact: monicapserrano@outlook.com
 *
 */

#include "nav2_informed_rrt_star_planner/informed_rrt_star_impl.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using std::placeholders::_1;

namespace nav2::ompl::planner::informed_rrt_star{

const bool ALLOW_PLANNING_IN_UNKNOWN_SPACE = true;


Impl::Impl(): se2_state_space_((new ob::SE2StateSpace())),
              space_information_(new ob::SpaceInformation(se2_state_space_)),
              pdef_(new ob::ProblemDefinition(space_information_)),
              planner_(new og::InformedRRTstar(space_information_)),
              optimization_objective_(new ob::PathLengthOptimizationObjective(space_information_)){

}

bool Impl::isStateValid(
      const ob::SpaceInformation *space_information_, const ob::State *state){
    const double kFreeSpaceCost = 0;
    ob::ScopedState<ob::SE2StateSpace> se2state(space_information_->getStateSpace());
    se2state = state;
    if(costmap_ == nullptr || !padded_footprint_.has_value()){
      std::cerr << "[InformedRRTStar] " << std::endl;
      throw std::logic_error("[InformedRRTStar] The planner has not been configured, "
                             "please call 'Impl::configure'."); 
    }
    nav2_costmap_2d::FootprintCollisionChecker footprint_collision_checker(costmap_->getCostmap());
    double cost = footprint_collision_checker.footprintCostAtPose(
      se2state->getX(), se2state->getY(),
      se2state->getYaw(), padded_footprint_.value());
    const bool result = cost == kFreeSpaceCost
      || (cost < kFreeSpaceCost && ALLOW_PLANNING_IN_UNKNOWN_SPACE);
    return result;
}

void Impl::configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
                     std::vector<geometry_msgs::msg::Point> padded_footprint){
  std::cout << "[InformedRRTStar] Computing OMPL path." << std::endl;
  costmap_ = costmap;
  padded_footprint_ = padded_footprint;
  ob::RealVectorBounds bounds(/* dimension */ 2);
  bounds.setLow(0, costmap->getCostmap()->getOriginX());
	bounds.setHigh(0, costmap->getCostmap()->getOriginX() + costmap->getCostmap()->getSizeInMetersX());
	bounds.setLow(1, costmap->getCostmap()->getOriginY());
	bounds.setHigh(1, costmap->getCostmap()->getOriginY() + costmap->getCostmap()->getSizeInMetersY());
  se2_state_space_->as<ob::SE2StateSpace>()->setBounds(bounds);
  space_information_->setStateValidityChecker(std::bind(
    &Impl::isStateValid, this, space_information_.get(), _1));

  pdef_->setOptimizationObjective(optimization_objective_);

}
std::vector<ob::State*> Impl::solve(const Pose2D& start, const Pose2D& goal, const double solving_time_s){
  mu.lock();
  ob::ScopedState<> start_state(se2_state_space_), goal_state(se2_state_space_);
  start_state = {start.x_m, start.y_m, start.yaw_rad, 0.0};
  goal_state = {goal.x_m, goal.y_m, goal.yaw_rad, 0.0};

  pdef_->setStartAndGoalStates(start_state, goal_state);
  planner_->clear();
  planner_->setProblemDefinition(pdef_);
  planner_->setup();
  ob::PlannerStatus solved = planner_->solve(solving_time_s);

  if(!solved){
    std::cerr << "[InformedRRTStar] The planner failed to find a feasible "
                           "time in a computing time of " << solving_time_s
                           << " seconds." << std::endl;
    return {};
  }
  std::cout << "[InformedRRTStar] Ompl finished planning."  << std::endl;

	og::PathGeometric& path = *pdef_->getSolutionPath()->as<og::PathGeometric>();
  og::PathSimplifier smooth_path(space_information_, pdef_->getGoal(), optimization_objective_);
  smooth_path.smoothBSpline(path);
  
  mu.unlock();
  return path.getStates();
}
}  // namespace nav2::ompl::planner::informed_rrt_star