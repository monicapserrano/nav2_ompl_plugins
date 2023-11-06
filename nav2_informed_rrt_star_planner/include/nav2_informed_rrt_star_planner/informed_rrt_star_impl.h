/**
 * @file informed_rrt_star_impl.h
 *
 * @brief Implementation of Informed RRT* using OMPL library
 *
 * @author Monica Perez Serrano
 * Contact: monicapserrano@outlook.com
 *
 */

#pragma once

#include <boost/thread/mutex.hpp>
#include <optional>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace ob = ompl::base;
namespace nav2::ompl::planner::informed_rrt_star{

struct Bounds{
  double lower_x_m;
  double upper_x_m;
  double lower_y_m;
  double upper_y_m;
};

struct Pose2D{
  double x_m;
  double y_m;
  double yaw_rad;
};

class Impl {

  public:
    Impl();
    ~Impl(){};
    
    bool isStateValid(const ob::SpaceInformation* space_information, const ob::State *state);
    void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, std::vector<geometry_msgs::msg::Point> padded_footprint);
    std::vector<ob::State*> solve(const Pose2D& start, const Pose2D& goal, const double solving_time_s);

  private:
    ob::StateSpacePtr se2_state_space_;
    ob::SpaceInformationPtr space_information_;
    ob::ProblemDefinitionPtr pdef_;
    ob::PlannerPtr planner_;
    boost::mutex mu;
    ob::OptimizationObjectivePtr optimization_objective_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
    std::optional<std::vector<geometry_msgs::msg::Point>> padded_footprint_;

};
} // namespace nav2::ompl::planner::informed_rrt_star