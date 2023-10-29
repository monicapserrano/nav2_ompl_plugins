/**
 * @file informed_rrt_star_planner.h
 *
 * @brief ROS2 Global Planner Plugin of Informed RRT* using OMPL library
 *
 * @author Monica Perez Serrano
 * Contact: monicapserrano@outlook.com
 *
 */


#pragma once

#include <boost/thread/mutex.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <ompl/base/SpaceInformation.h>
#include <rclcpp/rclcpp.hpp>

namespace ob = ompl::base;
namespace nav2_ompl_planner{

class InformedRRTStarPlanner: public nav2_core::GlobalPlanner{

  public:
    InformedRRTStarPlanner(){};
    ~InformedRRTStarPlanner(){}

    void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    inline void cleanup(){
      RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type InformedRRTStarPlanner",
        name_.c_str());}

    inline void activate(){
      RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type InformedRRTStarPlanner",
        name_.c_str());}

    inline void deactivate(){
      RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type InformedRRTStarPlanner",
        name_.c_str());}

    nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  private:
    bool isStateValid(const ob::SpaceInformation* space_information, const ob::State *state);
    double solving_time_s_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::vector<geometry_msgs::msg::Point> padded_footprint_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D* costmap_;
    std::string global_frame_, name_;

    double interpolation_resolution_;

    const bool ALLOW_PLANNING_IN_UNKNOWN_SPACE = true;

    boost::mutex mu;
};
}
