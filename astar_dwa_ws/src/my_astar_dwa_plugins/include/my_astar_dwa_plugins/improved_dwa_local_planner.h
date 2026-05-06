#ifndef IMPROVED_DWA_LOCAL_PLANNER_H
#define IMPROVED_DWA_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>  // 修改为tf
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "pathplanning/DWAPlanner.h"

namespace my_astar_dwa_plugins {

class ImprovedDWALocalPlanner : public nav_core::BaseLocalPlanner {
public:
    ImprovedDWALocalPlanner();
    
    // 必须实现的接口 - 修改为使用tf::TransformListener
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;
    
private:
    bool initialized_;
    tf::TransformListener* tf_;  // 修改为tf
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::string global_frame_;
    std::string robot_base_frame_;
    
    // 改进DWA规划器
    std::shared_ptr<pathplanning::DWAPlanner> dwa_planner_;
    
    // 数据
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::Twist robot_velocity_;
    
    // ROS节点
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber odom_sub_;
    
    // 辅助函数
    void loadParameters();
    bool getRobotPose(geometry_msgs::PoseStamped& pose);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void costmapToOccupancyGrid(nav_msgs::OccupancyGrid& grid);
};

}  // namespace my_astar_dwa_plugins

#endif