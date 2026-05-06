#ifndef IMPROVED_ASTAR_GLOBAL_PLANNER_H
#define IMPROVED_ASTAR_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "pathplanning/OAstar.h"

namespace my_astar_dwa_plugins {

class ImprovedAStarGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    ImprovedAStarGlobalPlanner();
    ImprovedAStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    
    // 必须实现的接口
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;
    
private:
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    std::string global_frame_;
    std::string name_;
    
    // 改进A*规划器
    std::shared_ptr<pathplanning::OAstar> astar_planner_;
    
    // ROS节点
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 参数（与原导航系统保持一致）
    double obstacle_threshold_;      // 障碍物阈值 0.65
    int inflate_radius_;            // 膨胀半径 8
    double safety_margin_;          // 安全边界 0.4
    double heuristic_weight_;       // 启发式权重 1.0
    
    // 辅助函数
    void loadParameters();
    bool convertPoseToMap(const geometry_msgs::PoseStamped& pose, int& mx, int& my);
    geometry_msgs::PoseStamped convertMapToPose(int mx, int my);
    void costmapToOccupancyGrid(nav_msgs::OccupancyGrid& grid);
};

}  // namespace my_astar_dwa_plugins

#endif