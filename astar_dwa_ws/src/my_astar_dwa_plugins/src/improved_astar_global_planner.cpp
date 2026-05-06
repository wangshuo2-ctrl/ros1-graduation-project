#include "my_astar_dwa_plugins/improved_astar_global_planner.h"
#include <angles/angles.h>
#include <nav_msgs/OccupancyGrid.h>

namespace my_astar_dwa_plugins {

ImprovedAStarGlobalPlanner::ImprovedAStarGlobalPlanner() : 
    initialized_(false), 
    costmap_ros_(nullptr), 
    costmap_(nullptr) {
}

ImprovedAStarGlobalPlanner::ImprovedAStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}

void ImprovedAStarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        name_ = name;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        
        // 从参数服务器加载参数
        private_nh_ = ros::NodeHandle("~/" + name);
        loadParameters();
        
        // 初始化A*规划器
        astar_planner_.reset(new pathplanning::OAstar());
        
        // 配置A*参数（与原导航文件保持一致）
        pathplanning::OAstarConfig astar_config;
        astar_config.heuristic_weight = heuristic_weight_;
        astar_config.inflate_radius = inflate_radius_;
        astar_config.safety_margin = safety_margin_;
        astar_config.obstacle_threshold = obstacle_threshold_;
        astar_config.max_iterations = 20000;  // 与文档一致
        astar_config.euclidean = true;       // 使用欧几里得距离
        astar_config.allow_outside_map = true;
        astar_config.enable_path_smoothing = true;
        astar_config.enable_bspline_smoothing = true;
        astar_config.bspline_samples = 150;
        astar_config.bspline_smoothness = 0.6;
        astar_config.enable_safety_check = true;
        astar_config.safety_check_distance = 0.4;
        astar_config.enable_constrained_smoothing = true;
        astar_config.constrained_smoothing_iterations = 3;
        astar_config.constrained_smoothing_weight = 0.2;
        
        astar_planner_->initOAstar(astar_config);
        
        initialized_ = true;
        ROS_INFO("ImprovedAStarGlobalPlanner initialized successfully");
    }
}

void ImprovedAStarGlobalPlanner::loadParameters() {
    // 加载参数，与原导航文件保持一致
    // 默认值来自文档中的参数设置
    private_nh_.param("obstacle_threshold", obstacle_threshold_, 0.65);
    private_nh_.param("inflate_radius", inflate_radius_, 8);
    private_nh_.param("safety_margin", safety_margin_, 0.4);
    private_nh_.param("heuristic_weight", heuristic_weight_, 1.0);
    
    ROS_INFO("ImprovedAStarGlobalPlanner parameters:");
    ROS_INFO("  obstacle_threshold: %.2f", obstacle_threshold_);
    ROS_INFO("  inflate_radius: %d", inflate_radius_);
    ROS_INFO("  safety_margin: %.2f", safety_margin_);
    ROS_INFO("  heuristic_weight: %.2f", heuristic_weight_);
}

bool ImprovedAStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                         const geometry_msgs::PoseStamped& goal,
                                         std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("Planner not initialized");
        return false;
    }
    
    plan.clear();
    
    // 转换起点和终点到地图坐标系
    int start_mx, start_my, goal_mx, goal_my;
    if (!convertPoseToMap(start, start_mx, start_my) || 
        !convertPoseToMap(goal, goal_mx, goal_my)) {
        ROS_ERROR("Failed to convert poses to map coordinates");
        return false;
    }
    
    // 更新地图
    nav_msgs::OccupancyGrid grid;
    costmapToOccupancyGrid(grid);
    nav_msgs::OccupancyGrid::ConstPtr grid_ptr(new nav_msgs::OccupancyGrid(grid));
    astar_planner_->updateMap(grid_ptr);
    
    // 设置起点和终点
    cv::Point start_cv(start_mx, start_my);
    cv::Point goal_cv(goal_mx, goal_my);
    astar_planner_->setStartPoint(start_cv);
    astar_planner_->setTargetPoint(goal_cv);
    
    // 执行规划
    std::vector<cv::Point> path_points;
    if (!astar_planner_->pathPlanning(path_points)) {
        ROS_WARN("A* planning failed");
        return false;
    }
    
    // 转换路径点
    for (const cv::Point& point : path_points) {
        geometry_msgs::PoseStamped pose = convertMapToPose(point.x, point.y);
        pose.header.frame_id = global_frame_;
        pose.header.stamp = ros::Time::now();
        plan.push_back(pose);
    }
    
    ROS_INFO("ImprovedAStarGlobalPlanner generated path with %zu points", plan.size());
    return true;
}

bool ImprovedAStarGlobalPlanner::convertPoseToMap(const geometry_msgs::PoseStamped& pose, int& mx, int& my) {
    if (!costmap_) return false;
    
    double wx = pose.pose.position.x;
    double wy = pose.pose.position.y;
    
    // 使用unsigned int类型接收结果
    unsigned int umx, umy;
    if (!costmap_->worldToMap(wx, wy, umx, umy)) {
        ROS_WARN("Point (%.2f, %.2f) is outside the costmap", wx, wy);
        return false;
    }
    
    // 转换为int类型（如果您的OAstar算法需要int）
    mx = static_cast<int>(umx);
    my = static_cast<int>(umy);
    return true;
}

geometry_msgs::PoseStamped ImprovedAStarGlobalPlanner::convertMapToPose(int mx, int my) {
    geometry_msgs::PoseStamped pose;
    
    if (!costmap_) return pose;
    
    double wx, wy;
    // 将int转换为unsigned int
    costmap_->mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
    
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;  // 默认朝向
    
    return pose;
}

void ImprovedAStarGlobalPlanner::costmapToOccupancyGrid(nav_msgs::OccupancyGrid& grid) {
    grid.header.frame_id = global_frame_;
    grid.header.stamp = ros::Time::now();
    
    grid.info.width = costmap_->getSizeInCellsX();
    grid.info.height = costmap_->getSizeInCellsY();
    grid.info.resolution = costmap_->getResolution();
    
    double origin_x, origin_y;
    origin_x = costmap_->getOriginX();
    origin_y = costmap_->getOriginY();
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    grid.data.resize(grid.info.width * grid.info.height);
    
    for (unsigned int y = 0; y < grid.info.height; ++y) {
        for (unsigned int x = 0; x < grid.info.width; ++x) {
            unsigned int cost = costmap_->getCost(x, y);
            int index = y * grid.info.width + x;
            
            // 将costmap代价转换为occupancy grid值
            if (cost == costmap_2d::NO_INFORMATION) {
                grid.data[index] = -1;  // 未知
            } else if (cost == costmap_2d::LETHAL_OBSTACLE) {
                grid.data[index] = 100;  // 障碍物
            } else if (cost > 0) {
                // 根据阈值判断
                grid.data[index] = (cost > static_cast<unsigned int>(obstacle_threshold_ * 100)) ? 100 : 0;
            } else {
                grid.data[index] = 0;  // 自由空间
            }
        }
    }
}

}  // namespace my_astar_dwa_plugins

// 插件导出宏
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_astar_dwa_plugins::ImprovedAStarGlobalPlanner, nav_core::BaseGlobalPlanner)