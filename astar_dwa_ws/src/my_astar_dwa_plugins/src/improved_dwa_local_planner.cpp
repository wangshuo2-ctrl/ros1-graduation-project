#include "my_astar_dwa_plugins/improved_dwa_local_planner.h"
#include <nav_msgs/Odometry.h>
#include <functional>

namespace my_astar_dwa_plugins {

ImprovedDWALocalPlanner::ImprovedDWALocalPlanner() : 
    initialized_(false), 
    tf_(nullptr), 
    costmap_ros_(nullptr) {
}

// 修改初始化函数，从tf2改为tf
void ImprovedDWALocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        global_frame_ = costmap_ros->getGlobalFrameID();
        robot_base_frame_ = costmap_ros->getBaseFrameID();
        
        // 加载参数
        private_nh_ = ros::NodeHandle("~/" + name);
        loadParameters();
        
        // 初始化DWA规划器
        dwa_planner_.reset(new pathplanning::DWAPlanner());
        
        // 配置DWA参数（与原导航文件保持一致）
        pathplanning::DWAParams dwa_params;
        
        // 速度限制参数
        private_nh_.param("max_vel_x", dwa_params.max_vel_x, 0.22);
        private_nh_.param("min_vel_x", dwa_params.min_vel_x, 0.0);
        private_nh_.param("max_vel_theta", dwa_params.max_vel_theta, 2.75);
        private_nh_.param("min_vel_theta", dwa_params.min_vel_theta, -2.75);
        
        // 加速度限制
        private_nh_.param("acc_lim_x", dwa_params.acc_lim_x, 2.5);
        private_nh_.param("acc_lim_theta", dwa_params.acc_lim_theta, 3.2);
        
        // 采样参数
        private_nh_.param("vx_samples", dwa_params.vx_samples, 20);
        private_nh_.param("vth_samples", dwa_params.vth_samples, 40);
        private_nh_.param("sim_time", dwa_params.sim_time, 1.5);
        private_nh_.param("dt", dwa_params.dt, 0.1);
        
        // 目标容差
        private_nh_.param("xy_goal_tolerance", dwa_params.xy_goal_tolerance, 0.15);
        private_nh_.param("yaw_goal_tolerance", dwa_params.yaw_goal_tolerance, 0.17);
        
        // 代价权重
        private_nh_.param("path_distance_bias", dwa_params.path_distance_bias, 32.0);
        private_nh_.param("goal_distance_bias", dwa_params.goal_distance_bias, 20.0);
        
        // 机器人参数
        private_nh_.param("robot_radius", dwa_params.robot_radius, 0.25);
        private_nh_.param("controller_frequency", dwa_params.controller_frequency, 10.0);
        private_nh_.param("lookahead_dist", dwa_params.lookahead_distance, 1.5);
        
        // 自适应参数
        private_nh_.param("enable_adaptive_vel", dwa_params.enable_adaptive_vel, true);
        private_nh_.param("min_safe_distance", dwa_params.min_safe_distance, 0.3);
        private_nh_.param("adaptive_speed_factor", dwa_params.adaptive_speed_factor, 0.7);
        
        // 全局路径方向角评价权重
        private_nh_.param("global_heading_bias", dwa_params.global_heading_bias, 0.5);
        
        dwa_planner_->setParams(dwa_params);
        dwa_planner_->initialize();
        
        // 订阅odom话题 - 使用lambda表达式避免boost::bind问题
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1,
            [this](const nav_msgs::Odometry::ConstPtr& msg) {
                this->odomCallback(msg);
            });
        
        initialized_ = true;
        ROS_INFO("ImprovedDWALocalPlanner initialized successfully");
    }
}

void ImprovedDWALocalPlanner::loadParameters() {
    ROS_INFO("ImprovedDWALocalPlanner parameters loaded");
}

bool ImprovedDWALocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("Planner not initialized");
        return false;
    }
    
    global_plan_ = plan;
    
    if (dwa_planner_) {
        dwa_planner_->setGlobalPlan(global_plan_);
    }
    
    ROS_INFO("DWA: Global plan set with %zu points", global_plan_.size());
    return true;
}

bool ImprovedDWALocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("Planner not initialized");
        return false;
    }
    
    // 获取机器人位姿
    if (!getRobotPose(robot_pose_)) {
        ROS_WARN("Could not get robot pose");
        return false;
    }
    
    // 更新代价地图
    nav_msgs::OccupancyGrid grid;
    costmapToOccupancyGrid(grid);
    nav_msgs::OccupancyGrid::ConstPtr grid_ptr(new nav_msgs::OccupancyGrid(grid));
    dwa_planner_->updateCostMap(grid_ptr);
    
    // 计算机器人速度
    cmd_vel = dwa_planner_->computeVelocityCommands(robot_pose_, robot_velocity_);
    
    return true;
}

bool ImprovedDWALocalPlanner::isGoalReached() {
    if (!initialized_ || global_plan_.empty()) {
        return false;
    }
    
    if (!getRobotPose(robot_pose_)) {
        return false;
    }
    
    if (dwa_planner_) {
        return dwa_planner_->isGoalReached(robot_pose_);
    }
    
    return false;
}

bool ImprovedDWALocalPlanner::getRobotPose(geometry_msgs::PoseStamped& pose) {
    if (!tf_) return false;
    
    pose.header.frame_id = robot_base_frame_;
    pose.header.stamp = ros::Time(0);
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.w = 1.0;
    
    try {
        tf::StampedTransform transform;
        tf_->lookupTransform(global_frame_, robot_base_frame_, ros::Time(0), transform);
        
        pose.pose.position.x = transform.getOrigin().x();
        pose.pose.position.y = transform.getOrigin().y();
        pose.pose.position.z = transform.getOrigin().z();
        
        pose.pose.orientation.x = transform.getRotation().x();
        pose.pose.orientation.y = transform.getRotation().y();
        pose.pose.orientation.z = transform.getRotation().z();
        pose.pose.orientation.w = transform.getRotation().w();
        
        return true;
    } catch (tf::TransformException& ex) {
        ROS_WARN("Failed to transform robot pose: %s", ex.what());
        return false;
    }
}

void ImprovedDWALocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_velocity_.linear.x = msg->twist.twist.linear.x;
    robot_velocity_.linear.y = msg->twist.twist.linear.y;
    robot_velocity_.angular.z = msg->twist.twist.angular.z;
}

void ImprovedDWALocalPlanner::costmapToOccupancyGrid(nav_msgs::OccupancyGrid& grid) {
    auto costmap = costmap_ros_->getCostmap();
    
    grid.header.frame_id = global_frame_;
    grid.header.stamp = ros::Time::now();
    
    grid.info.width = costmap->getSizeInCellsX();
    grid.info.height = costmap->getSizeInCellsY();
    grid.info.resolution = costmap->getResolution();
    
    double origin_x, origin_y;
    origin_x = costmap->getOriginX();
    origin_y = costmap->getOriginY();
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    grid.data.resize(grid.info.width * grid.info.height);
    
    for (unsigned int y = 0; y < grid.info.height; ++y) {
        for (unsigned int x = 0; x < grid.info.width; ++x) {
            unsigned int cost = costmap->getCost(x, y);
            int index = y * grid.info.width + x;
            
            if (cost == costmap_2d::NO_INFORMATION) {
                grid.data[index] = -1;
            } else if (cost == costmap_2d::LETHAL_OBSTACLE) {
                grid.data[index] = 100;
            } else if (cost > 0) {
                grid.data[index] = (cost > 50) ? 100 : 0;
            } else {
                grid.data[index] = 0;
            }
        }
    }
}

}  // namespace my_astar_dwa_plugins

// 插件导出宏
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_astar_dwa_plugins::ImprovedDWALocalPlanner, nav_core::BaseLocalPlanner)