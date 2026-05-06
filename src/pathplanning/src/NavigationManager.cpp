#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include "Astar.h"
#include "DWAPlanner.h"
#include "tf_quaternion_utils.h"
#include <mutex>
#include <memory>

class FixedNavigation {
private:
    ros::NodeHandle nh_;
    
    // 规划器实例
    std::shared_ptr<pathplanning::Astar> astar_planner_;
    pathplanning::DWAPlanner dwa_planner_;
    
    // ROS通信
    ros::Subscriber map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher global_path_pub_;
    ros::Publisher local_cmd_pub_;
    ros::Publisher status_pub_;
    
    // 导航状态
    enum NavigationState {
        IDLE,
        GLOBAL_PLANNING,
        LOCAL_EXECUTION,
        GOAL_REACHED,
        FAILED
    };
    
    NavigationState current_state_;
    
    // 数据存储
    nav_msgs::OccupancyGrid current_map_;
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::Twist robot_velocity_;
    std::vector<geometry_msgs::PoseStamped> global_path_;
    
    // 目标管理
    bool has_valid_goal_;
    bool has_valid_start_;
    ros::Time last_goal_time_;
    ros::Time last_planning_time_;
    
    // 配置参数
    struct Config {
        double control_frequency;
        double goal_tolerance;
        double planning_timeout;
        double min_replan_interval;
    } config_;
    
    // 性能监控
    int planning_attempts_;
    int successful_plans_;
    int consecutive_failures_;

public:
    FixedNavigation() : 
        current_state_(IDLE),
        has_valid_goal_(false),
        has_valid_start_(false),
        planning_attempts_(0),
        successful_plans_(0),
        consecutive_failures_(0)
    {
        // 初始化参数
        config_.control_frequency = 10.0;
        config_.goal_tolerance = 0.1;
        config_.planning_timeout = 30.0;
        config_.min_replan_interval = 2.0;
        
        // 初始化规划器
        pathplanning::AstarConfig astar_config;
        astar_config.inflate_radius = 3;
        astar_config.safety_margin = 0.2;
        astar_config.heuristic_weight = 1.0;
        astar_config.max_iterations = 5000;
        astar_config.min_planning_interval = 1.0;
        
        astar_planner_ = std::make_shared<pathplanning::Astar>();
        astar_planner_->initAstar(astar_config);
        
        dwa_planner_.initialize();
        
        // 初始化ROS话题
        initializeROS();
        
        last_goal_time_ = ros::Time::now();
        last_planning_time_ = ros::Time::now();
        
        ROS_INFO("=== Fixed Navigation System Initialized ===");
    }
    
    void run() {
        ros::Rate rate(config_.control_frequency);
        
        while (ros::ok()) {
            processNavigation();
            publishStatus();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void initializeROS() {
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
            "/map", 1, &FixedNavigation::mapCallback, this);
            
        start_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose", 1, &FixedNavigation::startCallback, this);
            
        target_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 1, &FixedNavigation::targetCallback, this);
            
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/odom", 1, &FixedNavigation::odomCallback, this);
        
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_plan", 1, true);
        local_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        status_pub_ = nh_.advertise<std_msgs::String>("/navigation_status", 1, true);
    }
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
        current_map_ = *grid;
        
        if (astar_planner_) {
            astar_planner_->updateMap(grid);
        }
        
        ROS_INFO("Map updated: %dx%d", grid->info.width, grid->info.height);
    }
    
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        if (!astar_planner_) {
            ROS_WARN_THROTTLE(5.0, "A* planner not initialized");
            return;
        }
        
        astar_planner_->setStartPoint(msg);
        has_valid_start_ = true;
        
        ROS_INFO("Start position set");
        
        // 如果已经有目标点，开始规划
        if (has_valid_goal_ && current_state_ == IDLE) {
            current_state_ = GLOBAL_PLANNING;
        }
    }
    
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ROS_INFO("=== NEW GOAL RECEIVED ===");
        
        // 创建安全的目标点
        geometry_msgs::PoseStamped safe_goal = *msg;
        safe_goal.pose.orientation = 
            tf_quaternion_utils::normalizeQuaternionStrict(safe_goal.pose.orientation);
        
        // 设置A*目标点
        if (astar_planner_) {
            astar_planner_->setTargetPoint(
                boost::make_shared<geometry_msgs::PoseStamped>(safe_goal));
        }
        
        has_valid_goal_ = true;
        last_goal_time_ = ros::Time::now();
        
        // 重置导航状态
        resetNavigation();
        current_state_ = GLOBAL_PLANNING;
        
        ROS_INFO("Target position set, starting global planning...");
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_pose_.header = msg->header;
        robot_pose_.pose = msg->pose.pose;
        robot_velocity_ = msg->twist.twist;
        
        // 确保机器人姿态的四元数是安全的
        tf_quaternion_utils::validateAndFixQuaternion(robot_pose_.pose.orientation);
    }
    
    void processNavigation() {
        switch (current_state_) {
            case IDLE:
                // 等待目标
                break;
                
            case GLOBAL_PLANNING:
                executeGlobalPlanning();
                break;
                
            case LOCAL_EXECUTION:
                executeLocalControl();
                break;
                
            case GOAL_REACHED:
                handleGoalReached();
                break;
                
            case FAILED:
                handleFailedState();
                break;
        }
    }
    
    void executeGlobalPlanning() {
        planning_attempts_++;
        
        // 检查规划条件
        if (!has_valid_start_ || !has_valid_goal_) {
            ROS_WARN_THROTTLE(2.0, "Not ready for global planning");
            current_state_ = FAILED;
            return;
        }
        
        // 检查重新规划间隔
        ros::Time now = ros::Time::now();
        if ((now - last_planning_time_).toSec() < config_.min_replan_interval) {
            return;
        }
        last_planning_time_ = now;
        
        ROS_INFO("Executing global planning (attempt %d)...", planning_attempts_);
        
        std::vector<cv::Point> path_points;
        bool success = false;
        
        if (astar_planner_) {
            success = astar_planner_->pathPlanning(path_points);
        }
        
        if (success && !path_points.empty()) {
            successful_plans_++;
            consecutive_failures_ = 0;
            
            // 转换路径格式
            global_path_ = convertToPoseStamped(path_points);
            
            // 设置DWA的全局路径
            dwa_planner_.setGlobalPlan(global_path_);
            
            // 发布全局路径
            publishGlobalPath(global_path_);
            
            current_state_ = LOCAL_EXECUTION;
            ROS_INFO("Global planning successful! Path with %zu points", global_path_.size());
        } else {
            consecutive_failures_++;
            ROS_WARN_THROTTLE(2.0, "Global planning failed (consecutive failures: %d)", 
                              consecutive_failures_);
            current_state_ = FAILED;
        }
    }
    
    void executeLocalControl() {
        if (!dwa_planner_.isInitialized()) {
            ROS_WARN_THROTTLE(5.0, "DWA planner not initialized");
            current_state_ = FAILED;
            return;
        }
        
        // 检查是否到达目标
        if (dwa_planner_.isGoalReached(robot_pose_)) {
            ROS_INFO("Goal reached!");
            current_state_ = GOAL_REACHED;
            return;
        }
        
        // 计算速度命令
        geometry_msgs::Twist cmd_vel = dwa_planner_.computeVelocityCommands(
            robot_pose_, robot_velocity_);
        
        // 发布速度命令
        local_cmd_pub_.publish(cmd_vel);
    }
    
    void handleFailedState() {
        publishStopCommand();
        
        ROS_WARN_THROTTLE(2.0, "Navigation failed. Resetting in 3 seconds...");
        
        // 根据连续失败次数调整等待时间
        double wait_time = std::min(3.0 + consecutive_failures_, 10.0);
        ros::Duration(wait_time).sleep();
        
        resetNavigation();
    }
    
    void handleGoalReached() {
        publishStopCommand();
        
        double success_rate = (planning_attempts_ > 0) ? 
            (successful_plans_ * 100.0 / planning_attempts_) : 0.0;
            
        ROS_INFO("=== NAVIGATION SUCCESSFULLY COMPLETED ===");
        ROS_INFO("Planning success rate: %.1f%% (%d/%d)", 
                 success_rate, successful_plans_, planning_attempts_);
        
        // 5秒后自动重置
        ros::Duration(5.0).sleep();
        resetNavigation();
    }
    
    void resetNavigation() {
        global_path_.clear();
        current_state_ = IDLE;
        consecutive_failures_ = 0;
        
        ROS_INFO("Navigation system reset to IDLE state");
    }
    
    std::vector<geometry_msgs::PoseStamped> convertToPoseStamped(const std::vector<cv::Point>& path_points) {
        std::vector<geometry_msgs::PoseStamped> poses;
        
        if (path_points.empty()) {
            return poses;
        }
        
        for (const auto& point : path_points) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            
            // 坐标转换
            double world_x = point.x * current_map_.info.resolution + current_map_.info.origin.position.x;
            double world_y = point.y * current_map_.info.resolution + current_map_.info.origin.position.y;
            
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation = tf_quaternion_utils::createSafeQuaternionFromYaw(0.0);
            
            poses.push_back(pose);
        }
        
        return poses;
    }
    
    void publishGlobalPath(const std::vector<geometry_msgs::PoseStamped>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.poses = path;
        global_path_pub_.publish(path_msg);
    }
    
    void publishStopCommand() {
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        local_cmd_pub_.publish(stop_cmd);
    }
    
    void publishStatus() {
        static ros::Time last_publish = ros::Time::now();
        if ((ros::Time::now() - last_publish).toSec() < 1.0) {
            return; // 限制发布频率
        }
        last_publish = ros::Time::now();
        
        std_msgs::String status_msg;
        switch (current_state_) {
            case IDLE: status_msg.data = "IDLE"; break;
            case GLOBAL_PLANNING: status_msg.data = "GLOBAL_PLANNING"; break;
            case LOCAL_EXECUTION: status_msg.data = "LOCAL_EXECUTION"; break;
            case GOAL_REACHED: status_msg.data = "GOAL_REACHED"; break;
            case FAILED: status_msg.data = "FAILED"; break;
        }
        status_pub_.publish(status_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fixed_navigation");
    
    // 设置ROS日志级别
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    try {
        FixedNavigation navigation;
        navigation.run();
    } catch (const std::exception& e) {
        ROS_FATAL("Navigation node failed: %s", e.what());
        return 1;
    }
    
    return 0;
}