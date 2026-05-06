#ifndef ASTAR_DWA_NAVIGATION_H
#define ASTAR_DWA_NAVIGATION_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "improved_astar.h"
#include "dwa_planner.h"

#include <memory>
#include <mutex>

class AStarDWANavigation {
public:
    AStarDWANavigation();
    void run();
    
private:
    enum NavigationState {
        WAITING_FOR_MAP = 0,
        WAITING_FOR_START = 1,
        WAITING_FOR_GOAL = 2,
        IDLE = 3,
        PLANNING = 4,
        EXECUTING = 5,
        GOAL_REACHED = 6,
        FAILED = 7
    };
    
    // ROS节点
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 规划器
    std::shared_ptr<pathplanning::ImprovedAstar> astar_planner_;
    std::shared_ptr<pathplanning::DWAPlanner> dwa_planner_;
    
    // 订阅器
    ros::Subscriber map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber odom_sub_;
    
    // 发布器
    ros::Publisher global_path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher status_pub_;
    ros::Publisher marker_pub_;
    
    // TF监听器
    tf::TransformListener tf_listener_;
    
    // 状态
    NavigationState current_state_;
    
    // 数据
    nav_msgs::OccupancyGrid current_map_;
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::Twist robot_velocity_;
    std::vector<geometry_msgs::PoseStamped> global_path_;
    geometry_msgs::PoseStamped current_goal_;
    geometry_msgs::PoseWithCovarianceStamped current_start_;
    
    // 配置参数
    struct Config {
        double control_frequency;
        double goal_tolerance;
        double min_replan_interval;
        bool enable_replanning;
        bool use_tf_for_pose;
        double max_planning_time;
    } config_;
    
    // 标志
    bool start_point_set_;
    bool target_point_set_;
    bool map_ready_;
    bool robot_pose_valid_;
    
    // 时间
    ros::Time last_planning_time_;
    
    // 计数器
    int planning_attempts_;
    int navigation_counter_;
    
    // 互斥锁
    std::mutex data_mutex_;
    
    // 初始化
    void initializeSafe();
    void initializeParameters();
    void initializePlanners();
    void initializeROS();
    
    // ROS回调
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid);
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // 导航处理
    void processNavigation();
    void handleWaitingForMap();
    void handleWaitingForStart();
    void handleWaitingForGoal();
    void handleIdleState();
    void executePlanning();
    void executeControl();
    void handleGoalReached();
    void handleFailedState();
    
    // 工具函数
    void clearPreviousPath();
    bool getCurrentRobotPose(geometry_msgs::PoseStamped& pose);
    bool getRobotPoseFromTF(geometry_msgs::PoseStamped& pose);
    bool isGoalReached(const geometry_msgs::PoseStamped& current_pose);
    void publishGlobalPath(const std::vector<geometry_msgs::PoseStamped>& path);
    void publishMarkers();
    void publishStopCommand();
    void publishStatus();
};

#endif // ASTAR_DWA_NAVIGATION_H
