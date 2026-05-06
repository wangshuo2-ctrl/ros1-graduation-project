#include "pathplanning/Astar.h"
#include "pathplanning/ctDWAPlanner.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <memory>
#include <iostream>
#include <chrono>

using namespace pathplanning;

namespace quaternion_utils {
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
        tf::Quaternion tf_quat;
        tf::quaternionMsgToTF(quat, tf_quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    geometry_msgs::Quaternion createQuaternionFromYaw(double yaw) {
        tf::Quaternion tf_quat = tf::createQuaternionFromYaw(yaw);
        geometry_msgs::Quaternion quat;
        tf::quaternionTFToMsg(tf_quat, quat);
        return quat;
    }
}

class FixedAStarDWANavigation {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    std::shared_ptr<Astar> astar_planner_;
    std::shared_ptr<ctDWAPlanner> dwa_planner_;  // 使用传统DWA规划器
    
    ros::Subscriber map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber odom_sub_;
    
    ros::Publisher global_path_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher status_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher astar_path_pub_;
    
    enum NavigationState {
        WAITING_FOR_MAP = 0,
        WAITING_FOR_START = 1,
        WAITING_FOR_GOAL = 2,
        BOTH_POINTS_SET = 3,
        PLANNING = 4,
        EXECUTING = 5,
        GOAL_REACHED = 6,
        FAILED = 7
    };
    
    NavigationState current_state_;
    
    nav_msgs::OccupancyGrid current_map_;
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::Twist robot_velocity_;
    std::vector<geometry_msgs::PoseStamped> global_path_;
    std::vector<geometry_msgs::PoseStamped> local_path_;
    geometry_msgs::PoseStamped current_goal_;
    geometry_msgs::PoseWithCovarianceStamped current_start_;
    
    struct Config {
        double control_frequency = 10.0;
        double goal_tolerance = 0.15;
        double min_replan_interval = 0.1;
        bool enable_replanning = true;
        bool use_tf_for_pose = false;
        double max_planning_time = 5.0;
        std::string global_path_topic = "/ct_global_plan";
        std::string local_path_topic = "/ct_local_path";
        std::string astar_path_topic = "/ct_astar_path";
    } config_;
    
    bool start_point_set_;
    bool target_point_set_;
    bool map_ready_;
    bool robot_pose_valid_;
    bool use_simulated_odom_;
    bool abort_current_planning_;
    
    ros::Time last_planning_time_;
    ros::Time last_control_time_;
    int planning_attempts_;
    int navigation_counter_;
    int navigation_id_;
    
    std::mutex data_mutex_;
    
public:
    FixedAStarDWANavigation() :
        private_nh_("~"),
        current_state_(WAITING_FOR_MAP),
        start_point_set_(false),
        target_point_set_(false),
        map_ready_(false),
        robot_pose_valid_(false),
        use_simulated_odom_(true),
        abort_current_planning_(false),
        planning_attempts_(0),
        navigation_counter_(0),
        navigation_id_(0) {
        
        ROS_INFO("=== Fixed A* + Traditional DWA Navigation System ===");
        ROS_INFO("Constructor called, initializing system...");
        initializeSafe();
        ROS_INFO("Navigation system initialized successfully");
    }
    
    void run() {
        ROS_INFO("Starting navigation system main loop at %.1f Hz", config_.control_frequency);
        ros::Rate rate(config_.control_frequency);
        
        while (ros::ok()) {
            processNavigation();
            publishStatus();
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("Navigation system shutdown");
    }
    
private:
    void initializeSafe() {
        try {
            initializeParameters();
            initializePlanners();
            initializeROS();
            
            last_planning_time_ = ros::Time::now();
            last_control_time_ = ros::Time::now();
            ROS_INFO("All components initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to initialize navigation system: %s", e.what());
            throw;
        }
    }
    
    void initializeParameters() {
        config_.control_frequency = 10.0;
        config_.goal_tolerance = 0.15;
        config_.min_replan_interval = 0.1;
        config_.enable_replanning = true;
        config_.use_tf_for_pose = false;
        config_.max_planning_time = 5.0;
        config_.global_path_topic = "/ct_global_plan";
        config_.local_path_topic = "/ct_local_path";
        config_.astar_path_topic = "/ct_astar_path";
        
        private_nh_.param("control_frequency", config_.control_frequency, config_.control_frequency);
        private_nh_.param("goal_tolerance", config_.goal_tolerance, config_.goal_tolerance);
        private_nh_.param("min_replan_interval", config_.min_replan_interval, config_.min_replan_interval);
        private_nh_.param("enable_replanning", config_.enable_replanning, config_.enable_replanning);
        private_nh_.param("use_tf_for_pose", config_.use_tf_for_pose, config_.use_tf_for_pose);
        private_nh_.param("max_planning_time", config_.max_planning_time, config_.max_planning_time);
        private_nh_.param("global_path_topic", config_.global_path_topic, config_.global_path_topic);
        private_nh_.param("local_path_topic", config_.local_path_topic, config_.local_path_topic);
        private_nh_.param("astar_path_topic", config_.astar_path_topic, config_.astar_path_topic);
        private_nh_.param("use_simulated_odom", use_simulated_odom_, true);
        
        ROS_INFO("Navigation parameters:");
        ROS_INFO("  Control frequency: %.1f Hz", config_.control_frequency);
        ROS_INFO("  Goal tolerance: %.3f m", config_.goal_tolerance);
        ROS_INFO("  Min replan interval: %.3f s", config_.min_replan_interval);
        ROS_INFO("  Max planning time: %.1f s", config_.max_planning_time);
        ROS_INFO("  Use TF for robot pose: %s", config_.use_tf_for_pose ? "YES" : "NO");
        ROS_INFO("  Use simulated odometry: %s", use_simulated_odom_ ? "YES" : "NO");
    }
    
    void initializePlanners() {
        ROS_INFO("Initializing A* planner...");
        try {
            astar_planner_ = std::make_shared<Astar>();
            
            AstarConfig astar_config;
            astar_config.heuristic_weight = 1.0;
            astar_config.inflate_radius = 3;
            astar_config.safety_margin = 0.3;
            astar_config.obstacle_clearance = 0.3;
            astar_config.turn_penalty_weight = 0.1;
            astar_config.max_iterations = 50000;
            astar_config.max_search_radius = 30;
            astar_config.enable_path_smoothing = true;
            
            astar_config.enable_bspline_smoothing = true;
            astar_config.bspline_degree = 3;
            astar_config.bspline_samples = 100;
            astar_config.bspline_smoothness = 0.8;
            
            astar_config.enable_path_resample = true;
            astar_config.resample_points = 100;
            
            astar_config.enable_safety_check = true;
            astar_config.safety_check_distance = 0.3;
            astar_config.safety_check_resolution = 5;
            
            astar_config.enable_constrained_smoothing = true;
            astar_config.constrained_smoothing_iterations = 3;
            astar_config.constrained_smoothing_weight = 0.2;
            
            astar_config.euclidean = true;
            astar_config.allow_outside_map = true;
            
            astar_planner_->initAstar(astar_config);
            ROS_INFO("A* planner initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to initialize A* planner. %s", e.what());
            throw;
        }
        
        ROS_INFO("Initializing Traditional DWA planner...");
        try {
            dwa_planner_ = std::make_shared<ctDWAPlanner>();
            
            ctDWAParams dwa_params;
            private_nh_.param("max_vel_x", dwa_params.max_vel_x, 0.22);
            private_nh_.param("min_vel_x", dwa_params.min_vel_x, 0.0);
            private_nh_.param("max_vel_x_backwards", dwa_params.max_vel_x_backwards, -0.1);
            private_nh_.param("max_vel_theta", dwa_params.max_vel_theta, 2.75);
            private_nh_.param("min_vel_theta", dwa_params.min_vel_theta, -2.75);
            private_nh_.param("acc_lim_x", dwa_params.acc_lim_x, 2.5);
            private_nh_.param("acc_lim_theta", dwa_params.acc_lim_theta, 3.2);
            private_nh_.param("vx_samples", dwa_params.vx_samples, 20);
            private_nh_.param("vth_samples", dwa_params.vth_samples, 40);
            private_nh_.param("sim_time", dwa_params.sim_time, 1.5);
            private_nh_.param("dt", dwa_params.dt, 0.1);
            private_nh_.param("xy_goal_tolerance", dwa_params.xy_goal_tolerance, 0.15);
            private_nh_.param("yaw_goal_tolerance", dwa_params.yaw_goal_tolerance, 0.17);
            private_nh_.param("goal_distance_bias", dwa_params.goal_distance_bias, 20.0);
            private_nh_.param("path_distance_bias", dwa_params.path_distance_bias, 32.0);
            private_nh_.param("robot_radius", dwa_params.robot_radius, 0.25);
            private_nh_.param("obstacle_cost_bias", dwa_params.obstacle_cost_bias, 1.0);
            private_nh_.param("occdist_scale", dwa_params.occdist_scale, 0.02);
            private_nh_.param("obstacle_cutoff_dist", dwa_params.obstacle_cutoff_dist, 2.5);
            private_nh_.param("controller_frequency", dwa_params.controller_frequency, 10.0);
            private_nh_.param("sim_granularity", dwa_params.sim_granularity, 0.025);
            private_nh_.param("obstacle_threshold", dwa_params.obstacle_threshold, 60);
            
            dwa_planner_->setParams(dwa_params);
            dwa_planner_->initialize();
            ROS_INFO("Traditional DWA planner initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to initialize Traditional DWA planner. %s", e.what());
            throw;
        }
    }
    
    void initializeROS() {
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &FixedAStarDWANavigation::mapCallback, this);
        start_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &FixedAStarDWANavigation::startCallback, this);
        target_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &FixedAStarDWANavigation::targetCallback, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &FixedAStarDWANavigation::odomCallback, this);
        
        global_path_pub_ = nh_.advertise<nav_msgs::Path>(config_.global_path_topic, 1, true);
        local_path_pub_ = nh_.advertise<nav_msgs::Path>(config_.local_path_topic, 1, true);
        astar_path_pub_ = nh_.advertise<nav_msgs::Path>(config_.astar_path_topic, 1, true);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/ct_cmd_vel", 1);
        status_pub_ = nh_.advertise<std_msgs::String>("/ct_navigation_status", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ct_navigation_markers", 1, true);
    }
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (!grid) return;
        if (grid->info.width == 0 || grid->info.height == 0) return;
        
        current_map_ = *grid;
        
        if (astar_planner_) astar_planner_->updateMap(grid);
        if (dwa_planner_) dwa_planner_->updateCostMap(grid);
        
        if (!map_ready_) {
            map_ready_ = true;
            if (current_state_ == WAITING_FOR_MAP) {
                current_state_ = WAITING_FOR_START;
            }
        }
    }
    
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (!msg) return;
        
        ROS_INFO("=== START POINT SET ===");
        ROS_INFO("Start point received: frame_id=%s", msg->header.frame_id.c_str());
        ROS_INFO("Position: (%.3f, %.3f, %.3f)",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);
        
        current_start_ = *msg;
        start_point_set_ = true;
        
        if (astar_planner_) astar_planner_->setStartPoint(msg);
        
        if (use_simulated_odom_) {
            robot_pose_.header = msg->header;
            robot_pose_.pose = msg->pose.pose;
            robot_pose_valid_ = true;
            robot_velocity_.linear.x = 0.0;
            robot_velocity_.angular.z = 0.0;
        }
        
        ROS_INFO("Start point set. Start ready: %s, Goal ready: %s", 
                 start_point_set_ ? "YES" : "NO", 
                 target_point_set_ ? "YES" : "NO");
        
        // 检查当前状态并处理
        switch (current_state_) {
            case WAITING_FOR_START:
                if (target_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for goal point...");
                    current_state_ = WAITING_FOR_GOAL;
                }
                break;
                
            case WAITING_FOR_GOAL:
                if (target_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for goal point...");
                    // 保持在WAITING_FOR_GOAL状态
                }
                break;
                
            case GOAL_REACHED:
                if (target_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for goal point...");
                    current_state_ = WAITING_FOR_GOAL;
                }
                break;
                
            case FAILED:
                if (target_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for goal point...");
                    current_state_ = WAITING_FOR_GOAL;
                }
                break;
                
            case EXECUTING:
                // 在执行过程中设置新起点，不触发重新规划
                ROS_INFO("New start point set during execution. Waiting for goal point to be set.");
                // 保持在EXECUTING状态
                break;
                
            case PLANNING:
                // 在规划过程中设置新起点，标记需要中止当前规划
                abort_current_planning_ = true;
                ROS_INFO("New start point set during planning. Will abort current planning and start new one.");
                break;
                
            case BOTH_POINTS_SET:
                // 在BOTH_POINTS_SET状态下设置新起点，检查目标点是否设置
                if (target_point_set_) {
                    ROS_INFO("Both points are set. Navigation will begin when conditions are met.");
                    // 保持在BOTH_POINTS_SET状态
                } else {
                    ROS_INFO("Start point set, but goal point not set. Waiting for goal point...");
                    current_state_ = WAITING_FOR_GOAL;
                }
                break;
                
            default:
                // 其他状态保持不变
                break;
        }
        
        // 清除之前的路径
        global_path_.clear();
        local_path_.clear();
        publishEmptyPaths();
    }
    
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (!msg) return;
        
        ROS_INFO("=== GOAL POINT SET ===");
        ROS_INFO("Goal point received: frame_id=%s", msg->header.frame_id.c_str());
        ROS_INFO("Position: (%.3f, %.3f, %.3f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
        current_goal_ = *msg;
        target_point_set_ = true;
        
        if (astar_planner_) astar_planner_->setTargetPoint(msg);
        
        ROS_INFO("Goal point set. Start ready: %s, Goal ready: %s", 
                 start_point_set_ ? "YES" : "NO", 
                 target_point_set_ ? "YES" : "NO");
        
        // 检查当前状态并处理
        switch (current_state_) {
            case WAITING_FOR_GOAL:
                if (start_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for start point...");
                    current_state_ = WAITING_FOR_START;
                }
                break;
                
            case WAITING_FOR_START:
                if (start_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for start point...");
                    // 保持在WAITING_FOR_START状态
                }
                break;
                
            case GOAL_REACHED:
                if (start_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for start point...");
                    current_state_ = WAITING_FOR_START;
                }
                break;
                
            case FAILED:
                if (start_point_set_) {
                    ROS_INFO("Both points are set! Navigation will begin when conditions are met.");
                    current_state_ = BOTH_POINTS_SET;
                } else {
                    ROS_INFO("Waiting for start point...");
                    current_state_ = WAITING_FOR_START;
                }
                break;
                
            case EXECUTING:
                // 在执行过程中设置新终点，不触发重新规划
                ROS_INFO("New goal point set during execution. Waiting for start point to be set.");
                // 保持在EXECUTING状态
                break;
                
            case PLANNING:
                // 在规划过程中设置新终点，标记需要中止当前规划
                abort_current_planning_ = true;
                ROS_INFO("New goal point set during planning. Will abort current planning and start new one.");
                break;
                
            case BOTH_POINTS_SET:
                // 在BOTH_POINTS_SET状态下设置新终点，检查起始点是否设置
                if (start_point_set_) {
                    ROS_INFO("Both points are set. Navigation will begin when conditions are met.");
                    // 保持在BOTH_POINTS_SET状态
                } else {
                    ROS_INFO("Goal point set, but start point not set. Waiting for start point...");
                    current_state_ = WAITING_FOR_START;
                }
                break;
                
            default:
                // 其他状态保持不变
                break;
        }
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!use_simulated_odom_) {
            robot_pose_.header = msg->header;
            robot_pose_.pose = msg->pose.pose;
            robot_velocity_ = msg->twist.twist;
            robot_pose_valid_ = true;
        }
    }
    
    void processNavigation() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        switch (current_state_) {
            case WAITING_FOR_MAP: handleWaitingForMap(); break;
            case WAITING_FOR_START: handleWaitingForStart(); break;
            case WAITING_FOR_GOAL: handleWaitingForGoal(); break;
            case BOTH_POINTS_SET: handleBothPointsSet(); break;
            case PLANNING: executePlanning(); break;
            case EXECUTING: executeControl(); break;
            case GOAL_REACHED: handleGoalReached(); break;
            case FAILED: handleFailedState(); break;
        }
    }
    
    void handleWaitingForMap() {
        static ros::Time last_log = ros::Time::now();
        if ((ros::Time::now() - last_log).toSec() > 2.0) {
            ROS_INFO("Waiting for map...");
            last_log = ros::Time::now();
        }
    }
    
    void handleWaitingForStart() {
        static ros::Time last_log = ros::Time::now();
        if ((ros::Time::now() - last_log).toSec() > 2.0) {
            ROS_INFO("Waiting for start point... Click '2D Pose Estimate' in RViz");
            last_log = ros::Time::now();
        }
    }
    
    void handleWaitingForGoal() {
        static ros::Time last_log = ros::Time::now();
        if ((ros::Time::now() - last_log).toSec() > 2.0) {
            ROS_INFO("Waiting for goal point... Click '2D Nav Goal' in RViz");
            last_log = ros::Time::now();
        }
    }
    
    void handleBothPointsSet() {
        ROS_DEBUG("In BOTH_POINTS_SET state, checking planning conditions...");
        
        if (!map_ready_) {
            ROS_WARN("Cannot plan: Map not ready");
            current_state_ = WAITING_FOR_MAP;
            return;
        }
        
        if (!start_point_set_) {
            ROS_WARN("Cannot plan: Start point not set");
            current_state_ = WAITING_FOR_START;
            return;
        }
        
        if (!target_point_set_) {
            ROS_WARN("Cannot plan: Goal point not set");
            current_state_ = WAITING_FOR_GOAL;
            return;
        }
        
        ros::Time now = ros::Time::now();
        double time_since_last_plan = (now - last_planning_time_).toSec();
        
        // 确保起点和终点都设置了，并且满足最小规划间隔
        if (time_since_last_plan >= config_.min_replan_interval) {
            ROS_INFO("=== Starting path planning (Navigation #%d) ===", navigation_id_ + 1);
            ROS_INFO("Start: (%.3f, %.3f)",
                    current_start_.pose.pose.position.x, current_start_.pose.pose.position.y);
            ROS_INFO("Goal: (%.3f, %.3f)",
                    current_goal_.pose.position.x, current_goal_.pose.position.y);
            
            current_state_ = PLANNING;
            last_planning_time_ = now;
            abort_current_planning_ = false;  // 重置中止标志
        } else {
            ROS_DEBUG("Waiting %.3f seconds for next replanning opportunity", 
                     config_.min_replan_interval - time_since_last_plan);
        }
    }
    
    void executePlanning() {
        ROS_INFO("=== EXECUTING PLANNING (Navigation #%d, attempt %d) ===", navigation_id_ + 1, planning_attempts_ + 1);
        planning_attempts_++;
        
        if (!astar_planner_) {
            ROS_WARN("A* planner is not initialized");
            current_state_ = FAILED;
            return;
        }
        
        if (!start_point_set_) {
            ROS_WARN("Start point not set");
            current_state_ = WAITING_FOR_START;
            return;
        }
        
        if (!target_point_set_) {
            ROS_WARN("Goal point not set");
            current_state_ = WAITING_FOR_GOAL;
            return;
        }
        
        if (!map_ready_) {
            ROS_WARN("Map not ready");
            current_state_ = FAILED;
            return;
        }
        
        std::vector<cv::Point> path_points;
        bool success = false;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        if (astar_planner_) {
            success = astar_planner_->pathPlanning(path_points);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        double planning_time = std::chrono::duration<double>(end_time - start_time).count();
        
        ROS_INFO("Planning took %.3f seconds", planning_time);
        
        // 检查是否在规划过程中设置了新点
        if (abort_current_planning_) {
            ROS_INFO("Planning was aborted due to new points being set. Moving to BOTH_POINTS_SET state.");
            current_state_ = BOTH_POINTS_SET;
            abort_current_planning_ = false;
            return;
        }
        
        if (success && !path_points.empty()) {
            global_path_ = convertToPoseStamped(path_points);
            
            if (dwa_planner_) {
                dwa_planner_->setGlobalPlan(global_path_);
                ROS_INFO("DWA: Global plan set with %zu points", global_path_.size());
            }
            
            publishGlobalPath(global_path_);
            publishAstarPath(global_path_);
            publishMarkers();
            
            current_state_ = EXECUTING;
            navigation_counter_++;
            navigation_id_++;  // 增加导航ID
            planning_attempts_ = 0;  // 重置尝试次数
            abort_current_planning_ = false;  // 重置中止标志
            ROS_INFO("=== Planning successful! Navigation #%d started ===", navigation_id_);
            
        } else {
            ROS_WARN("=== Planning failed ===");
            if (planning_attempts_ >= 3) {
                ROS_ERROR("Maximum planning attempts reached (%d), moving to FAILED state", planning_attempts_);
                current_state_ = FAILED;
            } else {
                ROS_INFO("Will retry planning in next cycle");
                // 回到BOTH_POINTS_SET状态，等待下次规划
                current_state_ = BOTH_POINTS_SET;
            }
        }
    }
    
    void executeControl() {
        if (!robot_pose_valid_) {
            publishStopCommand();
            ROS_WARN("Robot pose not valid, stopping");
            return;
        }
        
        // 检查目标是否已到达
        if (dwa_planner_ && dwa_planner_->isGoalReached(robot_pose_)) {
            current_state_ = GOAL_REACHED;
            publishStopCommand();
            ROS_INFO("=== GOAL REACHED! ===");
            ROS_INFO("Navigation #%d completed successfully", navigation_id_);
            return;
        }
        
        // 检查是否需要重新规划
        if (config_.enable_replanning) {
            ros::Time now = ros::Time::now();
            double time_since_last_plan = (now - last_planning_time_).toSec();
            
            if (time_since_last_plan >= config_.min_replan_interval) {
                // 检查起点和终点是否都仍然设置
                if (start_point_set_ && target_point_set_) {
                    ROS_INFO("Triggering replanning after %.1f seconds", time_since_last_plan);
                    current_state_ = BOTH_POINTS_SET;
                    return;
                }
            }
        }
        
        geometry_msgs::Twist cmd_vel;
        if (dwa_planner_) {
            cmd_vel = dwa_planner_->computeVelocityCommands(robot_pose_, robot_velocity_);
        } else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        
        updateLocalPath();
        
        if (std::isfinite(cmd_vel.linear.x) && std::isfinite(cmd_vel.angular.z)) {
            cmd_vel_pub_.publish(cmd_vel);
            
            if (use_simulated_odom_) {
                updateSimulatedOdometry(cmd_vel);
            }
        } else {
            publishStopCommand();
        }
        
        last_control_time_ = ros::Time::now();
    }
    
    void updateSimulatedOdometry(const geometry_msgs::Twist& cmd_vel) {
        static ros::Time last_update = ros::Time::now();
        ros::Time now = ros::Time::now();
        double dt = (now - last_update).toSec();
        
        if (dt > 0.0) {
            double current_yaw = quaternion_utils::getYawFromQuaternion(robot_pose_.pose.orientation);
            
            robot_pose_.pose.position.x += cmd_vel.linear.x * cos(current_yaw) * dt;
            robot_pose_.pose.position.y += cmd_vel.linear.x * sin(current_yaw) * dt;
            
            double new_yaw = current_yaw + cmd_vel.angular.z * dt;
            robot_pose_.pose.orientation = quaternion_utils::createQuaternionFromYaw(new_yaw);
            
            robot_velocity_ = cmd_vel;
            robot_pose_.header.stamp = now;
            last_update = now;
        }
    }
    
    void updateLocalPath() {
        if (!robot_pose_valid_ || global_path_.empty()) return;
        
        std::vector<geometry_msgs::PoseStamped> new_local_path;
        new_local_path.push_back(robot_pose_);
        
        int num_points = std::min(10, (int)global_path_.size());
        for (int i = 0; i < num_points; ++i) {
            new_local_path.push_back(global_path_[i]);
        }
        
        publishLocalPath(new_local_path);
    }
    
    void handleGoalReached() {
        static ros::Time last_log = ros::Time::now();
        if ((ros::Time::now() - last_log).toSec() > 1.0) {
            ROS_INFO("=== Ready for new navigation ===");
            ROS_INFO("Set new start and goal points to continue");
            last_log = ros::Time::now();
        }
        
        publishStopCommand();
    }
    
    void handleFailedState() {
        publishStopCommand();
        
        static ros::Time last_log = ros::Time::now();
        if ((ros::Time::now() - last_log).toSec() > 2.0) {
            ROS_WARN("Navigation failed! Set new start and goal points to retry");
            last_log = ros::Time::now();
        }
    }
    
    void publishEmptyPaths() {
        nav_msgs::Path empty_path;
        empty_path.header.stamp = ros::Time::now();
        empty_path.header.frame_id = "map";
        
        global_path_pub_.publish(empty_path);
        local_path_pub_.publish(empty_path);
        astar_path_pub_.publish(empty_path);
    }
    
    std::vector<geometry_msgs::PoseStamped> convertToPoseStamped(const std::vector<cv::Point>& path_points) {
        std::vector<geometry_msgs::PoseStamped> poses;
        
        if (path_points.empty() || !map_ready_) return poses;
        
        for (const auto& point : path_points) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            
            double world_x = point.x * current_map_.info.resolution + current_map_.info.origin.position.x;
            double world_y = point.y * current_map_.info.resolution + current_map_.info.origin.position.y;
            
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation = quaternion_utils::createQuaternionFromYaw(0.0);
            
            poses.push_back(pose);
        }
        
        return poses;
    }
    
    void publishGlobalPath(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (path.empty()) {
            ROS_WARN("Cannot publish empty global path");
            return;
        }
        
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.poses = path;
        
        global_path_pub_.publish(path_msg);
    }
    
    void publishLocalPath(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (path.empty()) return;
        
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.poses = path;
        
        local_path_pub_.publish(path_msg);
    }
    
    void publishAstarPath(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (path.empty()) {
            ROS_WARN("Cannot publish empty A* path");
            return;
        }
        
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.poses = path;
        
        astar_path_pub_.publish(path_msg);
    }
    
    void publishMarkers() {
        visualization_msgs::MarkerArray marker_array;
        
        if (start_point_set_) {
            visualization_msgs::Marker start_marker;
            start_marker.header.frame_id = "map";
            start_marker.header.stamp = ros::Time::now();
            start_marker.ns = "ct_navigation";
            start_marker.id = 0;
            start_marker.type = visualization_msgs::Marker::SPHERE;
            start_marker.action = visualization_msgs::Marker::ADD;
            start_marker.pose.position.x = current_start_.pose.pose.position.x;
            start_marker.pose.position.y = current_start_.pose.pose.position.y;
            start_marker.pose.position.z = 0.1;
            start_marker.scale.x = 0.3;
            start_marker.scale.y = 0.3;
            start_marker.scale.z = 0.3;
            start_marker.color.r = 0.0;
            start_marker.color.g = 1.0;
            start_marker.color.b = 0.0;
            start_marker.color.a = 1.0;
            marker_array.markers.push_back(start_marker);
        }
        
        if (target_point_set_) {
            visualization_msgs::Marker goal_marker;
            goal_marker.header.frame_id = "map";
            goal_marker.header.stamp = ros::Time::now();
            goal_marker.ns = "ct_navigation";
            goal_marker.id = 1;
            goal_marker.type = visualization_msgs::Marker::SPHERE;
            goal_marker.action = visualization_msgs::Marker::ADD;
            goal_marker.pose.position.x = current_goal_.pose.position.x;
            goal_marker.pose.position.y = current_goal_.pose.position.y;
            goal_marker.pose.position.z = 0.1;
            goal_marker.scale.x = 0.3;
            goal_marker.scale.y = 0.3;
            goal_marker.scale.z = 0.3;
            goal_marker.color.r = 1.0;
            goal_marker.color.g = 0.0;
            goal_marker.color.b = 0.0;
            goal_marker.color.a = 1.0;
            marker_array.markers.push_back(goal_marker);
        }
        
        if (!marker_array.markers.empty()) {
            marker_pub_.publish(marker_array);
        }
    }
    
    void publishStopCommand() {
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
    }
    
    void publishStatus() {
        static ros::Time last_publish = ros::Time::now();
        if ((ros::Time::now() - last_publish).toSec() < 1.0) return;
        
        last_publish = ros::Time::now();
        
        std_msgs::String status_msg;
        std::string state_str;
        
        switch (current_state_) {
            case WAITING_FOR_MAP: state_str = "WAITING_FOR_MAP"; break;
            case WAITING_FOR_START: state_str = "WAITING_FOR_START"; break;
            case WAITING_FOR_GOAL: state_str = "WAITING_FOR_GOAL"; break;
            case BOTH_POINTS_SET: state_str = "BOTH_POINTS_SET"; break;
            case PLANNING: state_str = "PLANNING"; break;
            case EXECUTING: state_str = "EXECUTING"; break;
            case GOAL_REACHED: state_str = "GOAL_REACHED"; break;
            case FAILED: state_str = "FAILED"; break;
        }
        
        status_msg.data = state_str;
        status_pub_.publish(status_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ct_astar_dwa_navigation");
    
    try {
        FixedAStarDWANavigation navigation_node;
        navigation_node.run();
    } catch (const std::exception& e) {
        ROS_FATAL("Navigation system failed: %s", e.what());
        return 1;
    }
    
    return 0;
}