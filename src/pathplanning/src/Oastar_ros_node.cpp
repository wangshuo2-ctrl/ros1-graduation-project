#include "pathplanning/OAstar.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <std_srvs/SetBool.h>

namespace pathplanning {

class OAStarROSNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布器和订阅器
    ros::Subscriber original_map_sub_;  // 订阅原始地图
    ros::Subscriber obstacle_map_sub_;  // 订阅障碍物地图
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher astar_global_path_pub_;
    ros::Publisher marker_pub_;
    
    // 服务
    ros::ServiceServer ignore_obstacles_service_;
    ros::ServiceServer lock_map_service_;
    ros::ServiceServer reset_first_plan_service_;
    
    // 状态标志
    bool map_initialized_;
    bool has_start_;
    bool has_goal_;
    
    // 当前起点和目标点
    geometry_msgs::PoseStamped current_start_;
    geometry_msgs::PoseStamped current_goal_;
    
    // 路径规划器
    OAstar astar_;
    
    // 地图参数
    double resolution_;
    double origin_x_;
    double origin_y_;
    int width_;
    int height_;
    
    // 上次规划时间
    ros::Time last_planning_time_;

public:
    OAStarROSNode():
        private_nh_("~"),
        map_initialized_(false),
        has_start_(false),
        has_goal_(false),
        resolution_(0.05),
        origin_x_(-5.0),
        origin_y_(-5.0),
        width_(200),
        height_(200) {
        ROS_INFO("OAStar ROS Node initializing...");
        
        // 初始化发布器和订阅器
        initializeSubscribers();
        initializePublishers();
        initializeServices();
        
        // 初始化参数
        initializeParameters();
        
        ROS_INFO("OAStar ROS Node initialized successfully");
        ROS_INFO("Waiting for map, start, and goal...");
    }
    
    ~OAStarROSNode() {
        ROS_INFO("OAStar ROS Node shutting down");
    }
    
    void run() {
        ros::spin();
    }

private:
    void initializeParameters() {
        // 从参数服务器获取参数
        OAstarConfig config;
        
        private_nh_.param<double>("heuristic_weight", config.heuristic_weight, 1.0);
        private_nh_.param<int>("inflate_radius", config.inflate_radius, 3);
        private_nh_.param<double>("safety_margin", config.safety_margin, 0.2);
        private_nh_.param<double>("obstacle_clearance", config.obstacle_clearance, 0.0);
        private_nh_.param<double>("turn_penalty_weight", config.turn_penalty_weight, 0.5);
        private_nh_.param<bool>("euclidean", config.euclidean, true);
        private_nh_.param<bool>("allow_outside_map", config.allow_outside_map, false);
        private_nh_.param<int>("max_iterations", config.max_iterations, 10000);
        
        // 平滑相关参数
        private_nh_.param<bool>("enable_path_smoothing", config.enable_path_smoothing, false);
        private_nh_.param<bool>("enable_bspline_smoothing", config.enable_bspline_smoothing, false);
        private_nh_.param<bool>("enable_path_resample", config.enable_path_resample, false);
        
        // 安全检查参数
        private_nh_.param<bool>("enable_safety_check", config.enable_safety_check, false);
        private_nh_.param<double>("safety_check_distance", config.safety_check_distance, 0.4);
        
        // 曲率约束参数
        private_nh_.param<bool>("enable_curvature_constraint", config.enable_curvature_constraint, false);
        private_nh_.param<double>("min_turn_radius", config.min_turn_radius, 0.7);
        private_nh_.param<double>("max_curvature", config.max_curvature, 1.5);
        
        // OA*特定参数
        private_nh_.param<bool>("ignore_dynamic_obstacles", config.ignore_dynamic_obstacles, true);
        private_nh_.param<bool>("lock_original_map", config.lock_original_map, true);
        
        // 设置规划器配置
        astar_.setConfig(config);
        astar_.setIgnoreDynamicObstacles(config.ignore_dynamic_obstacles);
        astar_.setLockOriginalMap(config.lock_original_map);
        
        ROS_INFO("OAStar parameters loaded");
        ROS_INFO("  ignore_dynamic_obstacles: %s", config.ignore_dynamic_obstacles ? "true" : "false");
        ROS_INFO("  lock_original_map: %s", config.lock_original_map ? "true" : "false");
    }
    
    void initializeSubscribers() {
        // 订阅原始地图
        original_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
            "/original_map", 1, &OAStarROSNode::originalMapCallback, this);
        
        // 订阅障碍物地图
        obstacle_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
            "/random_obstacle_map", 1, &OAStarROSNode::obstacleMapCallback, this);
        
        // 订阅起点
        start_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose", 1, &OAStarROSNode::startCallback, this);
        
        // 订阅目标点
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 1, &OAStarROSNode::goalCallback, this);
        
        ROS_INFO("Subscribers initialized");
    }
    
    void initializePublishers() {
        // 发布路径到/astar_global_path
        astar_global_path_pub_ = nh_.advertise<nav_msgs::Path>("/astar_global_path", 1, true);
        
        // 发布可视化标记
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/astar_markers", 1);
        
        ROS_INFO("Publishers initialized: /astar_global_path, /astar_markers");
    }
    
    void initializeServices() {
        // 忽略障碍物服务
        ignore_obstacles_service_ = private_nh_.advertiseService(
            "ignore_dynamic_obstacles", 
            &OAStarROSNode::ignoreObstaclesCallback, 
            this);
        
        // 锁定地图服务
        lock_map_service_ = private_nh_.advertiseService(
            "lock_original_map", 
            &OAStarROSNode::lockMapCallback, 
            this);
        
        // 重置第一次规划服务
        reset_first_plan_service_ = private_nh_.advertiseService(
            "reset_first_plan", 
            &OAStarROSNode::resetFirstPlanCallback, 
            this);
        
        ROS_INFO("Services initialized:");
        ROS_INFO("  /oastar_global_planner/ignore_dynamic_obstacles");
        ROS_INFO("  /oastar_global_planner/lock_original_map");
        ROS_INFO("  /oastar_global_planner/reset_first_plan");
    }
    
    void originalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        ROS_INFO("=== ORIGINAL MAP CALLBACK ===");
        ROS_INFO("Original map received: %dx%d, resolution: %.3f",
                 msg->info.width, msg->info.height, msg->info.resolution);
        ROS_INFO("Map origin: (%.2f, %.2f)",
                 msg->info.origin.position.x, msg->info.origin.position.y);
        
        // 更新地图参数
        resolution_ = msg->info.resolution;
        origin_x_ = msg->info.origin.position.x;
        origin_y_ = msg->info.origin.position.y;
        width_ = msg->info.width;
        height_ = msg->info.height;
        
        // 更新A*地图
        astar_.updateMap(msg);
        map_initialized_ = true;
        
        ROS_INFO("Original map initialized and ready for planning");
        
        // 如果有起点和目标点，立即尝试规划
        if (has_start_ && has_goal_) {
            ROS_INFO("Original map received with start and goal already set, triggering planning");
            triggerPlanning();
        }
    }
    
    void obstacleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        ROS_INFO("=== OBSTACLE MAP CALLBACK ===");
        
        // 无论是否忽略动态障碍物，都更新地图
        // OAstar内部会根据配置决定是否处理这个地图
        astar_.updateMap(msg);
        
        // 如果已有地图，立即尝试规划
        if (map_initialized_ && has_start_ && has_goal_) {
            ROS_INFO("Obstacle map updated, re-triggering planning");
            triggerPlanning();
        }
    }
    
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        ROS_INFO("=== START CALLBACK ===");
        ROS_INFO("Start point: (%.2f, %.2f)",
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y);
        
        // 保存起点
        current_start_.header = msg->header;
        current_start_.pose = msg->pose.pose;
        has_start_ = true;
        
        astar_.setStartPoint(msg);
        
        // 如果已有地图，开始规划
        if (map_initialized_) {
            if (has_goal_) {
                ROS_INFO("Start received with map and goal, triggering planning");
                triggerPlanning();
            } else {
                ROS_INFO("Start received, waiting for goal...");
            }
        } else {
            ROS_INFO("Start received, waiting for map...");
        }
        
        // 发布标记
        publishMarkers();
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ROS_INFO("=== GOAL CALLBACK ===");
        ROS_INFO("Goal point: (%.2f, %.2f)",
                 msg->pose.position.x,
                 msg->pose.position.y);
        ROS_INFO("Frame ID: %s", msg->header.frame_id.c_str());
        
        // 保存目标点
        current_goal_ = *msg;
        has_goal_ = true;
        
        astar_.setTargetPoint(msg);
        
        // 如果已有地图，开始规划
        if (map_initialized_) {
            if (has_start_) {
                ROS_INFO("Goal received with map and start, triggering planning");
                triggerPlanning();
            } else {
                ROS_INFO("Goal received, waiting for start...");
            }
        } else {
            ROS_INFO("Goal received, waiting for map...");
        }
        
        // 发布标记
        publishMarkers();
    }
    
    bool ignoreObstaclesCallback(std_srvs::SetBool::Request& req,
                                 std_srvs::SetBool::Response& res) {
        astar_.setIgnoreDynamicObstacles(req.data);
        res.success = true;
        res.message = req.data ? 
            "Now ignoring dynamic obstacles" : 
            "Now considering dynamic obstacles";
        
        ROS_INFO("%s", res.message.c_str());
        
        // 如果已准备好，重新规划
        if (map_initialized_ && has_start_ && has_goal_) {
            triggerPlanning();
        }
        
        return true;
    }
    
    bool lockMapCallback(std_srvs::SetBool::Request& req,
                         std_srvs::SetBool::Response& res) {
        astar_.setLockOriginalMap(req.data);
        res.success = true;
        res.message = req.data ? 
            "Original map locked" : 
            "Original map unlocked";
        
        ROS_INFO("%s", res.message.c_str());
        
        // 如果已准备好，重新规划
        if (map_initialized_ && has_start_ && has_goal_) {
            triggerPlanning();
        }
        
        return true;
    }
    
    bool resetFirstPlanCallback(std_srvs::SetBool::Request& req,
                                std_srvs::SetBool::Response& res) {
        if (req.data) {
            astar_.resetFirstPlan();
            res.success = true;
            res.message = "First plan reset. Next planning will compute new path.";
            ROS_INFO("First plan reset");
            
            // 如果已准备好，重新规划
            if (map_initialized_ && has_start_ && has_goal_) {
                triggerPlanning();
            }
        } else {
            res.success = false;
            res.message = "Reset first plan requires data: true";
        }
        
        return true;
    }
    
    void triggerPlanning() {
        if (!map_initialized_ || !has_start_ || !has_goal_) {
            ROS_WARN("Cannot trigger planning: map=%d, start=%d, goal=%d",
                     map_initialized_, has_start_, has_goal_);
            return;
        }
        
        // 检查规划间隔
        ros::Time now = ros::Time::now();
        if ((now - last_planning_time_).toSec() < 0.1) {
            ROS_DEBUG("Skipping planning, too frequent");
            return;
        }
        
        last_planning_time_ = now;
        
        ROS_INFO("=== TRIGGER PLANNING ===");
        ROS_INFO("From: (%.2f, %.2f)",
                 current_start_.pose.position.x, current_start_.pose.position.y);
        ROS_INFO("To: (%.2f, %.2f)",
                 current_goal_.pose.position.x, current_goal_.pose.position.y);
        
        astar_.printDebugInfo();
        
        try {
            // 执行路径规划
            std::vector<cv::Point> path_points;
            ROS_INFO("Starting OA* path planning...");
            
            if (astar_.pathPlanning(path_points)) {
                ROS_INFO("OA* planning successful, found %zu points", path_points.size());
                publishPath(path_points);
            } else {
                ROS_WARN("OA* planning failed, using fallback straight line");
                // 备用方案：生成直线路径
                generateStraightPath(path_points);
                publishPath(path_points);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Exception during planning: %s", e.what());
            // 异常情况下生成直线路径
            std::vector<cv::Point> path_points;
            generateStraightPath(path_points);
            publishPath(path_points);
        } catch (...) {
            ROS_ERROR("Unknown exception during planning");
            // 异常情况下生成直线路径
            std::vector<cv::Point> path_points;
            generateStraightPath(path_points);
            publishPath(path_points);
        }
    }
    
    void generateStraightPath(std::vector<cv::Point>& path_points) {
        path_points.clear();
        
        // 将起点和终点转换为地图坐标
        cv::Point start_pt = astar_.worldToMap(
            current_start_.pose.position.x, 
            current_start_.pose.position.y
        );
        cv::Point goal_pt = astar_.worldToMap(
            current_goal_.pose.position.x, 
            current_goal_.pose.position.y
        );
        
        ROS_INFO("Generating straight line from (%d,%d) to (%d,%d)",
                 start_pt.x, start_pt.y, goal_pt.x, goal_pt.y);
        
        // Bresenham直线算法
        int x1 = start_pt.x, y1 = start_pt.y;
        int x2 = goal_pt.x, y2 = goal_pt.y;
        
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            path_points.push_back(cv::Point(x1, y1));
            
            if (x1 == x2 && y1 == y2) {
                break;
            }
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
        
        ROS_INFO("Generated straight line with %zu points", path_points.size());
    }
    
    void publishPath(const std::vector<cv::Point>& path_points) {
        if (path_points.empty()) {
            ROS_WARN("No path points to publish");
            return;
        }
        
        // 创建路径消息
        nav_msgs::Path path_msg = astar_.convertToWorldPath(path_points);
        
        // 发布到路径话题
        astar_global_path_pub_.publish(path_msg);
        ROS_INFO("Published path to /astar_global_path with %zu points", path_msg.poses.size());
        
        // 发布可视化标记
        publishPathMarkers(path_msg);
    }
    
    void publishPathMarkers(const nav_msgs::Path& path) {
        visualization_msgs::MarkerArray marker_array;
        
        // 清理之前的标记
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "astar_path";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 路径线
        if (path.poses.size() > 1) {
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "astar_path";
            line_marker.id = 1;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.1;  // 加粗路径线
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;
            line_marker.lifetime = ros::Duration();
            
            for (const auto& pose : path.poses) {
                geometry_msgs::Point p;
                p.x = pose.pose.position.x;
                p.y = pose.pose.position.y;
                p.z = 0.0;
                line_marker.points.push_back(p);
            }
            
            marker_array.markers.push_back(line_marker);
        }
        
        marker_pub_.publish(marker_array);
    }
    
    void publishMarkers() {
        visualization_msgs::MarkerArray marker_array;
        
        // 清理之前的标记
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "astar";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 发布起点标记
        if (has_start_) {
            visualization_msgs::Marker start_marker;
            start_marker.header.frame_id = "map";
            start_marker.header.stamp = ros::Time::now();
            start_marker.ns = "astar";
            start_marker.id = 1;
            start_marker.type = visualization_msgs::Marker::SPHERE;
            start_marker.action = visualization_msgs::Marker::ADD;
            start_marker.pose = current_start_.pose;
            start_marker.pose.orientation.w = 1.0;
            start_marker.scale.x = 0.3;
            start_marker.scale.y = 0.3;
            start_marker.scale.z = 0.3;
            start_marker.color.r = 0.0;
            start_marker.color.g = 1.0;
            start_marker.color.b = 0.0;
            start_marker.color.a = 1.0;
            start_marker.lifetime = ros::Duration();
            marker_array.markers.push_back(start_marker);
        }
        
        // 发布目标点标记
        if (has_goal_) {
            visualization_msgs::Marker goal_marker;
            goal_marker.header.frame_id = "map";
            goal_marker.header.stamp = ros::Time::now();
            goal_marker.ns = "astar";
            goal_marker.id = 2;
            goal_marker.type = visualization_msgs::Marker::SPHERE;
            goal_marker.action = visualization_msgs::Marker::ADD;
            goal_marker.pose = current_goal_.pose;
            goal_marker.pose.orientation.w = 1.0;
            goal_marker.scale.x = 0.3;
            goal_marker.scale.y = 0.3;
            goal_marker.scale.z = 0.3;
            goal_marker.color.r = 1.0;
            goal_marker.color.g = 0.0;
            goal_marker.color.b = 0.0;
            goal_marker.color.a = 1.0;
            goal_marker.lifetime = ros::Duration();
            marker_array.markers.push_back(goal_marker);
        }
        
        marker_pub_.publish(marker_array);
    }
};

}  // namespace pathplanning

int main(int argc, char** argv) {
    ros::init(argc, argv, "oastar_ros_node");
    
    try {
        ROS_INFO("Starting OAStar ROS Node...");
        pathplanning::OAStarROSNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in OAStar ROS Node: %s", e.what());
        return 1;
    }
    
    return 0;
}