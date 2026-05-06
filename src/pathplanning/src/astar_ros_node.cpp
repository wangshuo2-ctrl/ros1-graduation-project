#include "pathplanning/Astar.h"
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

namespace pathplanning {

class AStarROSNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布器和订阅器
    ros::Subscriber map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher astar_global_path_pub_;
    ros::Publisher marker_pub_;
    
    // 状态标志
    bool map_initialized_;
    bool has_start_;
    bool has_goal_;
    
    // 当前起点和目标点
    geometry_msgs::PoseStamped current_start_;
    geometry_msgs::PoseStamped current_goal_;
    
    // 路径规划器
    Astar astar_;
    
    // 地图参数
    double resolution_;
    double origin_x_;
    double origin_y_;
    int width_;
    int height_;
    
    // 上次规划时间
    ros::Time last_planning_time_;
    
public:
    AStarROSNode() : 
        private_nh_("~"),
        map_initialized_(false),
        has_start_(false),
        has_goal_(false),
        resolution_(0.05),
        origin_x_(-5.0),
        origin_y_(-5.0),
        width_(200),
        height_(200)
    {
        ROS_INFO("AStar ROS Node initializing...");
        
        // 初始化发布器和订阅器
        initializeSubscribers();
        initializePublishers();
        
        // 初始化参数
        initializeParameters();
        
        ROS_INFO("AStar ROS Node initialized successfully");
        ROS_INFO("Waiting for map, start, and goal...");
    }
    
    ~AStarROSNode() {
        ROS_INFO("AStar ROS Node shutting down");
    }
    
    void run() {
        ros::spin();
    }
    
private:
    void initializeParameters() {
        // 从参数服务器获取参数
        AstarConfig config;
        private_nh_.param<double>("heuristic_weight", config.heuristic_weight, 1.0);
        private_nh_.param<int>("inflate_radius", config.inflate_radius, 3);
        private_nh_.param<double>("obstacle_clearance", config.obstacle_clearance, 0.0);
        private_nh_.param<double>("min_planning_interval", config.min_planning_interval, 0.5);
        private_nh_.param<double>("turn_penalty_weight", config.turn_penalty_weight, 0.5);
        private_nh_.param<bool>("euclidean", config.euclidean, true);
        private_nh_.param<bool>("allow_outside_map", config.allow_outside_map, false);
        private_nh_.param<int>("max_iterations", config.max_iterations, 10000);
        private_nh_.param<int>("max_search_radius", config.max_search_radius, 50);
        private_nh_.param<bool>("enable_path_smoothing", config.enable_path_smoothing, false);
        private_nh_.param<double>("safety_margin", config.safety_margin, 0.2);
        
        astar_.setConfig(config);
        
        ROS_INFO("AStar parameters loaded");
    }
    
    void initializeSubscribers() {
        // 订阅地图
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
            "/map", 1, &AStarROSNode::mapCallback, this);
        
        // 订阅起点
        start_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose", 1, &AStarROSNode::startCallback, this);
        
        // 订阅目标点
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 1, &AStarROSNode::goalCallback, this);
        
        ROS_INFO("Subscribers initialized");
    }
    
    void initializePublishers() {
        // 发布路径到/astar_global_path
        astar_global_path_pub_ = nh_.advertise<nav_msgs::Path>("/astar_global_path", 1, true);
        
        // 发布可视化标记
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/astar_markers", 1);
        
        ROS_INFO("Publishers initialized: /astar_global_path, /astar_markers");
    }
    
    // 世界坐标转地图坐标
    void worldToMap(double wx, double wy, int& mx, int& my) {
        mx = static_cast<int>((wx - origin_x_) / resolution_);
        my = static_cast<int>((wy - origin_y_) / resolution_);
        
        // 确保在地图范围内
        mx = std::max(0, std::min(mx, width_ - 1));
        my = std::max(0, std::min(my, height_ - 1));
    }
    
    // 地图坐标转世界坐标
    void mapToWorld(int mx, int my, double& wx, double& wy) {
        // 使用单元格中心
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        ROS_INFO("=== MAP CALLBACK ===");
        ROS_INFO("Map received: %dx%d, resolution: %.3f", 
                 msg->info.width, msg->info.height, msg->info.resolution);
        ROS_INFO("Map origin: (%.2f, %.2f)", 
                 msg->info.origin.position.x, msg->info.origin.position.y);
        
        // 更新地图参数
        resolution_ = msg->info.resolution;
        origin_x_ = msg->info.origin.position.x;
        origin_y_ = msg->info.origin.position.y;
        width_ = msg->info.width;
        height_ = msg->info.height;
        
        astar_.updateMap(msg);
        map_initialized_ = true;
        
        ROS_INFO("Map initialized and ready for planning");
        
        // 如果有起点和目标点，立即尝试规划
        if (has_start_ && has_goal_) {
            ROS_INFO("Map received with start and goal already set, triggering planning");
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
        
        try {
            // 执行路径规划
            std::vector<cv::Point> path_points;
            
            ROS_INFO("Starting A* path planning...");
            
            if (astar_.pathPlanning(path_points)) {
                ROS_INFO("A* planning successful, found %zu points", path_points.size());
                publishPath(path_points);
            } else {
                ROS_WARN("A* planning failed, using fallback straight line");
                
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
        int start_x, start_y, goal_x, goal_y;
        worldToMap(current_start_.pose.position.x, current_start_.pose.position.y, start_x, start_y);
        worldToMap(current_goal_.pose.position.x, current_goal_.pose.position.y, goal_x, goal_y);
        
        ROS_INFO("Generating straight line from (%d,%d) to (%d,%d)", 
                 start_x, start_y, goal_x, goal_y);
        
        // 计算距离
        int dx = goal_x - start_x;
        int dy = goal_y - start_y;
        float distance = sqrt(dx*dx + dy*dy);
        
        // 生成路径点
        int steps = std::max(20, static_cast<int>(distance));
        
        for (int i = 0; i <= steps; i++) {
            float t = static_cast<float>(i) / steps;
            int x = static_cast<int>(start_x + t * dx);
            int y = static_cast<int>(start_y + t * dy);
            
            // 确保在地图范围内
            x = std::max(0, std::min(x, width_ - 1));
            y = std::max(0, std::min(y, height_ - 1));
            
            path_points.push_back(cv::Point(x, y));
        }
        
        ROS_INFO("Generated straight line with %zu points", path_points.size());
    }
    
    void publishPath(const std::vector<cv::Point>& path_points) {
        if (path_points.empty()) {
            ROS_WARN("No path points to publish");
            return;
        }
        
        // 创建路径消息
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        
        ROS_INFO("Converting %zu map points to world coordinates...", path_points.size());
        
        // 将地图坐标转换为世界坐标
        for (const auto& point : path_points) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            
            double wx, wy;
            mapToWorld(point.x, point.y, wx, wy);
            
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path_msg.poses.push_back(pose);
        }
        
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
            line_marker.scale.x = 0.05;
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

} // namespace pathplanning

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_ros_node");
    
    try {
        ROS_INFO("Starting AStar ROS Node...");
        pathplanning::AStarROSNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in AStar ROS Node: %s", e.what());
        return 1;
    }
    
    return 0;
}
