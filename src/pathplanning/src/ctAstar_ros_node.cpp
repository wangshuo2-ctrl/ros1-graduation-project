#include "pathplanning/ctAstar.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutex>
#include <chrono>

namespace pathplanning {

class ctAStarROSNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    
    // 发布器
    ros::Publisher path_pub_;
    ros::Publisher markers_pub_;
    ros::Publisher map_pub_;  // 重新发布地图
    
    // TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    // 路径规划器
    ctAstar astar_;
    
    // 状态标志
    bool map_initialized_;
    bool has_start_;
    bool has_goal_;
    bool auto_replan_;
    bool enable_visualization_;
    
    // 坐标系名称
    std::string global_frame_;
    std::string robot_base_frame_;
    std::string laser_frame_;
    
    // 当前起点和目标点
    geometry_msgs::PoseStamped current_start_;
    geometry_msgs::PoseStamped current_goal_;
    
    // 最后规划的路径
    std::vector<cv::Point> last_path_;
    nav_msgs::Path last_path_msg_;
    ros::Time last_planning_time_;
    
    // 可视化参数
    double path_line_width_;
    double marker_lifetime_;
    double point_size_;
    
    // 互斥锁
    std::mutex planning_mutex_;
    
    // TF发布计时器
    ros::Timer tf_timer_;
    
public:
    ctAStarROSNode() : 
        private_nh_("~"),
        map_initialized_(false),
        has_start_(false),
        has_goal_(false),
        auto_replan_(true),
        enable_visualization_(true),
        path_line_width_(0.05),
        marker_lifetime_(0.0),
        point_size_(0.1),
        global_frame_("map"),
        robot_base_frame_("base_link"),
        laser_frame_("laser"),
        tf_broadcaster_(),
        static_tf_broadcaster_()
    {
        ROS_INFO("======================================");
        ROS_INFO("ctAstar ROS Node - Initializing");
        ROS_INFO("======================================");
        
        // 从参数服务器加载参数
        loadParameters();
        
        // 初始化A*规划器
        initializePlanner();
        
        // 初始化ROS接口
        initializeROSInterface();
        
        // 初始化静态TF变换
        initializeStaticTF();
        
        // 启动TF定时器
        tf_timer_ = nh_.createTimer(ros::Duration(0.1), 
                                    &ctAStarROSNode::tfTimerCallback, this);
        
        ROS_INFO("ctAstar ROS Node initialized successfully");
        ROS_INFO("Global frame: %s", global_frame_.c_str());
        ROS_INFO("Robot base frame: %s", robot_base_frame_.c_str());
    }
    
    void run() {
        ROS_INFO("ctAstar ROS Node started, waiting for input...");
        ros::spin();
    }
    
private:
    void loadParameters() {
        // 从参数服务器加载A*配置
        double heuristic_weight;
        int inflate_radius;
        bool use_euclidean;
        bool auto_heuristic;
        double min_planning_interval;
        
        // 算法参数
        private_nh_.param<double>("heuristic_weight", heuristic_weight, 1.0);
        private_nh_.param<int>("inflate_radius", inflate_radius, 1);
        private_nh_.param<bool>("euclidean", use_euclidean, true);
        private_nh_.param<bool>("auto_heuristic", auto_heuristic, true);
        private_nh_.param<double>("min_planning_interval", min_planning_interval, 0.5);
        
        astar_.initAstar(heuristic_weight, inflate_radius, use_euclidean);
        auto_replan_ = auto_heuristic;
        
        // 坐标系参数
        private_nh_.param<std::string>("global_frame", global_frame_, "map");
        private_nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        private_nh_.param<std::string>("laser_frame", laser_frame_, "laser");
        
        // 可视化参数
        private_nh_.param<bool>("enable_visualization", enable_visualization_, true);
        private_nh_.param<double>("path_line_width", path_line_width_, 0.05);
        private_nh_.param<double>("marker_lifetime", marker_lifetime_, 0.0);
        private_nh_.param<double>("point_size", point_size_, 0.1);
        
        ROS_INFO("Parameters loaded from parameter server");
    }
    
    void initializePlanner() {
        ROS_INFO("ctAstar planner initialized");
    }
    
    void initializeROSInterface() {
        // 初始化订阅器
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
            "/map", 1, &ctAStarROSNode::mapCallback, this);
        
        start_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose", 1, &ctAStarROSNode::startCallback, this);
        
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 1, &ctAStarROSNode::goalCallback, this);
        
        // 初始化发布器
        path_pub_ = nh_.advertise<nav_msgs::Path>("/astar_path", 1, true);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/astar_markers", 1);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/published_map", 1, true);
        
        ROS_INFO("ROS interface initialized");
        ROS_INFO("Subscribing to:");
        ROS_INFO("  - Map: /map");
        ROS_INFO("  - Start: /initialpose");
        ROS_INFO("  - Goal: /move_base_simple/goal");
        ROS_INFO("Publishing to:");
        ROS_INFO("  - Path: /astar_path");
        ROS_INFO("  - Markers: /astar_markers");
        ROS_INFO("  - Map: /published_map");
    }
    
    void initializeStaticTF() {
        ROS_INFO("Initializing static TF transforms...");
        
        // 发布静态TF变换
        publishStaticTransform("map", "odom", 0, 0, 0, 0, 0, 0);
        publishStaticTransform("odom", "base_link", 0, 0, 0, 0, 0, 0);
        publishStaticTransform("base_link", "laser", 0.1, 0, 0.2, 0, 0, 0);
        
        ROS_INFO("Static TF transforms initialized");
    }
    
    void publishStaticTransform(const std::string& parent, 
                                const std::string& child,
                                double x, double y, double z,
                                double roll, double pitch, double yaw) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = parent;
        transformStamped.child_frame_id = child;
        
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        
        static_tf_broadcaster_.sendTransform(transformStamped);
    }
    
    void tfTimerCallback(const ros::TimerEvent& event) {
        // 定期发布动态TF变换
        publishDynamicTFs();
    }
    
    void publishDynamicTFs() {
        // 如果没有起点，使用默认位置
        double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
        
        if (has_start_) {
            robot_x = current_start_.pose.position.x;
            robot_y = current_start_.pose.position.y;
            // 计算朝向（指向目标）
            if (has_goal_) {
                double dx = current_goal_.pose.position.x - robot_x;
                double dy = current_goal_.pose.position.y - robot_y;
                robot_yaw = atan2(dy, dx);
            }
        }
        
        // 发布map->base_link的TF（如果机器人移动）
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = global_frame_;
        transform.child_frame_id = robot_base_frame_;
        
        transform.transform.translation.x = robot_x;
        transform.transform.translation.y = robot_y;
        transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_yaw);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_.sendTransform(transform);
    }
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        ROS_INFO("=== MAP RECEIVED ===");
        ROS_INFO("Map info:");
        ROS_INFO("  Size: %dx%d pixels", msg->info.width, msg->info.height);
        ROS_INFO("  Resolution: %.3f m/pixel", msg->info.resolution);
        ROS_INFO("  Frame ID: %s", msg->header.frame_id.c_str());
        
        // 确保地图的frame_id正确
        nav_msgs::OccupancyGrid corrected_map = *msg;
        if (corrected_map.header.frame_id.empty()) {
            corrected_map.header.frame_id = global_frame_;
            ROS_INFO("  Setting frame_id to: %s", global_frame_.c_str());
        }
        
        // 更新规划器地图
        if (astar_.updateMap(msg)) {
            map_initialized_ = true;
            
            // 重新发布地图，确保frame_id正确
            corrected_map.header.stamp = ros::Time::now();
            map_pub_.publish(corrected_map);
            
            ROS_INFO("Map initialized successfully");
            
            if (has_start_ && has_goal_) {
                ROS_INFO("Map received with existing start and goal, triggering replan");
                triggerPlanning();
            }
        } else {
            ROS_ERROR("Failed to update map in planner");
        }
    }
    
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        ROS_INFO("=== START POINT SET ===");
        ROS_INFO("Start position:");
        ROS_INFO("  World: (%.2f, %.2f)", 
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y);
        
        // 保存起点
        current_start_.header = msg->header;
        current_start_.pose = msg->pose.pose;
        has_start_ = true;
        
        // 更新规划器起点
        if (astar_.setStartPoint(msg)) {
            ROS_INFO("Start point set successfully");
            
            // 发布可视化标记
            if (enable_visualization_) {
                publishStartMarker();
            }
            
            if (map_initialized_ && has_goal_) {
                ROS_INFO("Start received with map and goal, triggering planning");
                triggerPlanning();
            } else if (!map_initialized_) {
                ROS_WARN("Start received but map not initialized yet");
            } else if (!has_goal_) {
                ROS_WARN("Start received but goal not set yet");
            }
        } else {
            ROS_ERROR("Failed to set start point in planner");
        }
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ROS_INFO("=== GOAL POINT SET ===");
        ROS_INFO("Goal position:");
        ROS_INFO("  World: (%.2f, %.2f)", 
                 msg->pose.position.x,
                 msg->pose.position.y);
        
        // 保存目标点
        current_goal_ = *msg;
        has_goal_ = true;
        
        // 更新规划器目标点
        if (astar_.setTargetPoint(msg)) {
            ROS_INFO("Goal point set successfully");
            
            // 发布可视化标记
            if (enable_visualization_) {
                publishGoalMarker();
            }
            
            if (map_initialized_ && has_start_) {
                ROS_INFO("Goal received with map and start, triggering planning");
                triggerPlanning();
            } else if (!map_initialized_) {
                ROS_WARN("Goal received but map not initialized yet");
            } else if (!has_start_) {
                ROS_WARN("Goal received but start not set yet");
            }
        } else {
            ROS_ERROR("Failed to set goal point in planner");
        }
    }
    
    void triggerPlanning() {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        
        if (!map_initialized_ || !has_start_ || !has_goal_) {
            ROS_WARN("Cannot trigger planning: prerequisites not met");
            ROS_WARN("  Map initialized: %s", map_initialized_ ? "YES" : "NO");
            ROS_WARN("  Has start: %s", has_start_ ? "YES" : "NO");
            ROS_WARN("  Has goal: %s", has_goal_ ? "YES" : "NO");
            return;
        }
        
        // 检查规划间隔
        ros::Time now = ros::Time::now();
        double time_since_last_plan = (now - last_planning_time_).toSec();
        
        if (time_since_last_plan < 0.1) {  // 最小间隔100ms
            ROS_DEBUG("Skipping planning, too frequent: %.3f sec", time_since_last_plan);
            return;
        }
        
        ROS_INFO("=== A* PATH PLANNING TRIGGERED ===");
        
        // 执行路径规划
        std::vector<cv::Point> path_points;
        
        try {
            // 调用A*算法进行路径规划
            if (astar_.pathPlanning(path_points)) {
                ROS_INFO("A* planning SUCCESSFUL");
                ROS_INFO("  Path points: %zu", path_points.size());
                
                // 保存路径
                last_path_ = path_points;
                
                // 转换为ROS路径消息并发布
                last_path_msg_ = astar_.convertToWorldPath(path_points);
                last_path_msg_.header.frame_id = global_frame_;
                last_path_msg_.header.stamp = ros::Time::now();
                path_pub_.publish(last_path_msg_);
                
                // 发布可视化
                if (enable_visualization_) {
                    publishPathVisualization();
                }
                
                // 记录最后规划时间
                last_planning_time_ = now;
            } else {
                ROS_ERROR("A* planning FAILED");
                
                // 发布空路径，清除之前的路径显示
                nav_msgs::Path empty_path;
                empty_path.header.stamp = ros::Time::now();
                empty_path.header.frame_id = global_frame_;
                path_pub_.publish(empty_path);
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception during path planning: %s", e.what());
        } catch (...) {
            ROS_ERROR("Unknown exception during path planning");
        }
        
        ROS_INFO("=== A* PATH PLANNING COMPLETE ===");
    }
    
    void publishPathVisualization() {
        if (last_path_msg_.poses.empty()) {
            return;
        }
        
        visualization_msgs::MarkerArray marker_array;
        
        // 1. 清除之前的路径标记
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = global_frame_;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "astar_path";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 2. 创建路径线标记
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = global_frame_;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "astar_path";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        // 设置线属性
        line_marker.scale.x = path_line_width_;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;
        line_marker.lifetime = ros::Duration(marker_lifetime_);
        
        // 添加路径点
        for (const auto& pose : last_path_msg_.poses) {
            geometry_msgs::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = 0.0;
            line_marker.points.push_back(point);
        }
        
        marker_array.markers.push_back(line_marker);
        
        // 3. 创建路径点标记
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = global_frame_;
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "astar_path_points";
        points_marker.id = 2;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        
        points_marker.scale.x = point_size_ * 0.5;
        points_marker.scale.y = point_size_ * 0.5;
        points_marker.scale.z = point_size_ * 0.5;
        points_marker.color.r = 0.0;
        points_marker.color.g = 0.8;
        points_marker.color.b = 1.0;
        points_marker.color.a = 0.8;
        points_marker.lifetime = ros::Duration(marker_lifetime_);
        
        for (const auto& pose : last_path_msg_.poses) {
            geometry_msgs::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = 0.0;
            points_marker.points.push_back(point);
        }
        
        marker_array.markers.push_back(points_marker);
        
        // 发布标记
        markers_pub_.publish(marker_array);
    }
    
    void publishStartMarker() {
        visualization_msgs::MarkerArray marker_array;
        
        // 清除之前的起点标记
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = global_frame_;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "astar_start";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 创建起点标记
        visualization_msgs::Marker start_marker;
        start_marker.header.frame_id = global_frame_;
        start_marker.header.stamp = ros::Time::now();
        start_marker.ns = "astar_start";
        start_marker.id = 1;
        start_marker.type = visualization_msgs::Marker::SPHERE;
        start_marker.action = visualization_msgs::Marker::ADD;
        
        start_marker.pose = current_start_.pose;
        start_marker.pose.position.z = 0.0;
        start_marker.pose.orientation.w = 1.0;
        
        start_marker.scale.x = point_size_;
        start_marker.scale.y = point_size_;
        start_marker.scale.z = point_size_;
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        start_marker.lifetime = ros::Duration(marker_lifetime_);
        
        marker_array.markers.push_back(start_marker);
        
        // 创建文本标记
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = global_frame_;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "astar_start_text";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose = current_start_.pose;
        text_marker.pose.position.z += point_size_ + 0.1;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.3;
        text_marker.color.r = 0.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 0.0;
        text_marker.color.a = 1.0;
        text_marker.lifetime = ros::Duration(marker_lifetime_);
        text_marker.text = "Start";
        
        marker_array.markers.push_back(text_marker);
        
        markers_pub_.publish(marker_array);
    }
    
    void publishGoalMarker() {
        visualization_msgs::MarkerArray marker_array;
        
        // 清除之前的目标点标记
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = global_frame_;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "astar_goal";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 创建目标点标记
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = global_frame_;
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "astar_goal";
        goal_marker.id = 1;
        goal_marker.type = visualization_msgs::Marker::SPHERE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        
        goal_marker.pose = current_goal_.pose;
        goal_marker.pose.position.z = 0.0;
        goal_marker.pose.orientation.w = 1.0;
        
        goal_marker.scale.x = point_size_;
        goal_marker.scale.y = point_size_;
        goal_marker.scale.z = point_size_;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        goal_marker.color.a = 1.0;
        goal_marker.lifetime = ros::Duration(marker_lifetime_);
        
        marker_array.markers.push_back(goal_marker);
        
        // 创建文本标记
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = global_frame_;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "astar_goal_text";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose = current_goal_.pose;
        text_marker.pose.position.z += point_size_ + 0.1;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 0.0;
        text_marker.color.b = 0.0;
        text_marker.color.a = 1.0;
        text_marker.lifetime = ros::Duration(marker_lifetime_);
        text_marker.text = "Goal";
        
        marker_array.markers.push_back(text_marker);
        
        markers_pub_.publish(marker_array);
    }
};

} // namespace pathplanning

int main(int argc, char** argv) {
    ros::init(argc, argv, "ctAstar_ros_node");
    
    ROS_INFO("======================================");
    ROS_INFO("Starting ctAstar ROS Node");
    ROS_INFO("======================================");
    
    try {
        pathplanning::ctAStarROSNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Fatal error in ctAstar ROS Node: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Unknown fatal error in ctAstar ROS Node");
        return 1;
    }
    
    ROS_INFO("ctAstar ROS Node stopped");
    return 0;
}