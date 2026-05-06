#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

class RandomObstaclePublisher {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅与发布
    ros::Publisher costmap_pub_;
    ros::Publisher markers_pub_;
    ros::Subscriber click_subscriber_;  // 新增：订阅鼠标点击事件
    
    // 服务
    ros::ServiceServer random_obstacle_service_;
    ros::ServiceServer clear_obstacle_service_;
    ros::ServiceServer update_parameters_service_;
    ros::ServiceServer manual_obstacle_service_;  // 新增：手动放置障碍物服务
    
    // 服务客户端：用于获取原始静态地图
    ros::ServiceClient static_map_client_;

    // 数据
    nav_msgs::OccupancyGrid original_map_;
    nav_msgs::OccupancyGrid obstacle_map_;
    std::vector<geometry_msgs::Point> obstacle_centers_;
    
    // 参数
    int obstacle_count_;
    double obstacle_radius_;
    int obstacle_cost_value_;
    double update_rate_;
    bool enable_random_movement_;
    bool map_received_;
    bool allow_overlap_;  // 新增：是否允许障碍物重叠

    ros::Timer update_timer_;
    
    // 参数动态更新标志
    bool parameters_changed_;
    
    // 互斥锁，保护共享数据
    std::mutex data_mutex_;

public:
    RandomObstaclePublisher() : 
        private_nh_("~"),
        obstacle_count_(5),
        obstacle_radius_(0.3),
        obstacle_cost_value_(100),
        update_rate_(1.0),
        enable_random_movement_(false),
        map_received_(false),
        allow_overlap_(false),  // 默认不允许重叠
        parameters_changed_(false)
    {
        // 初始化随机种子
        std::srand(std::time(0));
        
        // 从参数服务器获取配置
        loadParameters();
        
        // 发布包含障碍物的代价地图
        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/random_obstacle_map", 1, true);
        
        // 发布障碍物的可视化标记
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/random_obstacles", 1, true);
        
        // 订阅鼠标点击事件
        click_subscriber_ = nh_.subscribe("/clicked_point", 10, 
                                         &RandomObstaclePublisher::clickCallback, this);
        
        // 提供服务
        random_obstacle_service_ = nh_.advertiseService("/generate_random_obstacles", 
                                                        &RandomObstaclePublisher::generateRandomObstaclesCallback, this);
        clear_obstacle_service_ = nh_.advertiseService("/clear_random_obstacles", 
                                                       &RandomObstaclePublisher::clearObstaclesCallback, this);
        update_parameters_service_ = nh_.advertiseService("/update_random_obstacle_parameters",
                                                         &RandomObstaclePublisher::updateParametersCallback, this);
        // 新增：手动放置障碍物服务
        manual_obstacle_service_ = nh_.advertiseService("/add_manual_obstacle", 
                                                       &RandomObstaclePublisher::addManualObstacleCallback, this);

        ROS_INFO("Enhanced Obstacle Publisher Initializing...");
        ROS_INFO("Features: Random generation + Manual placement");
        ROS_INFO("Parameters: count=%d, radius=%.2f, cost=%d", 
                 obstacle_count_, obstacle_radius_, obstacle_cost_value_);
        
        // 获取原始静态地图
        ROS_INFO("Waiting for static_map service...");
        ros::service::waitForService("/static_map");
        static_map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
        
        if (loadOriginalMap()) {
            ROS_INFO("Enhanced Obstacle Publisher Ready.");
            ROS_INFO("Services available:");
            ROS_INFO("  /generate_random_obstacles - Generate random obstacles");
            ROS_INFO("  /add_manual_obstacle - Add obstacle at specific position");
            ROS_INFO("  /clear_random_obstacles - Clear all obstacles");
            ROS_INFO("  /update_random_obstacle_parameters - Update obstacle parameters");
            
            ROS_INFO("Control Methods:");
            ROS_INFO("  1. Use RViz Point tool to click and place obstacles");
            ROS_INFO("  2. Use /add_manual_obstacle service to programmatically add obstacles");
            ROS_INFO("  3. Use GUI to input coordinates and place obstacles");
            
            // 初始化障碍物地图
            obstacle_map_ = original_map_;
            obstacle_map_.header.frame_id = "map";
            
            // 初始发布一次空地图
            publishCostmap();
            publishMarkers();
            
            // 启动定时器（如果启用了随机移动）
            if (enable_random_movement_) {
                update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), 
                                                &RandomObstaclePublisher::timerCallback, this);
                ROS_INFO("Random movement enabled. Obstacles will move randomly.");
            }
            
            // 创建参数动态更新定时器
            ros::Timer param_timer = nh_.createTimer(ros::Duration(1.0), 
                                                     &RandomObstaclePublisher::checkParametersCallback, this);
        } else {
            ROS_ERROR("Failed to load original map. Make sure map_server is running.");
        }
    }
    
    void loadParameters() {
        // 从参数服务器获取最新参数
        int new_obstacle_count;
        double new_obstacle_radius;
        int new_obstacle_cost_value;
        double new_update_rate;
        bool new_enable_random_movement;
        bool new_allow_overlap;
        
        // 获取参数
        private_nh_.param("obstacle_count", new_obstacle_count, obstacle_count_);
        private_nh_.param("obstacle_radius", new_obstacle_radius, obstacle_radius_);
        private_nh_.param("obstacle_cost_value", new_obstacle_cost_value, obstacle_cost_value_);
        private_nh_.param("update_rate", new_update_rate, update_rate_);
        private_nh_.param("enable_random_movement", new_enable_random_movement, enable_random_movement_);
        private_nh_.param("allow_overlap", new_allow_overlap, allow_overlap_);
        
        // 检查参数是否变化
        if (new_obstacle_count != obstacle_count_ || 
            fabs(new_obstacle_radius - obstacle_radius_) > 0.001 ||
            new_obstacle_cost_value != obstacle_cost_value_ ||
            fabs(new_update_rate - update_rate_) > 0.001 ||
            new_enable_random_movement != enable_random_movement_ ||
            new_allow_overlap != allow_overlap_) {
            
            parameters_changed_ = true;
            
            // 更新参数
            obstacle_count_ = new_obstacle_count;
            obstacle_radius_ = new_obstacle_radius;
            obstacle_cost_value_ = new_obstacle_cost_value;
            update_rate_ = new_update_rate;
            enable_random_movement_ = new_enable_random_movement;
            allow_overlap_ = new_allow_overlap;
            
            ROS_INFO("Parameters updated: count=%d, radius=%.2f, cost=%d, allow_overlap=%s", 
                     obstacle_count_, obstacle_radius_, obstacle_cost_value_, 
                     allow_overlap_ ? "true" : "false");
        }
    }

    bool loadOriginalMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            original_map_ = srv.response.map;
            map_received_ = true;
            ROS_INFO("Original map loaded: %d x %d, resolution: %.3f", 
                     original_map_.info.width, original_map_.info.height, 
                     original_map_.info.resolution);
            ROS_INFO("Map origin: (%.2f, %.2f)", 
                     original_map_.info.origin.position.x,
                     original_map_.info.origin.position.y);
            return true;
        } else {
            ROS_ERROR("Failed to call /static_map service");
            return false;
        }
    }
    
    // 检查参数变化的回调函数
    void checkParametersCallback(const ros::TimerEvent&) {
        loadParameters();
        
        // 如果参数发生变化且障碍物已存在，则重新绘制
        if (parameters_changed_ && !obstacle_centers_.empty()) {
            ROS_INFO("Obstacle parameters changed. Updating display...");
            updateCostmap();
            publishCostmap();
            publishMarkers();
            parameters_changed_ = false;
        }
    }
    
    // 服务回调：更新参数
    bool updateParametersCallback(std_srvs::Empty::Request& req, 
                                  std_srvs::Empty::Response& res) {
        loadParameters();
        
        // 如果障碍物已存在，则重新绘制
        if (!obstacle_centers_.empty()) {
            ROS_INFO("Obstacle parameters updated. Redrawing obstacles...");
            updateCostmap();
            publishCostmap();
            publishMarkers();
        }
        
        return true;
    }
    
    // 新增：手动添加障碍物服务回调
    bool addManualObstacleCallback(std_srvs::Trigger::Request& req, 
                                   std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 从参数服务器获取要添加的位置
        double x, y;
        if (!private_nh_.getParam("manual_obstacle_x", x) || 
            !private_nh_.getParam("manual_obstacle_y", y)) {
            res.success = false;
            res.message = "Please set manual_obstacle_x and manual_obstacle_y parameters first";
            return true;
        }
        
        if (addObstacleAtPosition(x, y)) {
            res.success = true;
            res.message = "Manual obstacle added at (" + std::to_string(x) + ", " + std::to_string(y) + ")";
            
            // 更新并发布地图
            updateCostmap();
            publishCostmap();
            publishMarkers();
        } else {
            res.success = false;
            res.message = "Failed to add manual obstacle at (" + std::to_string(x) + ", " + std::to_string(y) + ")";
        }
        
        return true;
    }

    // 服务回调：生成随机障碍物
    bool generateRandomObstaclesCallback(std_srvs::Empty::Request& req, 
                                         std_srvs::Empty::Response& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (!map_received_) {
            ROS_WARN("Cannot generate obstacles: map not loaded.");
            return false;
        }
        
        // 获取最新参数
        loadParameters();
        
        // 清除现有障碍物
        obstacle_centers_.clear();
        
        // 在自由空间随机生成障碍物
        int obstacles_placed = 0;
        int max_attempts = obstacle_count_ * 100;  // 使用动态参数，不再是固定值
        
        ROS_INFO("Attempting to place %d obstacles with radius %.2f...", 
                 obstacle_count_, obstacle_radius_);
        
        for (int attempts = 0; attempts < max_attempts && obstacles_placed < obstacle_count_; ++attempts) {
            // 随机选择地图内的一个坐标（世界坐标系）
            double world_x = original_map_.info.origin.position.x + 
                            (std::rand() % original_map_.info.width) * original_map_.info.resolution;
            double world_y = original_map_.info.origin.position.y + 
                            (std::rand() % original_map_.info.height) * original_map_.info.resolution;
            
            if (addObstacleAtPosition(world_x, world_y)) {
                obstacles_placed++;
            }
        }
        
        if (obstacles_placed < obstacle_count_) {
            ROS_WARN("Only placed %d obstacles out of %d requested. The map may be too crowded or obstacles are too large.", 
                     obstacles_placed, obstacle_count_);
        } else {
            ROS_INFO("Successfully placed %d random obstacles.", obstacles_placed);
        }
        
        // 更新并发布地图
        updateCostmap();
        publishCostmap();
        publishMarkers();
        
        return true;
    }
    
    // 新增：在指定位置添加障碍物
    bool addObstacleAtPosition(double world_x, double world_y) {
        // 检查点击是否在地图内
        if (!isPointInMap(world_x, world_y)) {
            ROS_WARN("Point (%.2f, %.2f) is out of map bounds.", world_x, world_y);
            return false;
        }
        
        // 将世界坐标转换为地图网格坐标
        int mx = static_cast<int>((world_x - original_map_.info.origin.position.x) / original_map_.info.resolution);
        int my = static_cast<int>((world_y - original_map_.info.origin.position.y) / original_map_.info.resolution);
        
        // 检查该点是否在自由空间
        int index = my * original_map_.info.width + mx;
        if (index >= 0 && index < static_cast<int>(original_map_.data.size())) {
            if (original_map_.data[index] != 0) { // 0 表示自由空间
                ROS_WARN("Point (%.2f, %.2f) is not in free space.", world_x, world_y);
                return false;
            }
        }
        
        // 如果不允许重叠，检查是否与现有障碍物重叠
        if (!allow_overlap_) {
            for (const auto& existing_obstacle : obstacle_centers_) {
                double dist = sqrt(pow(world_x - existing_obstacle.x, 2) + 
                                  pow(world_y - existing_obstacle.y, 2));
                if (dist < obstacle_radius_ * 2.0) {
                    ROS_WARN("Point (%.2f, %.2f) is too close to an existing obstacle.", 
                            world_x, world_y);
                    return false;
                }
            }
        }
        
        // 添加障碍物
        geometry_msgs::Point obstacle_center;
        obstacle_center.x = world_x;
        obstacle_center.y = world_y;
        obstacle_center.z = 0.0;
        obstacle_centers_.push_back(obstacle_center);
        
        ROS_INFO("Added obstacle at (%.2f, %.2f). Total obstacles: %zu", 
                 world_x, world_y, obstacle_centers_.size());
        
        return true;
    }
    
    // 新增：鼠标点击回调函数
    void clickCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        double world_x = msg->point.x;
        double world_y = msg->point.y;
        
        ROS_INFO("Mouse click at (%.2f, %.2f)", world_x, world_y);
        
        if (addObstacleAtPosition(world_x, world_y)) {
            // 更新并发布地图
            updateCostmap();
            publishCostmap();
            publishMarkers();
        }
    }
    
    // 服务回调：清除所有障碍物
    bool clearObstaclesCallback(std_srvs::Trigger::Request& req, 
                                std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        obstacle_centers_.clear();
        res.success = true;
        res.message = "All obstacles cleared.";
        
        ROS_INFO("All obstacles cleared.");
        
        updateCostmap();
        publishCostmap();
        publishMarkers();
        
        return true;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!enable_random_movement_ || obstacle_centers_.empty()) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 获取最新参数
        loadParameters();
        
        // 随机移动部分障碍物
        for (auto& center : obstacle_centers_) {
            if (std::rand() % 100 < 20) { // 20%的概率移动
                // 随机位移
                double dx = (std::rand() % 200 - 100) * 0.001; // -0.1 到 0.1
                double dy = (std::rand() % 200 - 100) * 0.001;
                
                // 检查新位置是否在地图内
                double new_x = center.x + dx;
                double new_y = center.y + dy;
                
                if (isPointInMap(new_x, new_y)) {
                    center.x = new_x;
                    center.y = new_y;
                }
            }
        }
        
        updateCostmap();
        publishCostmap();
        publishMarkers();
        
        ROS_DEBUG("Updated obstacle positions for random movement.");
    }

    bool isPointInMap(double world_x, double world_y) {
        double map_min_x = original_map_.info.origin.position.x;
        double map_max_x = map_min_x + original_map_.info.width * original_map_.info.resolution;
        double map_min_y = original_map_.info.origin.position.y;
        double map_max_y = map_min_y + original_map_.info.height * original_map_.info.resolution;
        
        return (world_x >= map_min_x && world_x < map_max_x && 
                world_y >= map_min_y && world_y < map_max_y);
    }

    // 核心函数：将 obstacle_centers_ 中的所有障碍物绘制到 obstacle_map_ 上
    void updateCostmap() {
        // 重置为原始地图
        obstacle_map_.data = original_map_.data;
        
        // 使用当前参数中的 obstacle_radius_
        int radius_cells = static_cast<int>(obstacle_radius_ / original_map_.info.resolution);
        int radius_sq = radius_cells * radius_cells;
        
        ROS_DEBUG("Updating costmap with %zu obstacles, radius: %.2f (cells: %d)", 
                 obstacle_centers_.size(), obstacle_radius_, radius_cells);
        
        for (const auto& center : obstacle_centers_) {
            // 将世界坐标转换为地图网格坐标
            int center_mx = static_cast<int>((center.x - original_map_.info.origin.position.x) / original_map_.info.resolution);
            int center_my = static_cast<int>((center.y - original_map_.info.origin.position.y) / original_map_.info.resolution);
            
            // 绘制圆形障碍物
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    if (dx*dx + dy*dy > radius_sq) continue; // 圆形判断
                    
                    int mx = center_mx + dx;
                    int my = center_my + dy;
                    
                    if (mx >= 0 && mx < static_cast<int>(original_map_.info.width) &&
                        my >= 0 && my < static_cast<int>(original_map_.info.height)) {
                        
                        int index = my * original_map_.info.width + mx;
                        // 只覆盖自由空间，保留未知区域
                        if (original_map_.data[index] == 0) {
                            obstacle_map_.data[index] = obstacle_cost_value_;
                        }
                    }
                }
            }
        }
        obstacle_map_.header.stamp = ros::Time::now();
    }

    void publishCostmap() {
        costmap_pub_.publish(obstacle_map_);
        ROS_DEBUG("Published obstacle map with %zu obstacles, radius=%.2f", 
                 obstacle_centers_.size(), obstacle_radius_);
    }

    void publishMarkers() {
        visualization_msgs::MarkerArray marker_array;
        
        ROS_DEBUG("Publishing markers for %zu obstacles with radius=%.2f", 
                 obstacle_centers_.size(), obstacle_radius_);
        
        for (size_t i = 0; i < obstacle_centers_.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "obstacles";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position = obstacle_centers_[i];
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            // 使用当前参数中的 obstacle_radius_ 设置标记尺寸
            double display_radius = obstacle_radius_;
            if (display_radius < 0.1) display_radius = 0.1;  // 最小显示半径
            
            marker.scale.x = display_radius * 2.0;  // 直径
            marker.scale.y = display_radius * 2.0;  // 直径
            marker.scale.z = 0.5; // 圆柱高度
            
            // 不同颜色的障碍物
            if (i % 3 == 0) {
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // 红色
            } else if (i % 3 == 1) {
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // 绿色
            } else {
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; // 蓝色
            }
            marker.color.a = 0.8; // 半透明
            
            marker.lifetime = ros::Duration(0);
            marker_array.markers.push_back(marker);
            
            // 添加文本标记显示障碍物编号
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "obstacles_text";
            text_marker.id = i;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position = obstacle_centers_[i];
            text_marker.pose.position.z = 0.6;  // 在圆柱上方显示
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.2;  // 文本大小
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = std::to_string(i);
            text_marker.lifetime = ros::Duration(0);
            marker_array.markers.push_back(text_marker);
        }
        
        // 如果障碍物列表被清空，需要发布一个删除所有标记的指令
        if (obstacle_centers_.empty()) {
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.ns = "obstacles";
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
            
            visualization_msgs::Marker delete_text_marker;
            delete_text_marker.header.frame_id = "map";
            delete_text_marker.header.stamp = ros::Time::now();
            delete_text_marker.ns = "obstacles_text";
            delete_text_marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(delete_text_marker);
            
            ROS_DEBUG("Published DELETEALL markers.");
        }
        
        markers_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_obstacle_publisher");
    RandomObstaclePublisher node;
    ros::spin();
    return 0;
}