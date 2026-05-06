#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <cmath>

class InteractiveObstaclePublisher {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅与发布
    ros::Subscriber point_sub_;
    ros::Publisher costmap_pub_;
    ros::Publisher markers_pub_;
    ros::Subscriber clear_sub_;
    
    // 服务客户端：用于获取原始静态地图
    ros::ServiceClient static_map_client_;

    // 数据
    nav_msgs::OccupancyGrid original_map_;
    nav_msgs::OccupancyGrid dynamic_costmap_;
    std::vector<geometry_msgs::Point> obstacle_centers_;
    
    // 参数
    double obstacle_radius_;
    int obstacle_cost_value_;
    double update_rate_;
    bool enable_dynamic_movement_;
    bool map_received_;

    ros::Timer update_timer_;

public:
    InteractiveObstaclePublisher() : 
        private_nh_("~"),
        obstacle_radius_(0.5),
        obstacle_cost_value_(100),
        update_rate_(10.0),
        enable_dynamic_movement_(false),
        map_received_(false)
    {
        // 从参数服务器获取配置
        private_nh_.param("obstacle_radius", obstacle_radius_, obstacle_radius_);
        private_nh_.param("obstacle_cost_value", obstacle_cost_value_, obstacle_cost_value_);
        private_nh_.param("update_rate", update_rate_, update_rate_);
        private_nh_.param("enable_dynamic_movement", enable_dynamic_movement_, enable_dynamic_movement_);

        // 订阅Rviz的点击点
        point_sub_ = nh_.subscribe("/clicked_point", 10, &InteractiveObstaclePublisher::pointCallback, this);
        
        // 订阅清除障碍物指令
        clear_sub_ = nh_.subscribe("/clear_obstacles", 1, &InteractiveObstaclePublisher::clearCallback, this);
        
        // 发布包含障碍物的代价地图
        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/interactive_costmap", 1, true);
        
        // 发布障碍物的可视化标记
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/interactive_obstacles", 1, true);

        ROS_INFO("Interactive Obstacle Publisher Initializing...");
        ROS_INFO("Parameters: radius=%.2f, cost=%d, update_rate=%.1f", 
                 obstacle_radius_, obstacle_cost_value_, update_rate_);
        
        // 获取原始静态地图
        ROS_INFO("Waiting for static_map service...");
        ros::service::waitForService("/static_map");
        static_map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
        
        if (loadOriginalMap()) {
            ROS_INFO("Interactive Obstacle Publisher Ready.");
            ROS_INFO("Instructions: In Rviz, switch to the 'Publish Point' tool and click on the map to add obstacles.");
            ROS_INFO("              Each click places a circular obstacle of radius %.2f m.", obstacle_radius_);
            ROS_INFO("              Publish to /clear_obstacles to remove all obstacles.");
            
            // 初始化动态代价地图
            dynamic_costmap_ = original_map_;
            dynamic_costmap_.header.frame_id = "map";
            
            // 发布初始空地图
            publishCostmap();
            publishMarkers();
            
            // 启动定时器
            update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), 
                                            &InteractiveObstaclePublisher::timerCallback, this);
        } else {
            ROS_ERROR("Failed to load original map. Make sure map_server is running.");
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
            return true;
        } else {
            ROS_ERROR("Failed to call /static_map service");
            return false;
        }
    }

    void clearCallback(const std_msgs::Empty::ConstPtr& msg) {
        obstacle_centers_.clear();
        ROS_INFO("All obstacles cleared by command.");
        updateCostmap();
        publishCostmap();
        publishMarkers();
    }

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& point_msg) {
        if (!map_received_) {
            ROS_WARN("Cannot add obstacle: map not loaded yet.");
            return;
        }
        
        // 检查点是否在地图范围内
        double x = point_msg->point.x;
        double y = point_msg->point.y;
        
        double map_min_x = original_map_.info.origin.position.x;
        double map_max_x = map_min_x + original_map_.info.width * original_map_.info.resolution;
        double map_min_y = original_map_.info.origin.position.y;
        double map_max_y = map_min_y + original_map_.info.height * original_map_.info.resolution;
        
        if (x < map_min_x || x >= map_max_x || y < map_min_y || y >= map_max_y) {
            ROS_WARN("Clicked point (%.2f, %.2f) is outside map bounds.", x, y);
            return;
        }
        
        obstacle_centers_.push_back(point_msg->point);
        ROS_INFO("Added obstacle at world coordinate: (%.2f, %.2f)", x, y);
        
        updateCostmap();
        publishCostmap();
        publishMarkers();
    }

    void timerCallback(const ros::TimerEvent&) {
        if (enable_dynamic_movement_ && !obstacle_centers_.empty()) {
            // 简单示例：让第一个障碍物沿圆周运动
            static double angle = 0.0;
            angle += 0.1; // 角度增量
            
            // 计算圆周运动的新位置
            double radius = 0.5; // 圆周半径
            double center_x = obstacle_centers_[0].x;
            double center_y = obstacle_centers_[0].y;
            
            obstacle_centers_[0].x = center_x + radius * cos(angle);
            obstacle_centers_[0].y = center_y + radius * sin(angle);
            
            updateCostmap();
            publishCostmap();
            publishMarkers();
            
            ROS_DEBUG("Moved obstacle 0 to (%.2f, %.2f)", 
                     obstacle_centers_[0].x, obstacle_centers_[0].y);
        }
    }

    void updateCostmap() {
        // 重置为原始地图
        dynamic_costmap_.data = original_map_.data;
        
        int radius_cells = static_cast<int>(obstacle_radius_ / original_map_.info.resolution);
        
        for (const auto& center : obstacle_centers_) {
            // 将世界坐标转换为地图网格坐标
            int center_mx = static_cast<int>((center.x - original_map_.info.origin.position.x) / original_map_.info.resolution);
            int center_my = static_cast<int>((center.y - original_map_.info.origin.position.y) / original_map_.info.resolution);
            
            // 绘制圆形障碍物
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    if (dx*dx + dy*dy > radius_cells * radius_cells) continue; // 圆形判断
                    
                    int mx = center_mx + dx;
                    int my = center_my + dy;
                    
                    if (mx >= 0 && mx < static_cast<int>(original_map_.info.width) &&
                        my >= 0 && my < static_cast<int>(original_map_.info.height)) {
                        
                        int index = my * original_map_.info.width + mx;
                        // 只覆盖自由空间，保留未知区域
                        if (original_map_.data[index] == 0) {
                            dynamic_costmap_.data[index] = obstacle_cost_value_;
                        }
                    }
                }
            }
        }
        dynamic_costmap_.header.stamp = ros::Time::now();
    }

    void publishCostmap() {
        costmap_pub_.publish(dynamic_costmap_);
    }

    void publishMarkers() {
        visualization_msgs::MarkerArray marker_array;
        
        for (size_t i = 0; i < obstacle_centers_.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "interactive_obstacles";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position = obstacle_centers_[i];
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = obstacle_radius_ * 2.0;
            marker.scale.y = obstacle_radius_ * 2.0;
            marker.scale.z = 0.5; // 圆柱高度
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8; // 半透明
            
            marker.lifetime = ros::Duration(0);
            marker_array.markers.push_back(marker);
        }
        
        // 如果障碍物列表被清空，需要发布一个删除所有标记的指令
        if (obstacle_centers_.empty()) {
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.ns = "interactive_obstacles";
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
        }
        
        markers_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "interactive_obstacle_publisher");
    InteractiveObstaclePublisher node;
    ros::spin();
    return 0;
}