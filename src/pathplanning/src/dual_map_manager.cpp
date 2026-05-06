#include "pathplanning/dual_map_manager.h"
#include <map_server/image_loader.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace pathplanning {

DualMapManager::DualMapManager() :
    current_mode_(AUTO_BUILD_MAP),
    map_ready_(false),
    auto_map_updated_(false),
    preload_map_loaded_(false),
    map_resolution_(0.05),
    map_origin_x_(-5.0),
    map_origin_y_(-5.0),
    map_width_(200),
    map_height_(200)
{
    // 初始化ROS发布者
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    
    // 订阅自主建图的地图
    auto_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
        "/auto_build_map", 1, &DualMapManager::autoMapCallback, this);
}

bool DualMapManager::initialize(const std::string& map_file_path) {
    ROS_INFO("Initializing DualMapManager");
    
    if (current_mode_ == PRELOAD_MAP && !map_file_path.empty()) {
        return switchMapMode(PRELOAD_MAP, map_file_path);
    }
    
    return true;
}

bool DualMapManager::switchMapMode(MapMode new_mode, const std::string& map_file_path) {
    ROS_INFO("Switching map mode from %d to %d", current_mode_, new_mode);
    
    if (new_mode == current_mode_) {
        ROS_INFO("Map mode unchanged.");
        return true;
    }
    
    if (new_mode == PRELOAD_MAP) {
        if (map_file_path.empty()) {
            ROS_ERROR("Cannot switch to preload mode without map file path!");
            return false;
        }
        if (!loadPredefinedMap(map_file_path)) {
            ROS_ERROR("Failed to load predefined map: %s", map_file_path.c_str());
            return false;
        }
        current_mode_ = PRELOAD_MAP;
        map_ready_ = preload_map_loaded_;
    } else {
        // AUTO_BUILD_MAP
        if (auto_map_updated_) {
            current_map_ = auto_build_map_;
            map_ready_ = true;
        } else {
            ROS_WARN("Auto-build map not yet available, map not ready.");
            map_ready_ = false;
        }
        current_mode_ = AUTO_BUILD_MAP;
    }
    
    // 发布当前地图
    if (map_ready_) {
        publishMap();
    }
    
    ROS_INFO("Map mode switched successfully. Map ready: %s", map_ready_ ? "true" : "false");
    return map_ready_;
}

bool DualMapManager::loadPredefinedMap(const std::string& map_file_path) {
    ROS_INFO("Loading predefined map from: %s", map_file_path.c_str());
    
    try {
        // 使用map_server的loadMapFromFile函数
        nav_msgs::GetMap::Response map_res;
        double origin[3] = {0.0, 0.0, 0.0};
        
        // 使用正确的函数调用
        map_server::loadMapFromFile(&map_res, map_file_path.c_str(), 0.05, false, 0.65, 0.196, origin);
        
        current_map_ = map_res.map;
        preload_map_ = map_res.map;
        preload_map_loaded_ = true;
        
        // 更新地图参数
        map_resolution_ = current_map_.info.resolution;
        map_origin_x_ = current_map_.info.origin.position.x;
        map_origin_y_ = current_map_.info.origin.position.y;
        map_width_ = current_map_.info.width;
        map_height_ = current_map_.info.height;
        map_ready_ = true;
        
        ROS_INFO("Predefined map loaded: %dx%d, res=%.3f, origin=(%.2f,%.2f)", 
                 map_width_, map_height_, map_resolution_, map_origin_x_, map_origin_y_);
        
        publishMap();
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while loading predefined map: %s", e.what());
        return false;
    }
}

void DualMapManager::updateAutoMap(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    auto_build_map_ = *grid;
    auto_map_updated_ = true;
    
    if (current_mode_ == AUTO_BUILD_MAP) {
        current_map_ = auto_build_map_;
        map_ready_ = true;
        publishMap();
    }
}

nav_msgs::OccupancyGrid DualMapManager::getCurrentMap() {
    return current_map_;
}

bool DualMapManager::isMapReady() const {
    return map_ready_;
}

DualMapManager::MapMode DualMapManager::getCurrentMode() const {
    return current_mode_;
}

void DualMapManager::getMapParameters(double& resolution, double& origin_x,
                                    double& origin_y, int& width, int& height) const {
    resolution = map_resolution_;
    origin_x = map_origin_x_;
    origin_y = map_origin_y_;
    width = map_width_;
    height = map_height_;
}

void DualMapManager::publishMap() {
    if (map_ready_) {
        current_map_.header.stamp = ros::Time::now();
        current_map_.header.frame_id = "map";
        map_pub_.publish(current_map_);
        ROS_DEBUG("Current map published");
    }
}

void DualMapManager::autoMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    updateAutoMap(grid);
}

} // namespace pathplanning