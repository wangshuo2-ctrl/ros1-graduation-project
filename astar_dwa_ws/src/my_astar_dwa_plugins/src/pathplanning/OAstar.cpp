#include "pathplanning/OAstar.h"
#include <cmath>
#include <limits>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <algorithm>
#include <memory>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <chrono>

namespace pathplanning {

// 构造函数
OAstar::OAstar():
    o_map_ready_(false),
    o_start_ready_(false),
    o_target_ready_(false),
    o_is_planning_(false),
    o_width_(0),
    o_height_(0),
    o_map_resolution_(0.0),
    o_map_origin_x_(0.0),
    o_map_origin_y_(0.0),
    o_last_planning_time_(ros::Time(0)),
    original_map_initialized_(false),
    ignore_dynamic_obstacles_(true),  // 默认忽略动态障碍物
    lock_original_map_(true),         // 默认锁定原始地图
    first_plan_completed_(false) {
    ROS_INFO("OAstar constructor called");
    ROS_INFO("OA* algorithm configured to ignore dynamic obstacles by default");
}

// 析构函数
OAstar::~OAstar() {
    ROS_INFO("OAstar destructor called");
}

// 清理开放集
void OAstar::clearOpenSet() {
    while (!o_open_set_.empty()) {
        o_open_set_.pop();
    }
}

// 清理节点
void OAstar::clearNodes() {
    o_all_nodes_.clear();
    o_close_set_.clear();
}

// 清理所有容器
void OAstar::clearContainers() {
    clearOpenSet();
    clearNodes();
}

// 初始化OAstar
void OAstar::initOAstar(const OAstarConfig& config) {
    ROS_INFO("OAstar::initOAstar called with configuration");
    o_config_ = config;
    
    // 设置曲率约束的默认值
    if (o_config_.min_turn_radius <= 0) {
        o_config_.min_turn_radius = 0.7;
    }
    
    if (o_config_.max_curvature <= 0) {
        o_config_.max_curvature = 1.5;
    }
    
    ROS_INFO("OAstar parameters configured:");
    ROS_INFO("  heuristic_weight: %.2f", o_config_.heuristic_weight);
    ROS_INFO("  inflate_radius: %d", o_config_.inflate_radius);
    ROS_INFO("  safety_margin: %.2f m", o_config_.safety_margin);
    ROS_INFO("  obstacle_clearance: %.2f m", o_config_.obstacle_clearance);
    ROS_INFO("  max_iterations: %d", o_config_.max_iterations);
    ROS_INFO("  ignore_dynamic_obstacles: %s", ignore_dynamic_obstacles_ ? "true" : "false");
    ROS_INFO("  lock_original_map: %s", lock_original_map_ ? "true" : "false");
    
    ROS_INFO("OAstar initialization completed successfully");
}

// 简化初始化
void OAstar::initOAstar(double heuristic_weight, int inflate_radius, double safety_margin) {
    OAstarConfig config;
    config.heuristic_weight = heuristic_weight;
    config.inflate_radius = inflate_radius;
    config.safety_margin = safety_margin;
    
    // 使用增强的安全参数
    config.min_turn_radius = 0.7;
    config.max_curvature = 1.5;
    config.enable_curvature_constraint = true;
    config.secondary_selection_max_iterations = 8;
    config.secondary_selection_search_radius = 0.3;
    config.inflate_radius = 8;
    config.obstacle_clearance = 0.5;
    config.safety_check_distance = 0.4;
    config.bspline_samples = 150;
    config.bspline_smoothness = 0.6;
    
    initOAstar(config);
}

// 设置配置
void OAstar::setConfig(const OAstarConfig& config) {
    o_config_ = config;
    ROS_INFO("OAstar config updated");
}

// 处理原始地图
void OAstar::processOriginalMap(const nav_msgs::OccupancyGrid& map) {
    if (!lock_original_map_) {
        return;
    }
    
    ROS_INFO("Processing and locking original map for OA*");
    original_map_ = map;
    original_map_initialized_ = true;
    
    // 处理地图
    occupancyGridToMat(map);
    processMap();
    o_map_ready_ = true;
    
    ROS_INFO("Original map locked and ready for planning");
}

// 地图更新函数
void OAstar::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    std::lock_guard<std::mutex> lock(o_map_mutex_);
    
    if (!grid) {
        ROS_ERROR("Received null map pointer");
        return;
    }
    
    if (grid->info.width == 0 || grid->info.height == 0) {
        ROS_WARN("Received empty map");
        return;
    }
    
    // 关键逻辑: 如果锁定原始地图，只处理第一次地图
    if (lock_original_map_ && original_map_initialized_) {
        ROS_INFO("Ignoring dynamic obstacle map update (original map locked)");
        return;
    }
    
    // 如果忽略动态障碍物，但已经有原始地图，也忽略更新
    if (ignore_dynamic_obstacles_ && original_map_initialized_) {
        ROS_INFO("Ignoring dynamic obstacle map (ignore_dynamic_obstacles=true)");
        return;
    }
    
    o_width_ = grid->info.width;
    o_height_ = grid->info.height;
    o_map_resolution_ = grid->info.resolution;
    o_map_origin_x_ = grid->info.origin.position.x;
    o_map_origin_y_ = grid->info.origin.position.y;
    
    ROS_INFO("OAstar updating map: %dx%d, resolution: %.3f, origin: (%.2f, %.2f)",
             o_width_, o_height_, o_map_resolution_, o_map_origin_x_, o_map_origin_y_);
    
    try {
        occupancyGridToMat(*grid);
        processMap();
        o_map_ready_ = true;
        
        // 如果是第一次处理地图，标记为原始地图
        if (!original_map_initialized_) {
            processOriginalMap(*grid);
        }
        
        ROS_INFO("Map updated and ready for planning");
    } catch (const std::exception& e) {
        ROS_ERROR("Error updating map: %s", e.what());
        o_map_ready_ = false;
    }
}

// 占用栅格转 OpenCV Mat
void OAstar::occupancyGridToMat(const nav_msgs::OccupancyGrid& grid) {
    if (grid.data.empty()) {
        ROS_WARN("Map data is empty");
        return;
    }
    
    o_costmap_ = cv::Mat(o_height_, o_width_, CV_8UC1);
    
    for (int y = 0; y < o_height_; ++y) {
        for (int x = 0; x < o_width_; ++x) {
            int index = y * o_width_ + x;
            
            if (index < 0 || index >= static_cast<int>(grid.data.size())) {
                ROS_WARN("Index out of bounds: %d, data size: %zu", index, grid.data.size());
                o_costmap_.at<uchar>(y, x) = O_UNKNOWN;
                continue;
            }
            
            int8_t value = grid.data[index];
            if (value < 0) {
                o_costmap_.at<uchar>(y, x) = O_UNKNOWN;
            } else if (value < o_config_.obstacle_threshold * 100) {
                o_costmap_.at<uchar>(y, x) = O_FREE;
            } else {
                o_costmap_.at<uchar>(y, x) = O_OBSTACLE;
            }
        }
    }
    
    ROS_INFO("Occupancy grid converted to OpenCV mat");
}

// 处理地图
void OAstar::processMap() {
    if (o_costmap_.empty()) {
        ROS_WARN("Cannot process map: costmap is empty");
        return;
    }
    
    try {
        // 创建膨胀地图
        o_inflated_map_ = o_costmap_.clone();
        
        if (o_config_.inflate_radius > 0) {
            int kernel_size = o_config_.inflate_radius * 2 + 1;
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(kernel_size, kernel_size)
            );
            cv::dilate(o_costmap_, o_inflated_map_, kernel);
        }
        
        ROS_INFO("Map processing complete, inflated radius: %d", o_config_.inflate_radius);
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV error processing map: %s", e.what());
        o_inflated_map_ = o_costmap_.clone();
    }
}

// 保存原始路径
void OAstar::saveOriginalPath(const std::vector<cv::Point>& path) {
    if (!first_plan_completed_ && !path.empty()) {
        first_plan_path_ = path;
        first_plan_completed_ = true;
        ROS_INFO("First plan path saved with %zu points", path.size());
    }
}

// 设置起点
void OAstar::setStartPoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Null start pointer received");
        return;
    }
    
    if (!o_map_ready_) {
        ROS_WARN("Cannot set start point: map not ready");
        return;
    }
    
    cv::Point map_point = worldToMap(msg->pose.pose.position.x, msg->pose.pose.position.y);
    setStartPoint(map_point);
}

// 设置目标点
void OAstar::setTargetPoint(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Null target pointer received");
        return;
    }
    
    if (!o_map_ready_) {
        ROS_WARN("Cannot set target point: map not ready");
        return;
    }
    
    cv::Point map_point = worldToMap(msg->pose.position.x, msg->pose.position.y);
    setTargetPoint(map_point);
}

// 设置起点
void OAstar::setStartPoint(cv::Point point) {
    if (!o_map_ready_) {
        ROS_WARN("Cannot set start point: map not ready");
        return;
    }
    
    if (isValidPoint(point)) {
        if (isPointSafeDetailed(point, o_config_.safety_margin)) {
            o_start_point_ = point;
            o_start_ready_ = true;
            ROS_INFO("OAstar start point set to: (%d, %d)", o_start_point_.x, o_start_point_.y);
        } else {
            ROS_WARN("Start point (%d, %d) is not safe, adjusting...", point.x, point.y);
            o_start_point_ = adjustToSafePoint(point, o_config_.safety_margin);
            
            if (isPointSafeDetailed(o_start_point_, o_config_.safety_margin)) {
                o_start_ready_ = true;
                ROS_INFO("OAstar adjusted start point to: (%d, %d)", o_start_point_.x, o_start_point_.y);
            } else {
                ROS_ERROR("Cannot find safe start point near (%d, %d)", point.x, point.y);
            }
        }
    }
}

// 设置目标点
void OAstar::setTargetPoint(cv::Point point) {
    if (!o_map_ready_) {
        ROS_WARN("Cannot set target point: map not ready");
        return;
    }
    
    if (isValidPoint(point)) {
        if (isPointSafeDetailed(point, o_config_.safety_margin)) {
            o_target_point_ = point;
            o_target_ready_ = true;
            ROS_INFO("OAstar target point set to: (%d, %d)", o_target_point_.x, o_target_point_.y);
        } else {
            ROS_WARN("Target point (%d, %d) is not safe, adjusting...", point.x, point.y);
            o_target_point_ = adjustToSafePoint(point, o_config_.safety_margin);
            
            if (isPointSafeDetailed(o_target_point_, o_config_.safety_margin)) {
                o_target_ready_ = true;
                ROS_INFO("OAstar adjusted target point to: (%d, %d)", o_target_point_.x, o_target_point_.y);
            } else {
                ROS_ERROR("Cannot find safe target point near (%d, %d)", point.x, point.y);
            }
        }
    }
}

// 路径规划
bool OAstar::pathPlanning(std::vector<cv::Point>& path) {
    if (!isReadyForPlanning()) {
        ROS_WARN("Not ready for planning. Map ready: %s, Start ready: %s, Target ready: %s",
                 o_map_ready_ ? "true" : "false",
                 o_start_ready_ ? "true" : "false",
                 o_target_ready_ ? "true" : "false");
        return false;
    }
    
    return pathPlanning(o_start_point_, o_target_point_, path);
}

// 带参数的路径规划
bool OAstar::pathPlanning(cv::Point start_point, cv::Point target_point, std::vector<cv::Point>& path) {
    if (o_is_planning_) {
        ROS_WARN("Path planning already in progress");
        return false;
    }
    
    ROS_INFO("OAstar starting path planning from (%d, %d) to (%d, %d)",
             start_point.x, start_point.y, target_point.x, target_point.y);
    
    // 如果已经有第一次规划的路径，并且忽略动态障碍物，则直接返回第一次的路径
    if (ignore_dynamic_obstacles_ && first_plan_completed_ && !first_plan_path_.empty()) {
        ROS_INFO("Using first plan path (ignoring dynamic obstacles)");
        path = first_plan_path_;
        return true;
    }
    
    // 重置规划器状态
    resetPlanner();
    o_is_planning_ = true;
    
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 执行A*算法
    std::shared_ptr<ONode> end_node = findPath(start_point, target_point);
    
    // 记录结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    path_metrics_.planning_time = std::chrono::duration<double>(end_time - start_time).count();
    
    if (end_node == nullptr) {
        ROS_ERROR("OA* path planning failed! No path found after %d iterations", path_metrics_.iterations);
        o_is_planning_ = false;
        analyzeFailureReason(start_point, target_point);
        return false;
    }
    
    // 提取原始路径
    extractPath(end_node, path);
    
    // 计算原始路径长度
    double original_length = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        original_length += calculateDistance(path[i], path[i + 1]);
    }
    
    // 检查原始路径安全性
    int unsafe_points = countUnsafePoints(path);
    if (unsafe_points > 0) {
        ROS_WARN("Raw path has %d unsafe points! Fixing...", unsafe_points);
    }
    
    ROS_INFO("Raw OA* path generated: %zu points, length: %.3f m, unsafe points: %d",
             path.size(), original_length, unsafe_points);
    
    // 应用路径平滑和优化
    if (path.size() > 2) {
        ROS_INFO("Applying path smoothing and optimization with enhanced safety...");
        
        // 第一步: 视线平滑，剔除冗余点
        if (o_config_.enable_path_smoothing) {
            smoothPath(path);
            ROS_INFO("After line-of-sight smoothing: %zu points", path.size());
        }
        
        // 检查平滑后路径安全性
        unsafe_points = countUnsafePoints(path);
        if (unsafe_points > 0) {
            ROS_WARN("Smoothed path has %d unsafe points! Adjusting...", unsafe_points);
            for (size_t i = 0; i < path.size(); i++) {
                if (!isPointSafeDetailed(path[i], o_config_.safety_margin)) {
                    path[i] = adjustToSafePoint(path[i], o_config_.safety_margin);
                    path_metrics_.unsafe_points_fixed++;
                }
            }
        }
        
        // 第二步: 简化，剔除共线点
        simplifyPath(path);
        ROS_INFO("After simplification: %zu points", path.size());
        
        // 第三步: 约束平滑
        if (o_config_.enable_constrained_smoothing && path.size() >= 3) {
            constrainedSmoothPath(path);
            ROS_INFO("After constrained smoothing: %zu points", path.size());
        }
        
        // 第四步: B样条平滑(包含曲率约束和二次选取策略)
        if (o_config_.enable_bspline_smoothing && path.size() >= 4) {
            try {
                std::vector<cv::Point> original_path = path;
                bsplineSmooth(path);
                
                // 检查B样条平滑后的安全性
                unsafe_points = countUnsafePoints(path);
                if (unsafe_points > 0) {
                    ROS_WARN("B-spline smoothed path has %d unsafe points! Adjusting...", unsafe_points);
                    for (size_t i = 0; i < path.size(); i++) {
                        if (!isPointSafeDetailed(path[i], o_config_.safety_margin)) {
                            path[i] = adjustToSafePoint(path[i], o_config_.safety_margin);
                            path_metrics_.unsafe_points_fixed++;
                        }
                    }
                }
                
                ROS_INFO("B-spline smoothing applied: %zu points -> %zu points",
                         original_path.size(), path.size());
            } catch (const std::exception& e) {
                ROS_WARN("B-spline smoothing failed: %s", e.what());
            }
        }
        
        // 第五步: 路径重采样
        if (o_config_.enable_path_resample && path.size() >= 2) {
            try {
                std::vector<cv::Point> resampled_path = resamplePath(path, o_config_.resample_points);
                
                // 检查重采样路径安全性
                if (isPathSafe(resampled_path, o_config_.safety_margin)) {
                    path = resampled_path;
                    ROS_INFO("Path resampled: %zu points -> %zu points", path.size(), resampled_path.size());
                } else {
                    ROS_WARN("Resampled path is not safe, using original");
                }
            } catch (const std::exception& e) {
                ROS_WARN("Path resampling failed: %s", e.what());
            }
        }
        
        // 最终安全检查
        if (o_config_.enable_safety_check) {
            bool final_safe = isPathSafe(path, o_config_.safety_check_distance);
            if (!final_safe) {
                ROS_WARN("Final path safety check failed, making final adjustments");
                for (size_t i = 0; i < path.size(); i++) {
                    if (!isPointSafeDetailed(path[i], o_config_.safety_check_distance)) {
                        cv::Point adjusted = adjustToSafePoint(path[i], o_config_.safety_check_distance);
                        if (adjusted != path[i]) {
                            path[i] = adjusted;
                            path_metrics_.unsafe_points_fixed++;
                        }
                    }
                }
            }
        }
    }
    
    // 计算最终路径长度
    path_metrics_.length = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        path_metrics_.length += calculateDistance(path[i], path[i + 1]);
    }
    
    ROS_INFO("Final OA* path: %zu points, length: %.3f m, unsafe points fixed: %d",
             path.size(), path_metrics_.length, path_metrics_.unsafe_points_fixed);
    
    // 最终路径点数
    path_metrics_.points_count = path.size();
    
    // 输出规划结果
    ROS_INFO("OA* path planning successful!");
    printMetricsTable();
    
    // 保存第一次规划的路径
    if (!first_plan_completed_) {
        saveOriginalPath(path);
    }
    
    // 清理内存
    clearContainers();
    o_is_planning_ = false;
    o_last_planning_time_ = ros::Time::now();
    
    return true;
}

// 查找路径的核心A*算法
std::shared_ptr<ONode> OAstar::findPath(const cv::Point& start, const cv::Point& target) {
    cv::Point adjusted_start = start;
    cv::Point adjusted_target = target;
    
    if (!o_map_ready_) {
        ROS_WARN("Cannot find path: map is not ready");
        return nullptr;
    }
    
    if (!isValidPoint(adjusted_start) || !isValidPoint(adjusted_target)) {
        ROS_WARN("Cannot find path: start or target point is invalid");
        ROS_WARN("Start: (%d, %d), Target: (%d, %d)",
                 adjusted_start.x, adjusted_start.y, adjusted_target.x, adjusted_target.y);
        return nullptr;
    }
    
    if (!isPointSafeDetailed(adjusted_start, o_config_.safety_margin)) {
        ROS_WARN("Start point (%d, %d) is not safe, adjusting...", adjusted_start.x, adjusted_start.y);
        adjusted_start = adjustToSafePoint(adjusted_start, o_config_.safety_margin);
        
        if (!isPointSafeDetailed(adjusted_start, o_config_.safety_margin)) {
            ROS_ERROR("Cannot find safe start point near (%d, %d)", adjusted_start.x, adjusted_start.y);
            return nullptr;
        }
    }
    
    if (!isPointSafeDetailed(adjusted_target, o_config_.safety_margin)) {
        ROS_WARN("Target point (%d, %d) is not safe, adjusting...", adjusted_target.x, adjusted_target.y);
        adjusted_target = adjustToSafePoint(adjusted_target, o_config_.safety_margin);
        
        if (!isPointSafeDetailed(adjusted_target, o_config_.safety_margin)) {
            ROS_ERROR("Cannot find safe target point near (%d, %d)", adjusted_target.x, adjusted_target.y);
            return nullptr;
        }
    }
    
    if (adjusted_start == adjusted_target) {
        ROS_WARN("Start and target points are the same");
        return std::make_shared<ONode>(adjusted_start);
    }
    
    ROS_INFO("Starting OA* search from (%d, %d) to (%d, %d)",
             adjusted_start.x, adjusted_start.y, adjusted_target.x, adjusted_target.y);
    
    // 重置统计
    path_metrics_.iterations = 0;
    path_metrics_.collision_checks = 0;
    path_metrics_.unsafe_points_fixed = 0;
    
    // 清除之前的节点
    clearContainers();
    
    // 创建起点节点
    auto start_node = std::make_shared<ONode>(adjusted_start);
    start_node->g_cost = 0;
    start_node->h_cost = calculateHeuristic(adjusted_start, adjusted_target);
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    
    // 添加到开放集
    o_open_set_.push(start_node);
    int start_index = pointToIndex(adjusted_start);
    o_all_nodes_[start_index] = start_node;
    
    // 主搜索循环
    while (!o_open_set_.empty() && path_metrics_.iterations < o_config_.max_iterations) {
        path_metrics_.iterations++;
        
        // 获取f值最小的节点
        auto current_node = o_open_set_.top();
        o_open_set_.pop();
        cv::Point current_point = current_node->point;
        int current_index = pointToIndex(current_point);
        
        // 检查是否已在关闭集
        if (o_close_set_.find(current_index) != o_close_set_.end()) {
            continue;
        }
        
        // 添加到关闭集
        o_close_set_.insert(current_index);
        
        // 检查是否到达目标
        if (current_point == adjusted_target) {
            ROS_INFO("Found path! Iterations: %d, Cost: %.2f, Collision checks: %d",
                     path_metrics_.iterations, current_node->g_cost, path_metrics_.collision_checks);
            return current_node;
        }
        
        // 获取有效邻居
        std::vector<cv::Point> neighbors = getValidNeighbors(current_point);
        
        for (const cv::Point& neighbor_point : neighbors) {
            int neighbor_index = pointToIndex(neighbor_point);
            
            // 检查是否已在关闭集
            if (o_close_set_.find(neighbor_index) != o_close_set_.end()) {
                continue;
            }
            
            // 计算移动成本
            double move_cost = 1.0;
            int dx = abs(neighbor_point.x - current_point.x);
            int dy = abs(neighbor_point.y - current_point.y);
            
            if (dx == 1 && dy == 1) {
                move_cost = 1.414;  // 对角线移动
                
                // 对角线移动额外检查相邻单元格
                if (!checkDiagonalSafety(current_point, neighbor_point)) {
                    continue;  // 跳过对角线移动
                }
            }
            
            // 计算转向惩罚
            double turn_penalty = calculateTurnPenalty(current_node, neighbor_point);
            double tentative_g = current_node->g_cost + move_cost + turn_penalty;
            
            // 检查节点是否已在开放集
            auto it = o_all_nodes_.find(neighbor_index);
            if (it != o_all_nodes_.end()) {
                auto neighbor_node = it->second;
                if (tentative_g < neighbor_node->g_cost) {
                    // 找到更优路径
                    neighbor_node->parent = current_node;
                    neighbor_node->g_cost = tentative_g;
                    neighbor_node->f_cost = neighbor_node->g_cost + neighbor_node->h_cost;
                    neighbor_node->direction = getDirection(current_point, neighbor_point);
                }
            } else {
                // 创建新节点
                auto neighbor_node = std::make_shared<ONode>(neighbor_point);
                neighbor_node->parent = current_node;
                neighbor_node->g_cost = tentative_g;
                neighbor_node->h_cost = calculateHeuristic(neighbor_point, adjusted_target);
                neighbor_node->f_cost = neighbor_node->g_cost + neighbor_node->h_cost;
                neighbor_node->direction = getDirection(current_point, neighbor_point);
                o_all_nodes_[neighbor_index] = neighbor_node;
                o_open_set_.push(neighbor_node);
            }
        }
    }
    
    ROS_WARN("OA* search failed: maximum iterations (%d) reached or no path found", o_config_.max_iterations);
    return nullptr;
}

// 提取路径
void OAstar::extractPath(const std::shared_ptr<ONode>& end_node, std::vector<cv::Point>& path) {
    path.clear();
    
    if (!end_node) {
        ROS_WARN("Cannot extract path: end node is null");
        return;
    }
    
    // 从终点回溯到起点
    std::shared_ptr<ONode> current = end_node;
    std::vector<cv::Point> reverse_path;
    
    while (current != nullptr) {
        reverse_path.push_back(current->point);
        current = current->parent;
    }
    
    // 反转路径
    path.resize(reverse_path.size());
    std::reverse_copy(reverse_path.begin(), reverse_path.end(), path.begin());
    
    ROS_INFO("OA* path extracted: %zu points", path.size());
    path_metrics_.points_count = path.size();
}

// 平滑路径
void OAstar::smoothPath(std::vector<cv::Point>& path) {
    if (path.size() < 3) {
        return;
    }
    
    std::vector<cv::Point> smoothed_path;
    smoothed_path.push_back(path[0]);
    
    for (size_t i = 1; i < path.size() - 1; i++) {
        // 检查当前点是否可以跳过
        if (i + 1 < path.size() - 1) {
            if (hasLineOfSight(smoothed_path.back(), path[i + 1])) {
                // 可以跳过当前点
                continue;
            }
        }
        smoothed_path.push_back(path[i]);
    }
    
    smoothed_path.push_back(path.back());
    
    if (smoothed_path.size() < path.size()) {
        ROS_INFO("OA* path smoothed: %zu points -> %zu points", path.size(), smoothed_path.size());
        path = smoothed_path;
    }
}

// 简化路径
void OAstar::simplifyPath(std::vector<cv::Point>& path) {
    if (path.size() < 3) {
        return;
    }
    
    std::vector<cv::Point> simplified;
    simplified.push_back(path[0]);
    
    for (size_t i = 1; i < path.size() - 1; i++) {
        cv::Point prev = simplified.back();
        cv::Point curr = path[i];
        cv::Point next = path[i + 1];
        
        // 检查三点是否共线
        int cross = (curr.x - prev.x) * (next.y - prev.y) - (curr.y - prev.y) * (next.x - prev.x);
        if (cross != 0) {  // 不共线
            simplified.push_back(curr);
        }
    }
    
    simplified.push_back(path.back());
    path = simplified;
}

// 约束平滑路径
void OAstar::constrainedSmoothPath(std::vector<cv::Point>& path) {
    if (path.size() < 3) {
        return;
    }
    
    ROS_INFO("Applying constrained smoothing with %d iterations, weight: %.2f",
             o_config_.constrained_smoothing_iterations, o_config_.constrained_smoothing_weight);
    
    std::vector<cv::Point> smoothed_path = path;
    
    for (int iter = 0; iter < o_config_.constrained_smoothing_iterations; ++iter) {
        std::vector<cv::Point> new_path = smoothed_path;
        
        // 跳过起点和终点
        for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
            // 计算当前位置
            double x = smoothed_path[i].x;
            double y = smoothed_path[i].y;
            
            // 计算梯度(朝向相邻点的中心)
            double prev_x = smoothed_path[i - 1].x;
            double prev_y = smoothed_path[i - 1].y;
            double next_x = smoothed_path[i + 1].x;
            double next_y = smoothed_path[i + 1].y;
            
            double gradient_x = (prev_x + next_x) * 0.5 - x;
            double gradient_y = (prev_y + next_y) * 0.5 - y;
            
            // 应用约束梯度下降
            double new_x = x + gradient_x * o_config_.constrained_smoothing_weight;
            double new_y = y + gradient_y * o_config_.constrained_smoothing_weight;
            
            cv::Point new_point(static_cast<int>(new_x + 0.5), static_cast<int>(new_y + 0.5));
            
            // 安全检查: 确保新点不进入障碍物
            if (isPointSafeDetailed(new_point, o_config_.safety_margin)) {
                new_path[i] = new_point;
            } else {
                // 如果新点不安全，尝试在梯度方向上找到最近的安全点
                cv::Point safe_point = adjustToSafePoint(new_point, o_config_.safety_margin);
                if (safe_point != new_point) {
                    new_path[i] = safe_point;
                    path_metrics_.unsafe_points_fixed++;
                }
            }
        }
        
        smoothed_path = new_path;
    }
    
    // 检查平滑后的路径安全性
    for (size_t i = 0; i < smoothed_path.size(); i++) {
        if (!isPointSafeDetailed(smoothed_path[i], o_config_.safety_margin)) {
            smoothed_path[i] = adjustToSafePoint(smoothed_path[i], o_config_.safety_margin);
        }
    }
    
    path = smoothed_path;
    
    ROS_INFO("Constrained smoothing applied, %d iterations, fixed %d unsafe points",
             o_config_.constrained_smoothing_iterations, path_metrics_.unsafe_points_fixed);
}

// B样条平滑
void OAstar::bsplineSmooth(std::vector<cv::Point>& path) {
    if (path.size() < 4) {
        ROS_WARN("Not enough points for B-spline smoothing (need at least 4 points)");
        return;
    }
    
    ROS_INFO("Applying B-spline smoothing to path with %zu points", path.size());
    ROS_INFO("B-spline parameters: degree=%d, samples=%d, smoothness=%.2f",
             o_config_.bspline_degree, o_config_.bspline_samples, o_config_.bspline_smoothness);
    
    // 保存原始路径用于二次选取
    std::vector<cv::Point> original_path = path;
    
    // 最大迭代次数，防止无限循环
    int max_iterations = o_config_.secondary_selection_max_iterations;
    int iteration = 0;
    bool smoothing_success = false;
    
    // 获取最大允许曲率
    double max_curvature = o_config_.max_curvature;
    
    // 计算增强的安全距离
    double enhanced_clearance = o_config_.safety_margin * 1.5;  // 增加50%的安全距离
    ROS_INFO("Using enhanced clearance for B-spline: %.2f m (original: %.2f m)",
             enhanced_clearance, o_config_.safety_margin);
    
    while (iteration < max_iterations && !smoothing_success) {
        iteration++;
        ROS_INFO("B-spline smoothing iteration %d/%d", iteration, max_iterations);
        
        // 使用当前路径作为控制点
        std::vector<cv::Point> control_points = path;
        
        // 计算B样条曲线
        std::vector<std::pair<double, double>> spline_points = 
            computeBSpline(control_points, o_config_.bspline_degree, o_config_.bspline_samples);
        
        // 将平滑后的点转换回整数坐标
        std::vector<cv::Point> smoothed_path;
        smoothed_path.reserve(spline_points.size());
        
        for (const auto& point : spline_points) {
            int x = static_cast<int>(point.first + 0.5);
            int y = static_cast<int>(point.second + 0.5);
            
            // 检查点是否在地图范围内
            if (x < 0) x = 0;
            if (x >= o_width_) x = o_width_ - 1;
            if (y < 0) y = 0;
            if (y >= o_height_) y = o_height_ - 1;
            
            smoothed_path.push_back(cv::Point(x, y));
        }
        
        // 移除重复点
        smoothed_path.erase(std::unique(smoothed_path.begin(), smoothed_path.end()), smoothed_path.end());
        
        if (smoothed_path.size() < 2) {
            ROS_WARN("B-spline smoothing produced too few points");
            break;
        }
        
        // 检查1: 使用增强安全距离进行路径碰撞检查
        bool collision_free = true;
        int collision_segment = -1;
        
        for (size_t i = 0; i < smoothed_path.size() - 1; i++) {
            if (!checkPathSegmentSafety(smoothed_path[i], smoothed_path[i + 1], enhanced_clearance)) {
                ROS_INFO("B-spline segment %zu has collision or is too close to obstacle (distance < %.2f m)",
                         i, enhanced_clearance);
                collision_free = false;
                collision_segment = static_cast<int>(i);
                break;
            }
        }
        
        // 检查2: 曲率约束检查
        bool curvature_ok = true;
        if (collision_free && o_config_.enable_curvature_constraint) {
            curvature_ok = checkCurvatureConstraint(smoothed_path, max_curvature);
        }
        
        // 如果通过所有检查，使用平滑后的路径
        if (collision_free && curvature_ok) {
            // 确保起始点和目标点不变
            if (!smoothed_path.empty()) {
                smoothed_path[0] = original_path[0];
                smoothed_path.back() = original_path.back();
            }
            
            // 最终安全性检查
            bool final_safe = true;
            for (size_t i = 0; i < smoothed_path.size(); i++) {
                if (!isPointSafeDetailed(smoothed_path[i], o_config_.safety_margin)) {
                    ROS_WARN("Smoothed path point %zu is unsafe, adjusting...", i);
                    cv::Point safe_point = adjustToSafePoint(smoothed_path[i], o_config_.safety_margin);
                    if (safe_point != smoothed_path[i]) {
                        smoothed_path[i] = safe_point;
                        path_metrics_.unsafe_points_fixed++;
                    } else {
                        final_safe = false;
                        break;
                    }
                }
            }
            
            if (final_safe) {
                path = smoothed_path;
                smoothing_success = true;
                ROS_INFO("B-spline smoothing applied successfully: %zu points -> %zu points (smooth)",
                         original_path.size(), path.size());
                ROS_INFO("Enhanced safety distance maintained: %.2f m", enhanced_clearance);
                break;
            }
        }
        
        // 如果检查失败，应用二次选取策略
        ROS_INFO("B-spline smoothing failed checks in iteration %d", iteration);
        
        if (!collision_free && collision_segment >= 0) {
            // 碰撞检测失败，应用二次选取策略
            ROS_INFO("Applying secondary node selection strategy for collision at segment %d", collision_segment);
            
            // 计算碰撞点对应的控制点索引
            int control_point_index = static_cast<int>((double)collision_segment * control_points.size() / smoothed_path.size());
            control_point_index = std::max(0, std::min(control_point_index, static_cast<int>(control_points.size()) - 1));
            
            // 在碰撞点附近调整控制点
            std::vector<cv::Point> new_control_points = 
                adjustControlPointsForCollision(control_points, control_point_index);
            
            // 如果调整了控制点，用新控制点重新尝试
            if (new_control_points != control_points) {
                path = new_control_points;
                continue;
            }
        }
        
        if (!curvature_ok) {
            // 曲率约束失败，尝试简化路径以减少曲率
            ROS_INFO("Curvature constraint violated, simplifying path to reduce curvature");
            
            if (path.size() > 4) {
                // 简化路径: 只保留关键点
                std::vector<cv::Point> simplified_path;
                simplified_path.push_back(path[0]);
                
                // 保留1/4、1/2、3/4位置的点
                size_t quarter = path.size() / 4;
                size_t half = path.size() / 2;
                size_t three_quarters = 3 * path.size() / 4;
                
                if (quarter < path.size()) simplified_path.push_back(path[quarter]);
                if (half < path.size()) simplified_path.push_back(path[half]);
                if (three_quarters < path.size()) simplified_path.push_back(path[three_quarters]);
                simplified_path.push_back(path.back());
                
                path = simplified_path;
                ROS_INFO("Path simplified from %zu to %zu points to reduce curvature",
                         control_points.size(), path.size());
                continue;
            }
        }
    }
    
    // 如果所有策略都失败，尝试回退到原始路径
    if (iteration == max_iterations) {
        ROS_WARN("All smoothing strategies failed after %d iterations", iteration);
        
        // 尝试使用原始路径进行约束平滑
        std::vector<cv::Point> temp_path = original_path;
        constrainedSmoothPath(temp_path);
        
        if (isPathSafe(temp_path, o_config_.safety_margin)) {
            path = temp_path;
            smoothing_success = true;
            ROS_INFO("Using constrained smoothing as fallback: %zu points", path.size());
        } else {
            // 最后的回退: 使用原始A*路径
            path = original_path;
            ROS_WARN("Using original OA* path due to smoothing failure");
        }
    }
    
    if (!smoothing_success) {
        ROS_WARN("B-spline smoothing failed, using constrained smoothing instead");
        constrainedSmoothPath(path);
    }
}

// 检查曲率约束
bool OAstar::checkCurvatureConstraint(const std::vector<cv::Point>& path, double max_curvature) {
    if (path.size() < 3) {
        return true;  // 少于3个点无法计算曲率
    }
    
    // 将地图坐标转换为世界坐标
    auto toWorld = [&](const cv::Point& p) -> std::pair<double, double> {
        double world_x = o_map_origin_x_ + (p.x + 0.5) * o_map_resolution_;
        double world_y = o_map_origin_y_ + (p.y + 0.5) * o_map_resolution_;
        return std::make_pair(world_x, world_y);
    };
    
    // 计算每个中间点的曲率
    for (size_t i = 1; i < path.size() - 1; i++) {
        const cv::Point& p0 = path[i - 1];
        const cv::Point& p1 = path[i];
        const cv::Point& p2 = path[i + 1];
        
        // 转换为世界坐标
        auto w0 = toWorld(p0);
        auto w1 = toWorld(p1);
        auto w2 = toWorld(p2);
        
        // 计算向量
        double dx1 = w1.first - w0.first;
        double dy1 = w1.second - w0.second;
        double dx2 = w2.first - w1.first;
        double dy2 = w2.second - w1.second;
        
        // 计算向量长度
        double l1 = sqrt(dx1 * dx1 + dy1 * dy1);
        double l2 = sqrt(dx2 * dx2 + dy2 * dy2);
        
        if (l1 < 1e-6 || l2 < 1e-6) {
            continue;  // 点太接近，跳过
        }
        
        // 计算向量夹角
        double dot = dx1 * dx2 + dy1 * dy2;
        double cross = dx1 * dy2 - dy1 * dx2;
        double angle = atan2(fabs(cross), dot);
        
        // 计算近似曲率(假设在小角度下，曲率≈角度变化/弧长)
        double avg_length = (l1 + l2) / 2.0;
        double curvature = 2.0 * fabs(sin(angle / 2.0)) / avg_length;
        
        if (curvature > max_curvature) {
            ROS_INFO("Curvature constraint violated at point %zu: curvature = %.4f > max_curvature = %.4f",
                     i, curvature, max_curvature);
            ROS_INFO("World points: (%.2f, %.2f) -> (%.2f, %.2f) -> (%.2f, %.2f)",
                     w0.first, w0.second, w1.first, w1.second, w2.first, w2.second);
            return false;
        }
    }
    
    return true;
}

// 调整控制点以避免碰撞
std::vector<cv::Point> OAstar::adjustControlPointsForCollision(
    const std::vector<cv::Point>& control_points, int collision_index) {
    
    std::vector<cv::Point> adjusted_points = control_points;
    
    if (collision_index < 0 || collision_index >= static_cast<int>(control_points.size())) {
        return adjusted_points;
    }
    
    // 将搜索半径转换为地图单位
    int search_radius_cells = static_cast<int>(o_config_.secondary_selection_search_radius / o_map_resolution_);
    if (search_radius_cells < 1) search_radius_cells = 1;
    
    ROS_INFO("Adjusting control points for collision at index %d, search radius: %d cells",
             collision_index, search_radius_cells);
    
    // 策略1: 在碰撞点附近调整控制点
    int start_idx = std::max(0, collision_index - 1);
    int end_idx = std::min(static_cast<int>(control_points.size()) - 1, collision_index + 1);
    bool points_adjusted = false;
    
    for (int i = start_idx; i <= end_idx; i++) {
        cv::Point original_point = adjusted_points[i];
        
        // 如果当前点不安全，尝试调整
        if (!isPointSafeDetailed(adjusted_points[i], o_config_.safety_margin)) {
            // 在多个方向上寻找安全点
            cv::Point best_point = original_point;
            double best_score = -std::numeric_limits<double>::max();
            
            // 搜索方向: 8个主要方向
            int dx_values[] = {-1, 0, 1, 0, -1, 1, 1, -1};
            int dy_values[] = {0, 1, 0, -1, 1, 1, -1, -1};
            
            for (int dir = 0; dir < 8; dir++) {
                for (int r = 1; r <= search_radius_cells; r++) {
                    int dx = dx_values[dir] * r;
                    int dy = dy_values[dir] * r;
                    cv::Point test_point(original_point.x + dx, original_point.y + dy);
                    
                    if (!isValidPoint(test_point)) {
                        continue;
                    }
                    
                    // 计算得分: 安全距离+保持原始位置接近性
                    double safety_score = calculateSafetyScore(test_point);
                    double proximity_penalty = sqrt(dx * dx + dy * dy) * o_map_resolution_ * 0.5;  // 惩罚远离原始位置
                    double total_score = safety_score - proximity_penalty;
                    
                    if (total_score > best_score) {
                        best_score = total_score;
                        best_point = test_point;
                    }
                }
            }
            
            // 如果找到了更好的点，更新
            if (best_point != original_point) {
                adjusted_points[i] = best_point;
                points_adjusted = true;
                ROS_INFO("  Adjusted control point %d from (%d, %d) to (%d, %d)",
                         i, original_point.x, original_point.y, best_point.x, best_point.y);
            }
        }
    }
    
    // 策略2: 在碰撞区域插入额外的控制点(如果碰撞发生在路径中间)
    if (!points_adjusted && collision_index > 0 && collision_index < static_cast<int>(adjusted_points.size()) - 1) {
        cv::Point before = adjusted_points[collision_index - 1];
        cv::Point after = adjusted_points[collision_index + 1];
        
        // 计算中点
        cv::Point mid_point(
            (before.x + after.x) / 2,
            (before.y + after.y) / 2
        );
        
        // 将中点调整到安全位置
        cv::Point safe_mid_point = adjustToSafePoint(mid_point, o_config_.safety_margin);
        if (safe_mid_point != mid_point) {
            // 插入额外的控制点
            adjusted_points.insert(adjusted_points.begin() + collision_index, safe_mid_point);
            ROS_INFO("  Inserted additional control point at collision area");
            points_adjusted = true;
        }
    }
    
    if (!points_adjusted) {
        ROS_INFO("  Could not find better control points, using original");
    }
    
    return adjusted_points;
}

// 计算安全评分
double OAstar::calculateSafetyScore(const cv::Point& point) {
    if (!isValidPoint(point)) {
        return -std::numeric_limits<double>::max();
    }
    
    // 计算到最近障碍物的距离
    double min_distance = std::numeric_limits<double>::max();
    int search_radius = static_cast<int>(o_config_.safety_margin * 2 / o_map_resolution_);
    
    for (int dy = -search_radius; dy <= search_radius; dy++) {
        for (int dx = -search_radius; dx <= search_radius; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            cv::Point test_point(point.x + dx, point.y + dy);
            
            if (isValidPoint(test_point)) {
                uchar value = o_inflated_map_.at<uchar>(test_point.y, test_point.x);
                if (value == O_OBSTACLE) {
                    double distance = sqrt(dx * dx + dy * dy) * o_map_resolution_;
                    if (distance < min_distance) {
                        min_distance = distance;
                    }
                }
            }
        }
    }
    
    // 如果没有找到障碍物，返回最大搜索距离
    if (min_distance == std::numeric_limits<double>::max()) {
        min_distance = search_radius * o_map_resolution_;
    }
    
    // 评分: 距离越远，分数越高
    return min_distance;
}

// 计算B样条曲线
std::vector<std::pair<double, double>> OAstar::computeBSpline(
    const std::vector<cv::Point>& control_points, int degree, int samples) {
    
    int n = control_points.size() - 1;  // 控制点索引最大值
    int p = std::min(degree, n);  // 阶数，不能大于控制点数
    
    if (n < p) {
        ROS_WARN("B-spline: Not enough control points for degree %d", degree);
        std::vector<std::pair<double, double>> result;
        for (const auto& point : control_points) {
            result.push_back(std::make_pair(point.x, point.y));
        }
        return result;
    }
    
    // 计算节点向量
    std::vector<double> knots = computeKnotVector(n, p);
    std::vector<std::pair<double, double>> result;
    result.reserve(samples);
    
    // 采样曲线
    double t_start = knots[p];
    double t_end = knots[n + 1];
    double step = (t_end - t_start) / (samples - 1);
    
    for (int i = 0; i < samples; ++i) {
        double t = t_start + i * step;
        
        // 计算B样条曲线上的点
        double x = 0.0, y = 0.0;
        for (int j = 0; j <= n; ++j) {
            double basis = bsplineBasis(j, p, t, knots);
            x += control_points[j].x * basis;
            y += control_points[j].y * basis;
        }
        
        result.push_back(std::make_pair(x, y));
    }
    
    return result;
}

// 计算节点向量
std::vector<double> OAstar::computeKnotVector(int n, int p) {
    std::vector<double> knots;
    int m = n + p + 1;
    knots.resize(m + 1);
    
    // 均匀节点向量
    for (int i = 0; i <= m; ++i) {
        knots[i] = static_cast<double>(i) / m;
    }
    
    // 使曲线通过第一个和最后一个控制点
    for (int i = 0; i <= p; ++i) {
        knots[i] = 0.0;
        knots[m - i] = 1.0;
    }
    
    return knots;
}

// 计算B样条基函数
double OAstar::bsplineBasis(int i, int p, double t, const std::vector<double>& knots) {
    if (p == 0) {
        if (t >= knots[i] && t < knots[i + 1]) {
            return 1.0;
        } else {
            return 0.0;
        }
    }
    
    double denom1 = knots[i + p] - knots[i];
    double denom2 = knots[i + p + 1] - knots[i + 1];
    double term1 = 0.0;
    double term2 = 0.0;
    
    if (denom1 != 0.0) {
        term1 = ((t - knots[i]) / denom1) * bsplineBasis(i, p - 1, t, knots);
    }
    
    if (denom2 != 0.0) {
        term2 = ((knots[i + p + 1] - t) / denom2) * bsplineBasis(i + 1, p - 1, t, knots);
    }
    
    return term1 + term2;
}

// 重采样路径
std::vector<cv::Point> OAstar::resamplePath(const std::vector<cv::Point>& path, int num_points) {
    if (path.size() < 2) {
        return path;
    }
    
    // 计算路径总长度
    double total_length = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        total_length += calculateDistance(path[i], path[i + 1]);
    }
    
    // 计算重采样步长
    double step_length = total_length / (num_points - 1);
    std::vector<cv::Point> resampled_path;
    resampled_path.reserve(num_points);
    resampled_path.push_back(path[0]);
    
    double current_length = 0.0;
    int current_segment = 0;
    
    for (int i = 1; i < num_points - 1; i++) {
        double target_length = i * step_length;
        
        // 找到目标点所在的线段
        while (current_segment < static_cast<int>(path.size()) - 1) {
            double segment_length = calculateDistance(path[current_segment], path[current_segment + 1]);
            
            if (current_length + segment_length > target_length) {
                // 计算在线段上的位置
                double ratio = (target_length - current_length) / segment_length;
                double x = path[current_segment].x + ratio * (path[current_segment + 1].x - path[current_segment].x);
                double y = path[current_segment].y + ratio * (path[current_segment + 1].y - path[current_segment].y);
                
                cv::Point point(static_cast<int>(x + 0.5), static_cast<int>(y + 0.5));
                
                // 检查边界
                if (point.x < 0) point.x = 0;
                if (point.x >= o_width_) point.x = o_width_ - 1;
                if (point.y < 0) point.y = 0;
                if (point.y >= o_height_) point.y = o_height_ - 1;
                
                resampled_path.push_back(point);
                break;
            }
            
            current_length += segment_length;
            current_segment++;
        }
    }
    
    // 添加最后一个点
    resampled_path.push_back(path.back());
    ROS_INFO("Path resampled: %zu points -> %zu points", path.size(), resampled_path.size());
    
    return resampled_path;
}

// 计算启发式函数
double OAstar::calculateHeuristic(const cv::Point& a, const cv::Point& b) const {
    if (o_config_.euclidean) {
        // 欧几里得距离
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy) * o_config_.heuristic_weight;
    } else {
        // 曼哈顿距离
        double dx = std::abs(a.x - b.x);
        double dy = std::abs(a.y - b.y);
        return (dx + dy) * o_config_.heuristic_weight;
    }
}

// 计算转向惩罚
double OAstar::calculateTurnPenalty(const std::shared_ptr<ONode>& current, const cv::Point& next) const {
    if (!current || !current->parent) {
        return 0.0;
    }
    
    int prev_dir = getDirection(current->parent->point, current->point);
    int curr_dir = getDirection(current->point, next);
    
    if (prev_dir == curr_dir) {
        return 0.0;
    }
    
    return o_config_.turn_penalty_weight;
}

// 计算距离
double OAstar::calculateDistance(const cv::Point& p1, const cv::Point& p2) const {
    double dx = static_cast<double>(p2.x - p1.x);
    double dy = static_cast<double>(p2.y - p1.y);
    return sqrt(dx * dx + dy * dy) * o_map_resolution_;
}

// 获取有效邻居
std::vector<cv::Point> OAstar::getValidNeighbors(const cv::Point& current) const {
    std::vector<cv::Point> neighbors = getAllNeighbors(current);
    std::vector<cv::Point> valid_neighbors;
    
    for (const cv::Point& neighbor : neighbors) {
        if (isValidPoint(neighbor) && isPointSafeDetailed(neighbor, o_config_.safety_margin)) {
            // 额外检查: 对于对角线移动，确保相邻的两个单元格也是自由的
            int dx = neighbor.x - current.x;
            int dy = neighbor.y - current.y;
            
            if (dx != 0 && dy != 0) {  // 对角线移动
                cv::Point side1(current.x + dx, current.y);
                cv::Point side2(current.x, current.y + dy);
                
                if (!isPointSafeDetailed(side1, o_config_.safety_margin) ||
                    !isPointSafeDetailed(side2, o_config_.safety_margin)) {
                    continue;  // 跳过对角线移动，如果角落有障碍物
                }
            }
            
            valid_neighbors.push_back(neighbor);
        }
    }
    
    return valid_neighbors;
}

// 获取所有邻居
std::vector<cv::Point> OAstar::getAllNeighbors(const cv::Point& current) const {
    std::vector<cv::Point> neighbors;
    
    // 8方向移动
    int dx[8] = {-1, 0, 1, 0, -1, 1, 1, -1};
    int dy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    
    for (int i = 0; i < 8; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        neighbors.push_back(cv::Point(nx, ny));
    }
    
    return neighbors;
}

// 检查视线
bool OAstar::hasLineOfSight(const cv::Point& a, const cv::Point& b) const {
    if (!isValidPoint(a) || !isValidPoint(b)) {
        return false;
    }
    
    // Bresenham直线算法
    int x0 = a.x, y0 = a.y;
    int x1 = b.x, y1 = b.y;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (x0 != x1 || y0 != y1) {
        if (!isPointSafeDetailed(cv::Point(x0, y0), o_config_.safety_margin)) {
            return false;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return isPointSafeDetailed(cv::Point(x1, y1), o_config_.safety_margin);
}

// 检查对角线安全性
bool OAstar::checkDiagonalSafety(const cv::Point& from, const cv::Point& to) const {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    if (dx == 0 || dy == 0) {
        return true;  // 不是对角线移动
    }
    
    // 检查两个相邻单元格
    cv::Point cell1(from.x + dx, from.y);
    cv::Point cell2(from.x, from.y + dy);
    
    return isPointSafeDetailed(cell1, o_config_.safety_margin) &&
           isPointSafeDetailed(cell2, o_config_.safety_margin);
}

// 检查碰撞
bool OAstar::checkCollision(const cv::Point& pt1, const cv::Point& pt2) const {
    return !checkPathSegmentSafety(pt1, pt2, 0.0);
}

// 检查路径段安全性
bool OAstar::checkPathSegmentSafety(const cv::Point& pt1, const cv::Point& pt2, double clearance) const {
    if (!isPointSafeDetailed(pt1, clearance) || !isPointSafeDetailed(pt2, clearance)) {
        return false;
    }
    
    // Bresenham直线算法检查碰撞
    int x0 = pt1.x, y0 = pt1.y;
    int x1 = pt2.x, y1 = pt2.y;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (x0 != x1 || y0 != y1) {
        if (!isPointSafeDetailed(cv::Point(x0, y0), clearance)) {
            return false;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return true;
}

// 获取方向
int OAstar::getDirection(const cv::Point& from, const cv::Point& to) const {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    if (dx == 0 && dy == 0) return 0;       // 未知方向
    if (dx == -1 && dy == 0) return 1;      // 左
    if (dx == -1 && dy == 1) return 2;      // 左上
    if (dx == 0 && dy == 1) return 3;       // 上
    if (dx == 1 && dy == 1) return 4;       // 右上
    if (dx == 1 && dy == 0) return 5;       // 右
    if (dx == 1 && dy == -1) return 6;      // 右下
    if (dx == 0 && dy == -1) return 7;      // 下
    if (dx == -1 && dy == -1) return 8;     // 左下
    
    return 0;  // 未知方向
}

// 世界坐标转地图坐标
cv::Point OAstar::worldToMap(double world_x, double world_y) const {
    int map_x = static_cast<int>((world_x - o_map_origin_x_) / o_map_resolution_);
    int map_y = static_cast<int>((world_y - o_map_origin_y_) / o_map_resolution_);
    return cv::Point(map_x, map_y);
}

cv::Point OAstar::worldToMap(const geometry_msgs::Point& point) const {
    return worldToMap(point.x, point.y);
}

// 地图坐标转世界坐标
geometry_msgs::Pose OAstar::mapToWorldPose(double map_x, double map_y) const {
    geometry_msgs::Pose pose;
    pose.position.x = o_map_origin_x_ + (map_x + 0.5) * o_map_resolution_;
    pose.position.y = o_map_origin_y_ + (map_y + 0.5) * o_map_resolution_;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

geometry_msgs::PoseStamped OAstar::mapToWorldPose(const cv::Point& map_point) const {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose = mapToWorldPose(map_point.x, map_point.y);
    return pose;
}

// 检查点是否有效
bool OAstar::isValidPoint(const cv::Point& pt) const {
    if (!o_map_ready_) {
        return false;
    }
    
    if (pt.x < 0 || pt.x >= o_width_ || pt.y < 0 || pt.y >= o_height_) {
        if (o_config_.allow_outside_map) {
            // 允许地图外的点，但会进行调整
            return false;
        }
        return false;
    }
    
    return true;
}

// 检查点是否安全
bool OAstar::isPointSafe(const cv::Point& pt) const {
    if (!isValidPoint(pt)) {
        return false;
    }
    
    try {
        if (o_inflated_map_.empty()) {
            uchar value = o_costmap_.at<uchar>(pt.y, pt.x);
            return (value != O_OBSTACLE);
        } else {
            uchar value = o_inflated_map_.at<uchar>(pt.y, pt.x);
            return (value != O_OBSTACLE);
        }
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV exception in isPointSafe: %s", e.what());
        return false;
    }
}

// 详细安全检查
bool OAstar::isPointSafeDetailed(const cv::Point& pt, double clearance) const {
    if (!isValidPoint(pt)) {
        return false;
    }
    
    // 检查中心点
    if (!isPointSafe(pt)) {
        return false;
    }
    
    // 如果有清除距离要求，检查周围区域
    if (clearance > 0.0) {
        int clearance_cells = static_cast<int>(clearance / o_map_resolution_);
        if (clearance_cells < 1) clearance_cells = 1;
        
        for (int dy = -clearance_cells; dy <= clearance_cells; dy++) {
            for (int dx = -clearance_cells; dx <= clearance_cells; dx++) {
                cv::Point check_pt(pt.x + dx, pt.y + dy);
                
                if (!isValidPoint(check_pt)) {
                    continue;
                }
                
                uchar value = o_inflated_map_.at<uchar>(check_pt.y, check_pt.x);
                if (value == O_OBSTACLE) {
                    // 计算距离
                    double distance = sqrt(dx * dx + dy * dy) * o_map_resolution_;
                    if (distance < clearance) {
                        return false;
                    }
                }
            }
        }
    }
    
    return true;
}

// 检查路径安全性
bool OAstar::isPathSafe(const std::vector<cv::Point>& path, double clearance) const {
    if (path.empty()) {
        return true;
    }
    
    // 检查所有点
    for (const auto& point : path) {
        if (!isPointSafeDetailed(point, clearance)) {
            ROS_WARN("Path point (%d, %d) is not safe!", point.x, point.y);
            return false;
        }
    }
    
    // 检查路径段
    for (size_t i = 0; i < path.size() - 1; i++) {
        if (!checkPathSegmentSafety(path[i], path[i + 1], clearance)) {
            ROS_WARN("Path segment from (%d, %d) to (%d, %d) has collision or is too close to obstacle!",
                     path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
            return false;
        }
    }
    
    return true;
}

// 统计不安全点
int OAstar::countUnsafePoints(const std::vector<cv::Point>& path) const {
    int unsafe_count = 0;
    for (const auto& point : path) {
        if (!isPointSafeDetailed(point, o_config_.safety_margin)) {
            unsafe_count++;
        }
    }
    return unsafe_count;
}

// 调整到自由点
cv::Point OAstar::adjustToFreePoint(const cv::Point& point, int max_radius) const {
    return adjustToSafePoint(point, 0.0);
}

// 调整到安全点
cv::Point OAstar::adjustToSafePoint(const cv::Point& point, double clearance) const {
    if (isValidPoint(point) && isPointSafeDetailed(point, clearance)) {
        return point;
    }
    
    // 螺旋搜索最近的安全点
    int max_radius = o_config_.max_search_radius;
    
    for (int radius = 1; radius <= max_radius; radius++) {
        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                if (dx == 0 && dy == 0) continue;
                
                cv::Point check_point(point.x + dx, point.y + dy);
                if (isValidPoint(check_point) && isPointSafeDetailed(check_point, clearance)) {
                    ROS_DEBUG("Adjusted point from (%d, %d) to (%d, %d)",
                             point.x, point.y, check_point.x, check_point.y);
                    return check_point;
                }
            }
        }
    }
    
    ROS_WARN("Cannot find safe point near (%d, %d) within radius %d and clearance %.2f",
             point.x, point.y, max_radius, clearance);
    return point;
}

// 点转索引
int OAstar::pointToIndex(const cv::Point& pt) const {
    return pt.y * o_width_ + pt.x;
}

// 索引转点
cv::Point OAstar::indexToPoint(int index) const {
    int y = index / o_width_;
    int x = index % o_width_;
    return cv::Point(x, y);
}

// 获取起点
cv::Point OAstar::getStartPoint() const {
    return o_start_point_;
}

// 获取目标点
cv::Point OAstar::getTargetPoint() const {
    return o_target_point_;
}
// 起点是否准备好
bool OAstar::isStartReady() const {
    return o_start_ready_;
}

// 目标点是否准备好
bool OAstar::isTargetReady() const {
    return o_target_ready_;
}

// 重置规划器
void OAstar::reset() {
    std::lock_guard<std::mutex> lock1(o_map_mutex_);
    std::lock_guard<std::mutex> lock2(o_planning_mutex_);
    
    clearContainers();
    
    // 释放OpenCV矩阵
    o_costmap_.release();
    o_inflated_map_.release();
    
    // 重置状态
    o_map_ready_ = false;
    o_start_ready_ = false;
    o_target_ready_ = false;
    o_is_planning_ = false;
    
    // 重置原始地图相关
    original_map_initialized_ = false;
    first_plan_completed_ = false;
    first_plan_path_.clear();
    
    // 重置路径度量
    path_metrics_ = OPathMetrics();
    
    ROS_INFO("OAstar planner reset");
}

// 重置规划器状态
void OAstar::resetPlanner() {
    clearContainers();
    o_is_planning_ = false;
    path_metrics_ = OPathMetrics();  // 重置路径度量
}

// 分析失败原因
void OAstar::analyzeFailureReason(const cv::Point& start, const cv::Point& target) const {
    ROS_WARN("=== OA* Path Planning Failure Analysis ===");
    
    if (!isValidPoint(start)) {
        ROS_WARN("Start point (%d, %d) is outside map bounds [%d, %d] x [%d, %d]",
                 start.x, start.y, 0, o_width_ - 1, 0, o_height_ - 1);
    } else if (!isPointSafeDetailed(start, o_config_.safety_margin)) {
        ROS_WARN("Start point (%d, %d) is in obstacle or too close to obstacle (clearance: %.2f)",
                 start.x, start.y, o_config_.safety_margin);
    }
    
    if (!isValidPoint(target)) {
        ROS_WARN("Target point (%d, %d) is outside map bounds [%d, %d] x [%d, %d]",
                 target.x, target.y, 0, o_width_ - 1, 0, o_height_ - 1);
    } else if (!isPointSafeDetailed(target, o_config_.safety_margin)) {
        ROS_WARN("Target point (%d, %d) is in obstacle or too close to obstacle (clearance: %.2f)",
                 target.x, target.y, o_config_.safety_margin);
    }
    
    if (isValidPoint(start) && isValidPoint(target) &&
        isPointSafeDetailed(start, o_config_.safety_margin) &&
        isPointSafeDetailed(target, o_config_.safety_margin)) {
        ROS_WARN("Both points are valid and safe, but no path found.");
        ROS_WARN("Start: (%d, %d), Target: (%d, %d)", start.x, start.y, target.x, target.y);
        ROS_WARN("Map size: %dx%d, resolution: %.3f", o_width_, o_height_, o_map_resolution_);
        ROS_WARN("Safety margin: %.2f, Clearance: %.2f", o_config_.safety_margin, o_config_.obstacle_clearance);
        ROS_WARN("Ignore dynamic obstacles: %s", ignore_dynamic_obstacles_ ? "true" : "false");
        ROS_WARN("Lock original map: %s", lock_original_map_ ? "true" : "false");
    }
    
    ROS_WARN("=== End Failure Analysis ===");
}

// 转换到世界路径
nav_msgs::Path OAstar::convertToWorldPath(const std::vector<cv::Point>& map_path) const {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    
    for (const auto& point : map_path) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        geometry_msgs::Pose world_pose = mapToWorldPose(point.x, point.y);
        pose.pose = world_pose;
        path_msg.poses.push_back(pose);
    }
    
    return path_msg;
}

// 打印调试信息
void OAstar::printDebugInfo() const {
    ROS_INFO("=== OA* Debug Info ===");
    ROS_INFO("Map ready: %s", o_map_ready_ ? "true" : "false");
    ROS_INFO("Map size: %dx%d, resolution: %.3f", o_width_, o_height_, o_map_resolution_);
    ROS_INFO("Start ready: %s, point: (%d, %d)",
             o_start_ready_ ? "true" : "false", o_start_point_.x, o_start_point_.y);
    ROS_INFO("Target ready: %s, point: (%d, %d)",
             o_target_ready_ ? "true" : "false", o_target_point_.x, o_target_point_.y);
    ROS_INFO("Last planning time: %.3f seconds ago",
             (ros::Time::now() - o_last_planning_time_).toSec());
    ROS_INFO("Collision checks: %d, Unsafe points fixed: %d",
             path_metrics_.collision_checks, path_metrics_.unsafe_points_fixed);
    ROS_INFO("Ignore dynamic obstacles: %s", ignore_dynamic_obstacles_ ? "true" : "false");
    ROS_INFO("Lock original map: %s", lock_original_map_ ? "true" : "false");
    ROS_INFO("First plan completed: %s, path points: %zu",
             first_plan_completed_ ? "true" : "false", first_plan_path_.size());
}

// 获取三线表格字符串
std::string OAstar::getSimpleThreeLineTable() const {
    return path_metrics_.toSimpleThreeLineTable();
}

// 打印度量表格
void OAstar::printMetricsTable() const {
    ROS_INFO("\n%s", getSimpleThreeLineTable().c_str());
}