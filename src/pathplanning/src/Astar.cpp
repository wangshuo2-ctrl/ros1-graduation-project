#include "pathplanning/Astar.h"
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

Astar::Astar() : 
    map_ready_(false),
    start_ready_(false),
    target_ready_(false),
    is_planning_(false),
    width_(0),
    height_(0),
    map_resolution_(0.0),
    map_origin_x_(0.0),
    map_origin_y_(0.0),
    last_planning_time_(ros::Time(0)) {
    ROS_INFO("Astar constructor called");
}

Astar::~Astar() {
    ROS_INFO("Astar destructor called");
}

void Astar::clearOpenSet() {
    while (!open_set_.empty()) {
        open_set_.pop();
    }
}

void Astar::clearNodes() {
    all_nodes_.clear();
    close_set_.clear();
}

void Astar::clearContainers() {
    clearOpenSet();
    clearNodes();
}

void Astar::initAstar(const AstarConfig& config) {
    ROS_INFO("Astar::initAstar called with configuration");
    config_ = config;
    
    // 设置曲率约束的默认值
    if (config_.min_turn_radius <= 0) {
        config_.min_turn_radius = 0.7;  // 从0.5增加到0.7米
    }
    
    if (config_.max_curvature <= 0) {
        config_.max_curvature = 1.5;    // 从2.0减小到1.5
    }
    
    ROS_INFO("Astar parameters configured:");
    ROS_INFO("  heuristic_weight: %.2f", config_.heuristic_weight);
    ROS_INFO("  inflate_radius: %d (增强: 从5增加到8)", config_.inflate_radius);
    ROS_INFO("  safety_margin: %.2f m (增强: 从0.3增加到0.4)", config_.safety_margin);
    ROS_INFO("  obstacle_clearance: %.2f m (增强: 从0.3增加到0.5)", config_.obstacle_clearance);
    ROS_INFO("  max_iterations: %d", config_.max_iterations);
    ROS_INFO("  enable_path_smoothing: %s", config_.enable_path_smoothing ? "true" : "false");
    ROS_INFO("  enable_bspline_smoothing: %s", config_.enable_bspline_smoothing ? "true" : "false");
    ROS_INFO("  bspline_degree: %d", config_.bspline_degree);
    ROS_INFO("  bspline_samples: %d (优化: 从200减少到150)", config_.bspline_samples);
    ROS_INFO("  bspline_smoothness: %.2f (优化: 从0.7减小到0.6)", config_.bspline_smoothness);
    ROS_INFO("  enable_path_resample: %s", config_.enable_path_resample ? "true" : "false");
    ROS_INFO("  resample_points: %d", config_.resample_points);
    ROS_INFO("  enable_safety_check: %s", config_.enable_safety_check ? "true" : "false");
    ROS_INFO("  safety_check_distance: %.2f m (增强: 从0.3增加到0.4)", config_.safety_check_distance);
    ROS_INFO("  safety_check_resolution: %d", config_.safety_check_resolution);
    ROS_INFO("  enable_constrained_smoothing: %s", config_.enable_constrained_smoothing ? "true" : "false");
    ROS_INFO("  constrained_smoothing_iterations: %d", config_.constrained_smoothing_iterations);
    ROS_INFO("  constrained_smoothing_weight: %.2f", config_.constrained_smoothing_weight);
    ROS_INFO("  min_turn_radius: %.2f m (增强: 从0.5增加到0.7)", config_.min_turn_radius);
    ROS_INFO("  max_curvature: %.2f 1/m (增强: 从2.0减小到1.5)", config_.max_curvature);
    ROS_INFO("  enable_curvature_constraint: %s", config_.enable_curvature_constraint ? "true" : "false");
    ROS_INFO("  secondary_selection_max_iterations: %d (增强: 从5增加到8)", config_.secondary_selection_max_iterations);
    ROS_INFO("  secondary_selection_search_radius: %.2f m (增强: 从0.2增加到0.3)", config_.secondary_selection_search_radius);
    
    ROS_INFO("Astar initialization completed successfully with enhanced safety parameters");
}

void Astar::initAstar(double heuristic_weight, int inflate_radius, double safety_margin) {
    AstarConfig config;
    config.heuristic_weight = heuristic_weight;
    config.inflate_radius = inflate_radius;
    config.safety_margin = safety_margin;
    
    // 使用增强的安全参数
    config.min_turn_radius = 0.7;  // 增强
    config.max_curvature = 1.5;    // 增强
    config.enable_curvature_constraint = true;
    config.secondary_selection_max_iterations = 8;  // 增强
    config.secondary_selection_search_radius = 0.3; // 增强
    config.inflate_radius = 8;     // 增强
    config.obstacle_clearance = 0.5; // 增强
    config.safety_check_distance = 0.4; // 增强
    config.bspline_samples = 150;  // 优化
    config.bspline_smoothness = 0.6; // 优化
    
    initAstar(config);
}

void Astar::setConfig(const AstarConfig& config) {
    config_ = config;
    ROS_INFO("Astar config updated");
}

void Astar::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!grid) {
        ROS_ERROR("Received null map pointer");
        return;
    }
    
    if (grid->info.width == 0 || grid->info.height == 0) {
        ROS_WARN("Received empty map");
        return;
    }
    
    width_ = grid->info.width;
    height_ = grid->info.height;
    map_resolution_ = grid->info.resolution;
    map_origin_x_ = grid->info.origin.position.x;
    map_origin_y_ = grid->info.origin.position.y;
    
    ROS_INFO("Updating map: %dx%d, resolution: %.3f, origin: (%.2f, %.2f)", 
             width_, height_, map_resolution_, map_origin_x_, map_origin_y_);
    
    try {
        occupancyGridToMat(*grid);
        processMap();
        map_ready_ = true;
        ROS_INFO("Map updated and ready for planning");
    } catch (const std::exception& e) {
        ROS_ERROR("Error updating map: %s", e.what());
        map_ready_ = false;
    }
}

void Astar::occupancyGridToMat(const nav_msgs::OccupancyGrid& grid) {
    if (grid.data.empty()) {
        ROS_WARN("Map data is empty");
        return;
    }
    
    costmap_ = cv::Mat(height_, width_, CV_8UC1);
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int index = y * width_ + x;
            if (index < 0 || index >= static_cast<int>(grid.data.size())) {
                ROS_WARN("Index out of bounds: %d, data size: %zu", index, grid.data.size());
                costmap_.at<uchar>(y, x) = UNKNOWN;
                continue;
            }
            
            int8_t value = grid.data[index];
            if (value < 0) {
                costmap_.at<uchar>(y, x) = UNKNOWN;
            } else if (value < config_.obstacle_threshold * 100) {
                costmap_.at<uchar>(y, x) = FREE;
            } else {
                costmap_.at<uchar>(y, x) = OBSTACLE;
            }
        }
    }
    
    ROS_INFO("Occupancy grid converted to OpenCV mat");
}

void Astar::processMap() {
    if (costmap_.empty()) {
        ROS_WARN("Cannot process map: costmap is empty");
        return;
    }
    
    try {
        // 创建膨胀地图
        inflated_map_ = costmap_.clone();
        
        if (config_.inflate_radius > 0) {
            int kernel_size = config_.inflate_radius * 2 + 1;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
            cv::dilate(costmap_, inflated_map_, kernel);
        }
        
        ROS_INFO("Map processing complete, inflated radius: %d (增强)", config_.inflate_radius);
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV error processing map: %s", e.what());
        inflated_map_ = costmap_.clone();
    }
}

void Astar::setStartPoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Null start pointer received");
        return;
    }
    
    if (!map_ready_) {
        ROS_WARN("Cannot set start point: map not ready");
        return;
    }
    
    cv::Point map_point = worldToMap(msg->pose.pose.position.x, msg->pose.pose.position.y);
    setStartPoint(map_point);
}

void Astar::setTargetPoint(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Null target pointer received");
        return;
    }
    
    if (!map_ready_) {
        ROS_WARN("Cannot set target point: map not ready");
        return;
    }
    
    cv::Point map_point = worldToMap(msg->pose.position.x, msg->pose.position.y);
    setTargetPoint(map_point);
}

void Astar::setStartPoint(cv::Point point) {
    if (!map_ready_) {
        ROS_WARN("Cannot set start point: map not ready");
        return;
    }
    
    if (isValidPoint(point)) {
        if (isPointSafeDetailed(point, config_.safety_margin)) {
            start_point_ = point;
            start_ready_ = true;
            ROS_INFO("Start point set to: (%d, %d)", start_point_.x, start_point_.y);
        } else {
            ROS_WARN("Start point (%d, %d) is not safe, adjusting...", point.x, point.y);
            start_point_ = adjustToSafePoint(point, config_.safety_margin);
            if (isPointSafeDetailed(start_point_, config_.safety_margin)) {
                start_ready_ = true;
                ROS_INFO("Adjusted start point to: (%d, %d)", start_point_.x, start_point_.y);
            } else {
                ROS_ERROR("Cannot find safe start point near (%d, %d)", point.x, point.y);
            }
        }
    } else {
        ROS_WARN("Start point (%d, %d) is invalid", point.x, point.y);
    }
}

void Astar::setTargetPoint(cv::Point point) {
    if (!map_ready_) {
        ROS_WARN("Cannot set target point: map not ready");
        return;
    }
    
    if (isValidPoint(point)) {
        if (isPointSafeDetailed(point, config_.safety_margin)) {
            target_point_ = point;
            target_ready_ = true;
            ROS_INFO("Target point set to: (%d, %d)", target_point_.x, target_point_.y);
        } else {
            ROS_WARN("Target point (%d, %d) is not safe, adjusting...", point.x, point.y);
            target_point_ = adjustToSafePoint(point, config_.safety_margin);
            if (isPointSafeDetailed(target_point_, config_.safety_margin)) {
                target_ready_ = true;
                ROS_INFO("Adjusted target point to: (%d, %d)", target_point_.x, target_point_.y);
            } else {
                ROS_ERROR("Cannot find safe target point near (%d, %d)", point.x, point.y);
            }
        }
    } else {
        ROS_WARN("Target point (%d, %d) is invalid", point.x, point.y);
    }
}

bool Astar::pathPlanning(std::vector<cv::Point>& path) {
    if (!isReadyForPlanning()) {
        ROS_WARN("Not ready for planning. Map ready: %s, Start ready: %s, Target ready: %s",
                 map_ready_ ? "true" : "false",
                 start_ready_ ? "true" : "false",
                 target_ready_ ? "true" : "false");
        return false;
    }
    
    return pathPlanning(start_point_, target_point_, path);
}

bool Astar::pathPlanning(cv::Point start_point, cv::Point target_point, std::vector<cv::Point>& path) {
    if (is_planning_) {
        ROS_WARN("Path planning already in progress");
        return false;
    }
    
    ROS_INFO("Starting path planning from (%d, %d) to (%d, %d)",
             start_point.x, start_point.y, target_point.x, target_point.y);
    
    // 重置规划器状态
    resetPlanner();
    is_planning_ = true;
    
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 执行A*算法
    std::shared_ptr<Node> end_node = findPath(start_point, target_point);
    
    // 记录结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    path_metrics_.planning_time = std::chrono::duration<double>(end_time - start_time).count();
    
    if (end_node == nullptr) {
        ROS_ERROR("A* path planning failed! No path found after %d iterations", path_metrics_.iterations);
        is_planning_ = false;
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
    
    ROS_INFO("Raw A* path generated: %zu points, length: %.3f m, unsafe points: %d", 
             path.size(), original_length, unsafe_points);
    
    // 应用路径平滑和优化
    if (path.size() > 2) {
        ROS_INFO("Applying path smoothing and optimization with enhanced safety...");
        
        // 第一步: 视线平滑，剔除冗余点
        if (config_.enable_path_smoothing) {
            smoothPath(path);
            ROS_INFO("After line-of-sight smoothing: %zu points", path.size());
            
            // 检查平滑后路径安全性
            unsafe_points = countUnsafePoints(path);
            if (unsafe_points > 0) {
                ROS_WARN("Smoothed path has %d unsafe points! Adjusting...", unsafe_points);
                for (size_t i = 0; i < path.size(); i++) {
                    if (!isPointSafeDetailed(path[i], config_.safety_margin)) {
                        path[i] = adjustToSafePoint(path[i], config_.safety_margin);
                        path_metrics_.unsafe_points_fixed++;
                    }
                }
            }
        }
        
        // 第二步: 简化，剔除共线点
        simplifyPath(path);
        ROS_INFO("After simplification: %zu points", path.size());
        
        // 第三步: 约束平滑
        if (config_.enable_constrained_smoothing && path.size() >= 3) {
            constrainedSmoothPath(path);
            ROS_INFO("After constrained smoothing: %zu points", path.size());
        }
        
        // 第四步: B样条平滑（包含曲率约束和二次选取策略）
        if (config_.enable_bspline_smoothing && path.size() >= 4) {
            try {
                std::vector<cv::Point> original_path = path;
                bsplineSmooth(path);
                
                // 检查B样条平滑后的安全性
                unsafe_points = countUnsafePoints(path);
                if (unsafe_points > 0) {
                    ROS_WARN("B-spline smoothed path has %d unsafe points! Adjusting...", unsafe_points);
                    for (size_t i = 0; i < path.size(); i++) {
                        if (!isPointSafeDetailed(path[i], config_.safety_margin)) {
                            path[i] = adjustToSafePoint(path[i], config_.safety_margin);
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
        if (config_.enable_path_resample && path.size() >= 2) {
            try {
                std::vector<cv::Point> resampled_path = resamplePath(path, config_.resample_points);
                
                // 检查重采样路径安全性
                if (isPathSafe(resampled_path, config_.safety_margin)) {
                    path = resampled_path;
                    ROS_INFO("Path resampled: %zu points -> %zu points", 
                             path.size(), resampled_path.size());
                } else {
                    ROS_WARN("Resampled path is not safe, using original");
                }
            } catch (const std::exception& e) {
                ROS_WARN("Path resampling failed: %s", e.what());
            }
        }
        
        // 最终安全检查
        if (config_.enable_safety_check) {
            bool final_safe = isPathSafe(path, config_.safety_check_distance);
            if (!final_safe) {
                ROS_WARN("Final path safety check failed, making final adjustments");
                for (size_t i = 0; i < path.size(); i++) {
                    if (!isPointSafeDetailed(path[i], config_.safety_check_distance)) {
                        cv::Point adjusted = adjustToSafePoint(path[i], config_.safety_check_distance);
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
    
    ROS_INFO("Final path: %zu points, length: %.3f m, unsafe points fixed: %d", 
             path.size(), path_metrics_.length, path_metrics_.unsafe_points_fixed);
    
    // 最终路径点数
    path_metrics_.points_count = path.size();
    
    // 输出规划结果
    ROS_INFO("Path planning successful!");
    printMetricsTable();
    
    // 清理内存
    clearContainers();
    is_planning_ = false;
    last_planning_time_ = ros::Time::now();
    
    return true;
}

std::shared_ptr<Node> Astar::findPath(const cv::Point& start, const cv::Point& target) {
    cv::Point adjusted_start = start;
    cv::Point adjusted_target = target;
    
    if (!map_ready_) {
        ROS_WARN("Cannot find path: map is not ready");
        return nullptr;
    }
    
    if (!isValidPoint(adjusted_start) || !isValidPoint(adjusted_target)) {
        ROS_WARN("Cannot find path: start or target point is invalid");
        ROS_WARN("Start: (%d, %d), Target: (%d, %d)",
                 adjusted_start.x, adjusted_start.y, adjusted_target.x, adjusted_target.y);
        return nullptr;
    }
    
    if (!isPointSafeDetailed(adjusted_start, config_.safety_margin)) {
        ROS_WARN("Start point (%d, %d) is not safe, adjusting...", 
                 adjusted_start.x, adjusted_start.y);
        adjusted_start = adjustToSafePoint(adjusted_start, config_.safety_margin);
        if (!isPointSafeDetailed(adjusted_start, config_.safety_margin)) {
            ROS_ERROR("Cannot find safe start point near (%d, %d)", 
                      adjusted_start.x, adjusted_start.y);
            return nullptr;
        }
    }
    
    if (!isPointSafeDetailed(adjusted_target, config_.safety_margin)) {
        ROS_WARN("Target point (%d, %d) is not safe, adjusting...", 
                 adjusted_target.x, adjusted_target.y);
        adjusted_target = adjustToSafePoint(adjusted_target, config_.safety_margin);
        if (!isPointSafeDetailed(adjusted_target, config_.safety_margin)) {
            ROS_ERROR("Cannot find safe target point near (%d, %d)", 
                      adjusted_target.x, adjusted_target.y);
            return nullptr;
        }
    }
    
    if (adjusted_start == adjusted_target) {
        ROS_WARN("Start and target points are the same");
        return std::make_shared<Node>(adjusted_start);
    }
    
    ROS_INFO("Starting A* search from (%d, %d) to (%d, %d)",
             adjusted_start.x, adjusted_start.y, adjusted_target.x, adjusted_target.y);
    
    // 重置统计
    path_metrics_.iterations = 0;
    path_metrics_.collision_checks = 0;
    path_metrics_.unsafe_points_fixed = 0;
    
    // 清除之前的节点
    clearContainers();
    
    // 创建起点节点
    auto start_node = std::make_shared<Node>(adjusted_start);
    start_node->g_cost = 0;
    start_node->h_cost = calculateHeuristic(adjusted_start, adjusted_target);
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    
    // 添加到开放集
    open_set_.push(start_node);
    int start_index = pointToIndex(adjusted_start);
    all_nodes_[start_index] = start_node;
    
    // 主搜索循环
    while (!open_set_.empty() && path_metrics_.iterations < config_.max_iterations) {
        path_metrics_.iterations++;
        
        // 获取f值最小的节点
        auto current_node = open_set_.top();
        open_set_.pop();
        
        cv::Point current_point = current_node->point;
        int current_index = pointToIndex(current_point);
        
        // 检查是否已在关闭集
        if (close_set_.find(current_index) != close_set_.end()) {
            continue;
        }
        
        // 添加到关闭集
        close_set_.insert(current_index);
        
        // 检查是否到达目标
        if (current_point == adjusted_target) {
            ROS_INFO("Found path! Iterations: %d, Cost: %.2f, Collision checks: %d",
                     path_metrics_.iterations, 
                     current_node->g_cost,
                     path_metrics_.collision_checks);
            return current_node;
        }
        
        // 获取有效邻居
        std::vector<cv::Point> neighbors = getValidNeighbors(current_point);
        
        for (const cv::Point& neighbor_point : neighbors) {
            int neighbor_index = pointToIndex(neighbor_point);
            
            // 检查是否已在关闭集
            if (close_set_.find(neighbor_index) != close_set_.end()) {
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
            auto it = all_nodes_.find(neighbor_index);
            if (it != all_nodes_.end()) {
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
                auto neighbor_node = std::make_shared<Node>(neighbor_point);
                neighbor_node->parent = current_node;
                neighbor_node->g_cost = tentative_g;
                neighbor_node->h_cost = calculateHeuristic(neighbor_point, adjusted_target);
                neighbor_node->f_cost = neighbor_node->g_cost + neighbor_node->h_cost;
                neighbor_node->direction = getDirection(current_point, neighbor_point);
                
                all_nodes_[neighbor_index] = neighbor_node;
                open_set_.push(neighbor_node);
            }
        }
    }
    
    ROS_WARN("A* search failed: maximum iterations (%d) reached or no path found", 
             path_metrics_.iterations);
    return nullptr;
}

void Astar::extractPath(const std::shared_ptr<Node>& end_node, std::vector<cv::Point>& path) {
    path.clear();
    
    if (!end_node) {
        ROS_WARN("Cannot extract path: end node is null");
        return;
    }
    
    // 从终点回溯到起点
    std::shared_ptr<Node> current = end_node;
    std::vector<cv::Point> reverse_path;
    
    while (current != nullptr) {
        reverse_path.push_back(current->point);
        current = current->parent;
    }
    
    // 反转路径
    path.resize(reverse_path.size());
    std::reverse_copy(reverse_path.begin(), reverse_path.end(), path.begin());
    
    ROS_INFO("Path extracted: %zu points", path.size());
    path_metrics_.points_count = path.size();
}

void Astar::smoothPath(std::vector<cv::Point>& path) {
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
        ROS_INFO("Path smoothed: %zu points -> %zu points", path.size(), smoothed_path.size());
        path = smoothed_path;
    }
}

void Astar::simplifyPath(std::vector<cv::Point>& path) {
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

void Astar::constrainedSmoothPath(std::vector<cv::Point>& path) {
    if (path.size() < 3) {
        return;
    }
    
    ROS_INFO("Applying constrained smoothing with %d iterations, weight: %.2f",
             config_.constrained_smoothing_iterations,
             config_.constrained_smoothing_weight);
    
    std::vector<cv::Point> smoothed_path = path;
    
    for (int iter = 0; iter < config_.constrained_smoothing_iterations; ++iter) {
        std::vector<cv::Point> new_path = smoothed_path;
        
        // 跳过起点和终点
        for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
            // 计算当前位置
            double x = smoothed_path[i].x;
            double y = smoothed_path[i].y;
            
            // 计算梯度（朝向相邻点的中心）
            double prev_x = smoothed_path[i - 1].x;
            double prev_y = smoothed_path[i - 1].y;
            double next_x = smoothed_path[i + 1].x;
            double next_y = smoothed_path[i + 1].y;
            
            double gradient_x = (prev_x + next_x) * 0.5 - x;
            double gradient_y = (prev_y + next_y) * 0.5 - y;
            
            // 应用约束梯度下降
            double new_x = x + gradient_x * config_.constrained_smoothing_weight;
            double new_y = y + gradient_y * config_.constrained_smoothing_weight;
            
            cv::Point new_point(static_cast<int>(new_x + 0.5), static_cast<int>(new_y + 0.5));
            
            // 安全检查：确保新点不进入障碍物
            if (isPointSafeDetailed(new_point, config_.safety_margin)) {
                new_path[i] = new_point;
            } else {
                // 如果新点不安全，尝试在梯度方向上找到最近的安全点
                cv::Point safe_point = adjustToSafePoint(new_point, config_.safety_margin);
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
        if (!isPointSafeDetailed(smoothed_path[i], config_.safety_margin)) {
            smoothed_path[i] = adjustToSafePoint(smoothed_path[i], config_.safety_margin);
            path_metrics_.unsafe_points_fixed++;
        }
    }
    
    path = smoothed_path;
    
    ROS_INFO("Constrained smoothing applied, %d iterations, fixed %d unsafe points",
             config_.constrained_smoothing_iterations, path_metrics_.unsafe_points_fixed);
}

// 修改后的B样条平滑函数，包含曲率约束和二次选取策略
void Astar::bsplineSmooth(std::vector<cv::Point>& path) {
    if (path.size() < 4) {
        ROS_WARN("Not enough points for B-spline smoothing (need at least 4 points)");
        return;
    }
    
    ROS_INFO("Applying B-spline smoothing to path with %zu points", path.size());
    ROS_INFO("B-spline parameters: degree=%d, samples=%d, smoothness=%.2f",
             config_.bspline_degree, config_.bspline_samples, config_.bspline_smoothness);
    
    // 保存原始路径用于二次选取
    std::vector<cv::Point> original_path = path;
    
    // 最大迭代次数，防止无限循环
    int max_iterations = config_.secondary_selection_max_iterations;
    int iteration = 0;
    bool smoothing_success = false;
    
    // 获取最大允许曲率
    double max_curvature = config_.max_curvature;
    
    // 计算增强的安全距离
    double enhanced_clearance = config_.safety_margin * 1.5;  // 增加50%的安全距离
    ROS_INFO("Using enhanced clearance for B-spline: %.2f m (original: %.2f m)", 
             enhanced_clearance, config_.safety_margin);
    
    while (iteration < max_iterations && !smoothing_success) {
        iteration++;
        ROS_INFO("B-spline smoothing iteration %d/%d", iteration, max_iterations);
        
        // 使用当前路径作为控制点
        std::vector<cv::Point> control_points = path;
        
        // 计算B样条曲线
        std::vector<std::pair<double, double>> spline_points = computeBSpline(control_points,
                                                                              config_.bspline_degree,
                                                                              config_.bspline_samples);
        
        // 将平滑后的点转换回整数坐标
        std::vector<cv::Point> smoothed_path;
        smoothed_path.reserve(spline_points.size());
        
        for (const auto& point : spline_points) {
            int x = static_cast<int>(point.first + 0.5);
            int y = static_cast<int>(point.second + 0.5);
            
            // 检查点是否在地图范围内
            if (x < 0) x = 0;
            if (x >= width_) x = width_ - 1;
            if (y < 0) y = 0;
            if (y >= height_) y = height_ - 1;
            
            smoothed_path.push_back(cv::Point(x, y));
        }
        
        // 移除重复点
        smoothed_path.erase(std::unique(smoothed_path.begin(), smoothed_path.end()),
                           smoothed_path.end());
        
        if (smoothed_path.size() < 2) {
            ROS_WARN("B-spline smoothing produced too few points");
            break;
        }
        
        // 检查1: 使用增强安全距离进行路径碰撞检查
        bool collision_free = true;
        int collision_segment = -1;
        
        for (size_t i = 0; i < smoothed_path.size() - 1; i++) {
            if (!checkPathSegmentSafety(smoothed_path[i], smoothed_path[i + 1], enhanced_clearance)) {
                ROS_INFO("B-spline segment %zu has collision or is too close to obstacle (distance < %.2fm)", 
                         i, enhanced_clearance);
                collision_free = false;
                collision_segment = static_cast<int>(i);
                break;
            }
        }
        
        // 检查2: 曲率约束检查
        bool curvature_ok = true;
        if (collision_free && config_.enable_curvature_constraint) {
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
                if (!isPointSafeDetailed(smoothed_path[i], config_.safety_margin)) {
                    ROS_WARN("Smoothed path point %zu is unsafe, adjusting...", i);
                    cv::Point safe_point = adjustToSafePoint(smoothed_path[i], config_.safety_margin);
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
            int control_point_index = static_cast<int>((double)collision_segment / smoothed_path.size() * control_points.size());
            control_point_index = std::max(0, std::min(control_point_index, static_cast<int>(control_points.size()) - 1));
            
            // 在碰撞点附近调整控制点
            std::vector<cv::Point> new_control_points = adjustControlPointsForCollision(control_points, control_point_index);
            
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
                // 简化路径：只保留关键点
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
        
        // 如果所有策略都失败，尝试回退到原始路径
        if (iteration == max_iterations) {
            ROS_WARN("All smoothing strategies failed after %d iterations", iteration);
            
            // 尝试使用原始路径进行约束平滑
            std::vector<cv::Point> temp_path = original_path;
            constrainedSmoothPath(temp_path);
            
            if (isPathSafe(temp_path, config_.safety_margin)) {
                path = temp_path;
                smoothing_success = true;
                ROS_INFO("Using constrained smoothing as fallback: %zu points", path.size());
            } else {
                // 最后的回退：使用原始A*路径
                path = original_path;
                ROS_WARN("Using original A* path due to smoothing failure");
            }
        }
    }
    
    if (!smoothing_success) {
        ROS_WARN("B-spline smoothing failed, using constrained smoothing instead");
        constrainedSmoothPath(path);
    }
}

// 检查路径的曲率约束
bool Astar::checkCurvatureConstraint(const std::vector<cv::Point>& path, double max_curvature) {
    if (path.size() < 3) {
        return true;  // 少于3个点无法计算曲率
    }
    
    // 将地图坐标转换为世界坐标
    auto toWorld = [&](const cv::Point& p) -> std::pair<double, double> {
        double world_x = map_origin_x_ + (p.x + 0.5) * map_resolution_;
        double world_y = map_origin_y_ + (p.y + 0.5) * map_resolution_;
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
        
        // 计算近似曲率（假设在小角度下，曲率 ≈ 角度变化 / 弧长）
        double avg_length = (l1 + l2) / 2.0;
        double curvature = 2.0 * fabs(sin(angle / 2.0)) / avg_length;
        
        if (curvature > max_curvature) {
            ROS_INFO("Curvature constraint violated at point %zu: curvature=%.4f > max_curvature=%.4f", 
                     i, curvature, max_curvature);
            ROS_INFO("  World points: (%.2f,%.2f) -> (%.2f,%.2f) -> (%.2f,%.2f)",
                     w0.first, w0.second, w1.first, w1.second, w2.first, w2.second);
            return false;
        }
    }
    
    return true;
}

// 调整控制点以避开障碍物
std::vector<cv::Point> Astar::adjustControlPointsForCollision(
    const std::vector<cv::Point>& control_points, 
    int collision_index) {
    
    std::vector<cv::Point> adjusted_points = control_points;
    
    if (collision_index < 0 || collision_index >= static_cast<int>(control_points.size())) {
        return adjusted_points;
    }
    
    // 将搜索半径转换为地图单位
    int search_radius_cells = static_cast<int>(config_.secondary_selection_search_radius / map_resolution_);
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
        if (!isPointSafeDetailed(adjusted_points[i], config_.safety_margin)) {
            // 在多个方向上寻找安全点
            cv::Point best_point = original_point;
            double best_score = -std::numeric_limits<double>::max();
            
            // 搜索方向：8个主要方向
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
                    
                    // 计算得分：安全距离 + 保持原始位置接近性
                    double safety_score = calculateSafetyScore(test_point);
                    double proximity_penalty = sqrt(dx*dx + dy*dy) * map_resolution_ * 0.5;  // 惩罚远离原始位置
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
                ROS_INFO("  Adjusted control point %d from (%d,%d) to (%d,%d)", 
                         i, original_point.x, original_point.y, 
                         best_point.x, best_point.y);
            }
        }
    }
    
    // 策略2: 在碰撞区域插入额外的控制点（如果碰撞发生在路径中间）
    if (!points_adjusted && collision_index > 0 && collision_index < static_cast<int>(adjusted_points.size()) - 1) {
        cv::Point before = adjusted_points[collision_index - 1];
        cv::Point after = adjusted_points[collision_index + 1];
        
        // 计算中点
        cv::Point mid_point(
            (before.x + after.x) / 2,
            (before.y + after.y) / 2
        );
        
        // 将中点调整到安全位置
        cv::Point safe_mid_point = adjustToSafePoint(mid_point, config_.safety_margin);
        
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

// 计算点的安全评分
double Astar::calculateSafetyScore(const cv::Point& point) {
    if (!isValidPoint(point)) {
        return -std::numeric_limits<double>::max();
    }
    
    // 计算到最近障碍物的距离
    double min_distance = std::numeric_limits<double>::max();
    int search_radius = static_cast<int>(config_.safety_margin * 2 / map_resolution_);
    
    for (int dy = -search_radius; dy <= search_radius; dy++) {
        for (int dx = -search_radius; dx <= search_radius; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            cv::Point test_point(point.x + dx, point.y + dy);
            if (isValidPoint(test_point)) {
                uchar value = inflated_map_.at<uchar>(test_point.y, test_point.x);
                if (value == OBSTACLE) {
                    double distance = sqrt(dx*dx + dy*dy) * map_resolution_;
                    if (distance < min_distance) {
                        min_distance = distance;
                    }
                }
            }
        }
    }
    
    // 如果没有找到障碍物，返回最大搜索距离
    if (min_distance == std::numeric_limits<double>::max()) {
        min_distance = search_radius * map_resolution_;
    }
    
    // 评分：距离越远，分数越高
    return min_distance;
}

std::vector<std::pair<double, double>> Astar::computeBSpline(const std::vector<cv::Point>& control_points,
                                                             int degree, int samples) {
    int n = control_points.size() - 1;  // 控制点索引最大值
    int p = std::min(degree, n);        // 阶数，不能大于控制点数
    
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

std::vector<double> Astar::computeKnotVector(int n, int p) {
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

double Astar::bsplineBasis(int i, int p, double t, const std::vector<double>& knots) {
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

std::vector<cv::Point> Astar::resamplePath(const std::vector<cv::Point>& path, int num_points) {
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
                if (point.x >= width_) point.x = width_ - 1;
                if (point.y < 0) point.y = 0;
                if (point.y >= height_) point.y = height_ - 1;
                
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

double Astar::calculateHeuristic(const cv::Point& a, const cv::Point& b) const {
    if (config_.euclidean) {
        // 欧几里得距离
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy) * config_.heuristic_weight;
    } else {
        // 曼哈顿距离
        double dx = std::abs(a.x - b.x);
        double dy = std::abs(a.y - b.y);
        return (dx + dy) * config_.heuristic_weight;
    }
}

double Astar::calculateTurnPenalty(const std::shared_ptr<Node>& current, const cv::Point& next) const {
    if (!current || !current->parent) {
        return 0.0;
    }
    
    int prev_dir = getDirection(current->parent->point, current->point);
    int curr_dir = getDirection(current->point, next);
    
    if (prev_dir == curr_dir) {
        return 0.0;
    }
    
    return config_.turn_penalty_weight;
}

double Astar::calculateDistance(const cv::Point& p1, const cv::Point& p2) const {
    double dx = static_cast<double>(p2.x - p1.x);
    double dy = static_cast<double>(p2.y - p1.y);
    return sqrt(dx * dx + dy * dy) * map_resolution_;
}

std::vector<cv::Point> Astar::getValidNeighbors(const cv::Point& current) const {
    std::vector<cv::Point> neighbors = getAllNeighbors(current);
    std::vector<cv::Point> valid_neighbors;
    
    for (const cv::Point& neighbor : neighbors) {
        if (isValidPoint(neighbor) && isPointSafeDetailed(neighbor, config_.safety_margin)) {
            // 额外检查：对于对角线移动，确保相邻的两个单元格也是自由的
            int dx = neighbor.x - current.x;
            int dy = neighbor.y - current.y;
            
            if (dx != 0 && dy != 0) {  // 对角线移动
                cv::Point side1(current.x + dx, current.y);
                cv::Point side2(current.x, current.y + dy);
                
                if (!isPointSafeDetailed(side1, config_.safety_margin) ||
                    !isPointSafeDetailed(side2, config_.safety_margin)) {
                    continue;  // 跳过对角线移动，如果角落有障碍物
                }
            }
            
            valid_neighbors.push_back(neighbor);
        }
    }
    
    return valid_neighbors;
}

std::vector<cv::Point> Astar::getAllNeighbors(const cv::Point& current) const {
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

bool Astar::hasLineOfSight(const cv::Point& a, const cv::Point& b) const {
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
        if (!isPointSafeDetailed(cv::Point(x0, y0), config_.safety_margin)) {
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
    
    return isPointSafeDetailed(cv::Point(x1, y1), config_.safety_margin);
}

bool Astar::checkDiagonalSafety(const cv::Point& from, const cv::Point& to) const {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    if (dx == 0 || dy == 0) {
        return true;  // 不是对角线移动
    }
    
    // 检查两个相邻单元格
    cv::Point cell1(from.x + dx, from.y);
    cv::Point cell2(from.x, from.y + dy);
    
    return isPointSafeDetailed(cell1, config_.safety_margin) &&
           isPointSafeDetailed(cell2, config_.safety_margin);
}

bool Astar::checkCollision(const cv::Point& pt1, const cv::Point& pt2) const {
    return !checkPathSegmentSafety(pt1, pt2, 0.0);
}

bool Astar::checkPathSegmentSafety(const cv::Point& pt1, const cv::Point& pt2, double clearance) const {
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
    
    return isPointSafeDetailed(cv::Point(x1, y1), clearance);
}

int Astar::getDirection(const cv::Point& from, const cv::Point& to) const {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    if (dx == 0 && dy == 0) return 0;
    if (dx == -1 && dy == 0)  return 1;   // 左
    if (dx == -1 && dy == 1)  return 2;   // 左上
    if (dx == 0  && dy == 1)  return 3;   // 上
    if (dx == 1  && dy == 1)  return 4;   // 右上
    if (dx == 1  && dy == 0)  return 5;   // 右
    if (dx == 1  && dy == -1) return 6;   // 右下
    if (dx == 0  && dy == -1) return 7;   // 下
    if (dx == -1 && dy == -1) return 8;   // 左下
    
    return 0;
}

cv::Point Astar::worldToMap(double world_x, double world_y) const {
    int map_x = static_cast<int>((world_x - map_origin_x_) / map_resolution_);
    int map_y = static_cast<int>((world_y - map_origin_y_) / map_resolution_);
    return cv::Point(map_x, map_y);
}

cv::Point Astar::worldToMap(const geometry_msgs::Point& point) const {
    return worldToMap(point.x, point.y);
}

geometry_msgs::Pose Astar::mapToWorldPose(double map_x, double map_y) const {
    geometry_msgs::Pose pose;
    pose.position.x = map_origin_x_ + (map_x + 0.5) * map_resolution_;
    pose.position.y = map_origin_y_ + (map_y + 0.5) * map_resolution_;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

geometry_msgs::PoseStamped Astar::mapToWorldPose(const cv::Point& map_point) const {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose = mapToWorldPose(map_point.x, map_point.y);
    return pose;
}

bool Astar::isValidPoint(const cv::Point& pt) const {
    if (!map_ready_) {
        return false;
    }
    
    if (pt.x < 0 || pt.x >= width_ || pt.y < 0 || pt.y >= height_) {
        if (config_.allow_outside_map) {
            // 允许地图外的点，但会进行调整
            return false;
        }
        return false;
    }
    
    return true;
}

bool Astar::isPointSafe(const cv::Point& pt) const {
    if (!isValidPoint(pt)) {
        return false;
    }
    
    try {
        if (inflated_map_.empty()) {
            uchar value = costmap_.at<uchar>(pt.y, pt.x);
            return (value != OBSTACLE);
        } else {
            uchar value = inflated_map_.at<uchar>(pt.y, pt.x);
            return (value != OBSTACLE);
        }
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV exception in isPointSafe: %s", e.what());
        return false;
    }
}

bool Astar::isPointSafeDetailed(const cv::Point& pt, double clearance) const {
    if (!isValidPoint(pt)) {
        return false;
    }
    
    // 检查中心点
    if (!isPointSafe(pt)) {
        return false;
    }
    
    // 如果有清除距离要求，检查周围区域
    if (clearance > 0.0) {
        int clearance_cells = static_cast<int>(clearance / map_resolution_);
        if (clearance_cells < 1) clearance_cells = 1;
        
        for (int dy = -clearance_cells; dy <= clearance_cells; dy++) {
            for (int dx = -clearance_cells; dx <= clearance_cells; dx++) {
                cv::Point check_pt(pt.x + dx, pt.y + dy);
                if (!isValidPoint(check_pt)) {
                    continue;
                }
                
                uchar value = inflated_map_.at<uchar>(check_pt.y, check_pt.x);
                if (value == OBSTACLE) {
                    // 计算距离
                    double distance = sqrt(dx*dx + dy*dy) * map_resolution_;
                    if (distance < clearance) {
                        return false;
                    }
                }
            }
        }
    }
    
    return true;
}

bool Astar::isPathSafe(const std::vector<cv::Point>& path, double clearance) const {
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
            ROS_WARN("Path segment from (%d,%d) to (%d,%d) has collision or is too close to obstacle!",
                     path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
            return false;
        }
    }
    
    return true;
}

int Astar::countUnsafePoints(const std::vector<cv::Point>& path) const {
    int unsafe_count = 0;
    for (const auto& point : path) {
        if (!isPointSafeDetailed(point, config_.safety_margin)) {
            unsafe_count++;
        }
    }
    return unsafe_count;
}

cv::Point Astar::adjustToFreePoint(const cv::Point& point, int max_radius) const {
    return adjustToSafePoint(point, 0.0);
}

cv::Point Astar::adjustToSafePoint(const cv::Point& point, double clearance) const {
    if (isValidPoint(point) && isPointSafeDetailed(point, clearance)) {
        return point;
    }
    
    // 螺旋搜索最近的安全点
    int max_radius = config_.max_search_radius;
    for (int radius = 1; radius <= max_radius; radius++) {
        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                if (dx == 0 && dy == 0) continue;
                
                cv::Point check_point(point.x + dx, point.y + dy);
                if (isValidPoint(check_point) && isPointSafeDetailed(check_point, clearance)) {
                    ROS_DEBUG("Adjusted point from (%d,%d) to (%d,%d)", 
                             point.x, point.y, check_point.x, check_point.y);
                    return check_point;
                }
            }
        }
    }
    
    ROS_WARN("Cannot find safe point near (%d,%d) within radius %d and clearance %.2f", 
             point.x, point.y, max_radius, clearance);
    return point;
}

int Astar::pointToIndex(const cv::Point& pt) const {
    return pt.y * width_ + pt.x;
}

cv::Point Astar::indexToPoint(int index) const {
    int y = index / width_;
    int x = index % width_;
    return cv::Point(x, y);
}

cv::Point Astar::getStartPoint() const {
    return start_point_;
}

cv::Point Astar::getTargetPoint() const {
    return target_point_;
}

bool Astar::isReadyForPlanning() const {
    return map_ready_ && start_ready_ && target_ready_;
}

bool Astar::isStartReady() const {
    return start_ready_;
}

bool Astar::isTargetReady() const {
    return target_ready_;
}

void Astar::reset() {
    std::lock_guard<std::mutex> lock1(map_mutex_);
    std::lock_guard<std::mutex> lock2(planning_mutex_);
    
    clearContainers();
    
    // 释放OpenCV矩阵
    costmap_.release();
    inflated_map_.release();
    
    // 重置状态
    map_ready_ = false;
    start_ready_ = false;
    target_ready_ = false;
    is_planning_ = false;
    
    // 重置路径度量
    path_metrics_ = PathMetrics();
    
    ROS_INFO("Astar planner reset");
}

void Astar::resetPlanner() {
    clearContainers();
    is_planning_ = false;
    path_metrics_ = PathMetrics();  // 重置路径度量
}

void Astar::analyzeFailureReason(const cv::Point& start, const cv::Point& target) const {
    ROS_WARN("=== Path Planning Failure Analysis ===");
    
    if (!isValidPoint(start)) {
        ROS_WARN("Start point (%d, %d) is outside map bounds [%d, %d] x [%d, %d]",
                 start.x, start.y, 0, width_ - 1, 0, height_ - 1);
    } else if (!isPointSafeDetailed(start, config_.safety_margin)) {
        ROS_WARN("Start point (%d, %d) is in obstacle or too close to obstacle (clearance: %.2f)",
                 start.x, start.y, config_.safety_margin);
    }
    
    if (!isValidPoint(target)) {
        ROS_WARN("Target point (%d, %d) is outside map bounds [%d, %d] x [%d, %d]",
                 target.x, target.y, 0, width_ - 1, 0, height_ - 1);
    } else if (!isPointSafeDetailed(target, config_.safety_margin)) {
       ROS_WARN("Target point (%d, %d) is in obstacle or too close to obstacle (clearance: %.2f)",
                         target.x, target.y, config_.safety_margin);
    }
    
    if (isValidPoint(start) && isValidPoint(target) &&
        isPointSafeDetailed(start, config_.safety_margin) &&
        isPointSafeDetailed(target, config_.safety_margin)) {
        ROS_WARN("Both points are valid and safe, but no path found.");
        ROS_WARN("Start: (%d, %d), Target: (%d, %d)", start.x, start.y, target.x, target.y);
        ROS_WARN("Map size: %dx%d, resolution: %.3f", width_, height_, map_resolution_);
        ROS_WARN("Safety margin: %.2f, Clearance: %.2f", config_.safety_margin, config_.obstacle_clearance);
    }
    
    ROS_WARN("=== End Failure Analysis ===");
}

nav_msgs::Path Astar::convertToWorldPath(const std::vector<cv::Point>& map_path) const {
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

void Astar::printDebugInfo() const {
    ROS_INFO("=== A* Debug Info ===");
    ROS_INFO("Map ready: %s", map_ready_ ? "true" : "false");
    ROS_INFO("Map size: %dx%d, resolution: %.3f", width_, height_, map_resolution_);
    ROS_INFO("Start ready: %s, point: (%d, %d)",
             start_ready_ ? "true" : "false", start_point_.x, start_point_.y);
    ROS_INFO("Target ready: %s, point: (%d, %d)",
             target_ready_ ? "true" : "false", target_point_.x, target_point_.y);
    ROS_INFO("Last planning time: %.3f seconds ago",
             (ros::Time::now() - last_planning_time_).toSec());
    ROS_INFO("Collision checks: %d, Unsafe points fixed: %d",
             path_metrics_.collision_checks, path_metrics_.unsafe_points_fixed);
}

std::string Astar::getSimpleThreeLineTable() const {
    return path_metrics_.toSimpleThreeLineTable();
}

void Astar::printMetricsTable() const {
    ROS_INFO("\n%s", getSimpleThreeLineTable().c_str());
}

}  // namespace pathplanning
