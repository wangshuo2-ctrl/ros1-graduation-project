#include "pathplanning/ctAstar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace pathplanning {

ctAstar::ctAstar() : 
    has_start_(false), 
    has_target_(false),
    resolution_(0.05),
    origin_x_(0.0),
    origin_y_(0.0),
    width_(0),
    height_(0)
{
    ROS_INFO("ctAstar constructor called");
    costmap_ = cv::Mat::zeros(0, 0, CV_8UC1);
    inflated_map_ = cv::Mat::zeros(0, 0, CV_8UC1);
    last_planning_time_ = ros::Time(0);
}

ctAstar::~ctAstar() {
    ROS_INFO("ctAstar destructor called");
    // 不需要手动清理，智能指针会自动管理
}

void ctAstar::setConfig(const ctAstarConfig& config) {
    config_ = config;
    ROS_INFO("ctAstar config updated");
}

void ctAstar::initAstar(double heuristic_weight, int inflate_radius, bool euclidean) {
    config_.heuristic_weight = heuristic_weight;
    config_.inflate_radius = inflate_radius;
    config_.euclidean = euclidean;
    ROS_INFO("Astar initialized with heuristic_weight: %.2f, inflate_radius: %d, euclidean: %d", 
             heuristic_weight, inflate_radius, euclidean);
}

bool ctAstar::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    if (!map) {
        ROS_ERROR("Null map pointer received");
        return false;
    }
    
    if (map->info.width <= 0 || map->info.height <= 0) {
        ROS_ERROR("Invalid map dimensions: %dx%d", map->info.width, map->info.height);
        return false;
    }
    
    resolution_ = map->info.resolution;
    origin_x_ = map->info.origin.position.x;
    origin_y_ = map->info.origin.position.y;
    width_ = map->info.width;
    height_ = map->info.height;
    
    ROS_INFO("Updating map: %dx%d, resolution: %.3f, origin: (%.2f, %.2f)", 
             width_, height_, resolution_, origin_x_, origin_y_);
    
    // 重新分配地图
    costmap_ = cv::Mat::zeros(height_, width_, CV_8UC1);
    inflated_map_ = cv::Mat::zeros(height_, width_, CV_8UC1);
    
    // 检查数据大小
    if (map->data.size() != static_cast<size_t>(width_ * height_)) {
        ROS_ERROR("Map data size mismatch: expected %d, got %zu", 
                 width_ * height_, map->data.size());
        return false;
    }
    
    // 安全地填充成本地图
    try {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                int index = y * width_ + x;
                if (index >= 0 && index < static_cast<int>(map->data.size())) {
                    int8_t value = map->data[index];
                    if (value > 50) { // 障碍物
                        costmap_.at<uchar>(y, x) = 255;
                    }
                }
            }
        }
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV exception while filling costmap: %s", e.what());
        return false;
    }
    
    // 膨胀障碍物
    inflateObstacles();
    
    ROS_INFO("Map updated successfully. Inflated radius: %d", config_.inflate_radius);
    return true;
}

void ctAstar::inflateObstacles() {
    if (config_.inflate_radius <= 0) {
        costmap_.copyTo(inflated_map_);
        return;
    }
    
    try {
        int radius = config_.inflate_radius;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                   cv::Size(2*radius+1, 2*radius+1));
        cv::dilate(costmap_, inflated_map_, kernel);
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV exception while inflating obstacles: %s", e.what());
        costmap_.copyTo(inflated_map_);
    }
}

bool ctAstar::setStartPoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
    if (!start) {
        ROS_ERROR("Null start pointer received");
        return false;
    }
    
    double world_x = start->pose.pose.position.x;
    double world_y = start->pose.pose.position.y;
    
    // 世界坐标转地图坐标
    int mx = static_cast<int>((world_x - origin_x_) / resolution_);
    int my = static_cast<int>((world_y - origin_y_) / resolution_);
    
    return setStartPoint(cv::Point(mx, my));
}

bool ctAstar::setStartPoint(const cv::Point& point) {
    // 确保在地图范围内
    if (!isValidPoint(point.x, point.y)) {
        ROS_WARN("Start point (%d, %d) is outside map bounds [0-%d, 0-%d]", 
                 point.x, point.y, width_-1, height_-1);
        if (!config_.allow_outside_map) {
            return false;
        }
        int mx = std::max(0, std::min(point.x, width_ - 1));
        int my = std::max(0, std::min(point.y, height_ - 1));
        start_point_ = cv::Point(mx, my);
    } else {
        start_point_ = point;
    }
    
    has_start_ = true;
    
    ROS_INFO("Start point set to: (%d, %d)", start_point_.x, start_point_.y);
    return true;
}

bool ctAstar::setTargetPoint(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    if (!goal) {
        ROS_ERROR("Null goal pointer received");
        return false;
    }
    
    double world_x = goal->pose.position.x;
    double world_y = goal->pose.position.y;
    
    // 世界坐标转地图坐标
    int mx = static_cast<int>((world_x - origin_x_) / resolution_);
    int my = static_cast<int>((world_y - origin_y_) / resolution_);
    
    return setTargetPoint(cv::Point(mx, my));
}

bool ctAstar::setTargetPoint(const cv::Point& point) {
    // 确保在地图范围内
    if (!isValidPoint(point.x, point.y)) {
        ROS_WARN("Target point (%d, %d) is outside map bounds [0-%d, 0-%d]", 
                 point.x, point.y, width_-1, height_-1);
        if (!config_.allow_outside_map) {
            return false;
        }
        int mx = std::max(0, std::min(point.x, width_ - 1));
        int my = std::max(0, std::min(point.y, height_ - 1));
        target_point_ = cv::Point(mx, my);
    } else {
        target_point_ = point;
    }
    
    has_target_ = true;
    
    ROS_INFO("Target point set to: (%d, %d)", target_point_.x, target_point_.y);
    return true;
}

bool ctAstar::isReadyForPlanning() const {
    return has_start_ && has_target_;
}

cv::Point ctAstar::getStartPoint() const {
    return start_point_;
}

cv::Point ctAstar::getTargetPoint() const {
    return target_point_;
}

bool ctAstar::isStartReady() const {
    return has_start_;
}

bool ctAstar::isTargetReady() const {
    return has_target_;
}

bool ctAstar::shouldReplan() const {
    if (!has_start_ || !has_target_) {
        return false;
    }
    
    ros::Time now = ros::Time::now();
    if ((now - last_planning_time_).toSec() < config_.min_planning_interval) {
        return false;
    }
    
    return true;
}

bool ctAstar::isValidPoint(int x, int y) const {
    if (width_ <= 0 || height_ <= 0) {
        return false;
    }
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool ctAstar::isFree(int x, int y) const {
    if (!isValidPoint(x, y)) {
        return false;
    }
    
    // 安全访问OpenCV矩阵
    try {
        return inflated_map_.at<uchar>(y, x) == 0; // 0 = 自由空间
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV exception while checking if point (%d, %d) is free: %s", 
                 x, y, e.what());
        return false;
    }
}

double ctAstar::calculateHeuristic(int x1, int y1, int x2, int y2) const {
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    
    if (config_.euclidean) {
        return std::sqrt(dx*dx + dy*dy);
    } else {
        return dx + dy; // 曼哈顿距离
    }
}

std::vector<cv::Point> ctAstar::getNeighbors(int x, int y) const {
    std::vector<cv::Point> neighbors;
    
    // 8方向邻居
    int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    for (int i = 0; i < 8; i++) {
        int nx = x + dx8[i];
        int ny = y + dy8[i];
        
        if (isValidPoint(nx, ny) && isFree(nx, ny)) {
            neighbors.push_back(cv::Point(nx, ny));
        }
    }
    
    return neighbors;
}

bool ctAstar::pathPlanning(std::vector<cv::Point>& path) {
    ROS_INFO("Starting A* path planning from (%d, %d) to (%d, %d)", 
             start_point_.x, start_point_.y, target_point_.x, target_point_.y);
    
    path.clear();
    
    // 检查起点和终点是否有效
    if (!isValidPoint(start_point_.x, start_point_.y)) {
        ROS_ERROR("Start point (%d, %d) is invalid", start_point_.x, start_point_.y);
        return false;
    }
    
    if (!isValidPoint(target_point_.x, target_point_.y)) {
        ROS_ERROR("Target point (%d, %d) is invalid", target_point_.x, target_point_.y);
        return false;
    }
    
    // 检查起点和终点是否自由
    if (!isFree(start_point_.x, start_point_.y)) {
        ROS_ERROR("Start point (%d, %d) is in obstacle!", start_point_.x, start_point_.y);
        return false;
    }
    
    if (!isFree(target_point_.x, target_point_.y)) {
        ROS_ERROR("Target point (%d, %d) is in obstacle!", target_point_.x, target_point_.y);
        return false;
    }
    
    // 如果起点和目标点相同
    if (start_point_.x == target_point_.x && start_point_.y == target_point_.y) {
        path.push_back(start_point_);
        ROS_INFO("Start and target are the same point");
        return true;
    }
    
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 创建开放集和关闭集
    std::priority_queue<std::shared_ptr<ctNode>, 
                       std::vector<std::shared_ptr<ctNode>>, 
                       ctNode::Compare> open_set;
    
    std::unordered_map<int, std::shared_ptr<ctNode>> all_nodes;
    std::unordered_set<int> closed_set;
    
    // 创建起点节点
    auto start_node = std::make_shared<ctNode>(start_point_.x, start_point_.y);
    start_node->g = 0;
    start_node->h = calculateHeuristic(start_point_.x, start_point_.y, 
                                      target_point_.x, target_point_.y);
    start_node->f = start_node->g + config_.heuristic_weight * start_node->h;
    
    open_set.push(start_node);
    all_nodes[getNodeId(start_point_.x, start_point_.y)] = start_node;
    
    int iterations = 0;
    bool found_path = false;
    std::shared_ptr<ctNode> current_node = nullptr;
    
    while (!open_set.empty() && iterations < config_.max_iterations) {
        iterations++;
        
        // 获取f值最小的节点
        current_node = open_set.top();
        open_set.pop();
        
        int current_id = getNodeId(current_node->x, current_node->y);
        
        // 如果已经在关闭集中，跳过
        if (closed_set.find(current_id) != closed_set.end()) {
            continue;
        }
        
        // 添加到关闭集
        closed_set.insert(current_id);
        
        // 检查是否到达目标
        if (current_node->x == target_point_.x && current_node->y == target_point_.y) {
            found_path = true;
            break;
        }
        
        // 获取邻居
        auto neighbors = getNeighbors(current_node->x, current_node->y);
        
        for (const auto& neighbor : neighbors) {
            int nx = neighbor.x;
            int ny = neighbor.y;
            int neighbor_id = getNodeId(nx, ny);
            
            // 如果已经在关闭集中，跳过
            if (closed_set.find(neighbor_id) != closed_set.end()) {
                continue;
            }
            
            // 计算移动代价（直线1.0，对角线1.414）
            double move_cost = 1.0;
            if (std::abs(nx - current_node->x) == 1 && std::abs(ny - current_node->y) == 1) {
                move_cost = 1.414; // 对角线
            }
            
            double tentative_g = current_node->g + move_cost;
            
            // 检查是否在节点映射中
            auto it = all_nodes.find(neighbor_id);
            if (it != all_nodes.end()) {
                auto neighbor_node = it->second;
                
                // 如果新路径更优，更新
                if (tentative_g < neighbor_node->g) {
                    neighbor_node->g = tentative_g;
                    neighbor_node->h = calculateHeuristic(nx, ny, 
                                                         target_point_.x, target_point_.y);
                    neighbor_node->f = neighbor_node->g + config_.heuristic_weight * neighbor_node->h;
                    neighbor_node->parent = current_node;
                }
            } else {
                // 创建新节点
                auto neighbor_node = std::make_shared<ctNode>(nx, ny);
                neighbor_node->g = tentative_g;
                neighbor_node->h = calculateHeuristic(nx, ny, 
                                                     target_point_.x, target_point_.y);
                neighbor_node->f = neighbor_node->g + config_.heuristic_weight * neighbor_node->h;
                neighbor_node->parent = current_node;
                
                all_nodes[neighbor_id] = neighbor_node;
                open_set.push(neighbor_node);
            }
        }
    }
    
    // 记录结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_duration = end_time - start_time;
    double planning_time = planning_duration.count();
    
    if (found_path && current_node) {
        // 重建路径
        std::shared_ptr<ctNode> node = current_node;
        while (node) {
            path.push_back(cv::Point(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        
        // 计算路径统计信息
        int path_points = path.size();
        double path_length = 0.0;
        double total_curvature = 0.0;
        int curvature_points = 0;
        
        // 计算路径长度和曲率
        if (path_points >= 2) {
            for (size_t i = 0; i < path.size() - 1; ++i) {
                double dx = (path[i+1].x - path[i].x) * resolution_;
                double dy = (path[i+1].y - path[i].y) * resolution_;
                path_length += std::sqrt(dx*dx + dy*dy);
            }
            
            // 计算曲率（连续三个点形成的角度变化）
            if (path_points >= 3) {
                for (size_t i = 1; i < path.size() - 1; ++i) {
                    cv::Point prev = path[i-1];
                    cv::Point curr = path[i];
                    cv::Point next = path[i+1];
                    
                    // 计算两个向量
                    double dx1 = (curr.x - prev.x) * resolution_;
                    double dy1 = (curr.y - prev.y) * resolution_;
                    double dx2 = (next.x - curr.x) * resolution_;
                    double dy2 = (next.y - curr.y) * resolution_;
                    
                    // 计算向量长度
                    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    
                    if (len1 > 0.0 && len2 > 0.0) {
                        // 计算点积
                        double dot = dx1*dx2 + dy1*dy2;
                        // 计算角度（弧度）
                        double cos_angle = dot / (len1 * len2);
                        // 防止浮点误差
                        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
                        double angle = std::acos(cos_angle);
                        
                        // 计算曲率（角度变化 / 平均长度）
                        double avg_length = (len1 + len2) / 2.0;
                        if (avg_length > 0.0) {
                            total_curvature += angle / avg_length;
                            curvature_points++;
                        }
                    }
                }
            }
        }
        
        // 计算平均曲率
        double avg_curvature = curvature_points > 0 ? total_curvature / curvature_points : 0.0;
        
        // 输出三线表格
        ROS_INFO("+---------------------------------------+");
        ROS_INFO("|       A* PATH PLANNING RESULTS        |");
        ROS_INFO("+-------------------+-------------------+");
        ROS_INFO("| Metric            | Value             |");
        ROS_INFO("+-------------------+-------------------+");
        ROS_INFO("| Path Length       | %6.3f m         |", path_length);
        ROS_INFO("| Planning Time     | %6.3f ms        |", planning_time * 1000.0);
        ROS_INFO("| Path Curvature    | %6.3f rad/m     |", avg_curvature);
        ROS_INFO("| Path Points       | %6d points      |", path_points);
        ROS_INFO("| Iterations        | %6d iterations  |", iterations);
        ROS_INFO("+-------------------+-------------------+");
        
        // 更新上次规划时间
        last_planning_time_ = ros::Time::now();
        
        return true;
    } else {
        ROS_WARN("A* planning failed after %d iterations. Time: %.3f s", 
                 iterations, planning_time);
        
        // 尝试生成直线路径
        if (generateStraightPath(path)) {
            // 计算直线路径的统计信息
            int path_points = path.size();
            double path_length = 0.0;
            
            if (path_points >= 2) {
                for (size_t i = 0; i < path.size() - 1; ++i) {
                    double dx = (path[i+1].x - path[i].x) * resolution_;
                    double dy = (path[i+1].y - path[i].y) * resolution_;
                    path_length += std::sqrt(dx*dx + dy*dy);
                }
            }
            
            // 输出三线表格（直线路径）
            ROS_INFO("+---------------------------------------+");
            ROS_INFO("|  STRAIGHT LINE FALLBACK PATH RESULTS  |");
            ROS_INFO("+-------------------+-------------------+");
            ROS_INFO("| Metric            | Value             |");
            ROS_INFO("+-------------------+-------------------+");
            ROS_INFO("| Path Length       | %6.3f m         |", path_length);
            ROS_INFO("| Planning Time     | %6.3f ms        |", planning_time * 1000.0);
            ROS_INFO("| Path Curvature    |     0.000 rad/m  |");  // 直线曲率为0
            ROS_INFO("| Path Points       | %6d points      |", path_points);
            ROS_INFO("| Iterations        | %6d iterations  |", iterations);
            ROS_INFO("+-------------------+-------------------+");
            
            last_planning_time_ = ros::Time::now();
            return true;
        }
        
        return false;
    }
}

bool ctAstar::generateStraightPath(std::vector<cv::Point>& path) const {
    path.clear();
    
    // 计算距离
    int dx = target_point_.x - start_point_.x;
    int dy = target_point_.y - start_point_.y;
    float distance = sqrt(dx*dx + dy*dy);
    
    // 生成路径点
    int steps = std::max(20, static_cast<int>(distance));
    
    for (int i = 0; i <= steps; i++) {
        float t = static_cast<float>(i) / steps;
        int x = static_cast<int>(start_point_.x + t * dx);
        int y = static_cast<int>(start_point_.y + t * dy);
        
        // 确保在地图范围内
        x = std::max(0, std::min(x, width_ - 1));
        y = std::max(0, std::min(y, height_ - 1));
        
        // 检查是否为障碍物
        if (!isFree(x, y)) {
            ROS_WARN("Straight line path blocked at point (%d, %d)", x, y);
            return false;
        }
        
        path.push_back(cv::Point(x, y));
    }
    
    ROS_INFO("Generated straight line with %zu points", path.size());
    return true;
}

geometry_msgs::Pose ctAstar::mapToWorldPose(int x, int y) const {
    geometry_msgs::Pose pose;
    
    // 使用单元格中心
    pose.position.x = origin_x_ + (x + 0.5) * resolution_;
    pose.position.y = origin_y_ + (y + 0.5) * resolution_;
    pose.position.z = 0.0;
    
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    
    return pose;
}

geometry_msgs::PoseStamped ctAstar::mapToWorldPoseStamped(int x, int y) const {
    geometry_msgs::PoseStamped pose_stamped;
    
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = mapToWorldPose(x, y);
    
    return pose_stamped;
}

nav_msgs::Path ctAstar::convertToWorldPath(const std::vector<cv::Point>& path_points) const {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    
    for (const auto& point : path_points) {
        geometry_msgs::PoseStamped pose = mapToWorldPoseStamped(point.x, point.y);
        path_msg.poses.push_back(pose);
    }
    
    return path_msg;
}

} // namespace pathplanning