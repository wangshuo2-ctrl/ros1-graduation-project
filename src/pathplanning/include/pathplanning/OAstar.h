#ifndef OASTAR_H
#define OASTAR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <mutex>
#include <chrono>
#include <limits>
#include <memory>
#include <iomanip>
#include <sstream>
#include <std_srvs/SetBool.h>

namespace pathplanning {

// 路径度量结构
struct OPathMetrics {
    double length;               // 路径长度（米）
    double planning_time;        // 规划时间（秒）
    int iterations;              // 迭代次数
    int points_count;            // 路径点数
    int collision_checks;        // 碰撞检查次数
    int unsafe_points_fixed;     // 修复的不安全点数
    
    OPathMetrics() : 
        length(0.0), 
        planning_time(0.0),
        iterations(0), 
        points_count(0),
        collision_checks(0),
        unsafe_points_fixed(0) {}
    
    // 简化的表格
    std::string toSimpleThreeLineTable() const {
        std::stringstream ss;
        ss << "+---------------------+-------------------+\n";
        ss << "| Metric              | Value             |\n";
        ss << "+---------------------+-------------------+\n";
        ss << "| Path Length (m)     | " << std::setw(18) << std::fixed << std::setprecision(3) 
           << length << "|\n";
        ss << "| Planning Time (s)   | " << std::setw(18) << std::fixed << std::setprecision(3) 
           << planning_time << "|\n";
        ss << "| Iterations          | " << std::setw(18) << iterations << "|\n";
        ss << "| Path Points         | " << std::setw(18) << points_count << "|\n";
        ss << "| Collision Checks    | " << std::setw(18) << collision_checks << "|\n";
        ss << "| Unsafe Points Fixed | " << std::setw(18) << unsafe_points_fixed << "|\n";
        ss << "+---------------------+-------------------+\n";
        return ss.str();
    }
};

// OAstar配置结构
struct OAstarConfig {
    // 基本参数
    double heuristic_weight = 1.0;           // 启发式权重
    double obstacle_threshold = 0.65;        // 障碍物阈值
    int inflate_radius = 8;                  // 膨胀半径
    double safety_margin = 0.4;              // 安全边距
    double obstacle_clearance = 0.5;         // 障碍物清除距离
    double turn_penalty_weight = 0.1;        // 转向惩罚权重
    int max_iterations = 20000;              // 最大迭代次数
    int max_search_radius = 20;              // 最大搜索半径
    bool euclidean = true;                   // 使用欧几里得距离
    bool allow_outside_map = true;           // 允许地图外调整
    
    // 平滑相关参数
    bool enable_path_smoothing = true;       // 启用路径平滑
    bool enable_bspline_smoothing = true;    // 启用B样条平滑
    int bspline_degree = 3;                  // B样条阶数
    int bspline_samples = 150;               // 采样点数
    double bspline_smoothness = 0.6;         // 平滑度
    
    // 路径重采样参数
    bool enable_path_resample = true;        // 启用路径重采样
    int resample_points = 150;               // 重采样点数
    
    // 安全检查参数
    bool enable_safety_check = true;         // 启用到障碍物安全距离检查
    double safety_check_distance = 0.4;      // 安全检查距离
    int safety_check_resolution = 5;         // 安全检查分辨率
    
    // 路径约束平滑参数
    bool enable_constrained_smoothing = true;  // 启用约束平滑
    int constrained_smoothing_iterations = 3;  // 约束平滑迭代次数
    double constrained_smoothing_weight = 0.2; // 约束平滑权重
    
    // 曲率约束参数
    double min_turn_radius = 0.7;            // 最小转弯半径
    double max_curvature = 1.5;              // 最大曲率
    bool enable_curvature_constraint = true; // 启用曲率约束
    
    // 二次选取策略参数
    int secondary_selection_max_iterations = 8;  // 二次选取最大迭代次数
    double secondary_selection_search_radius = 0.3;  // 二次选取搜索半径
    
    // 新增：忽略动态障碍物参数
    bool use_original_map_only = true;       // 只使用原始地图，忽略动态障碍物
    bool ignore_dynamic_obstacles = true;    // 忽略动态障碍物
    bool lock_original_map = true;           // 锁定原始地图
    double enhanced_safety_factor = 1.5;     // 增强安全系数
};

// 节点结构
struct ONode {
    cv::Point point;                         // 节点坐标
    double g_cost;                           // 从起点到当前节点的实际成本
    double h_cost;                           // 从当前节点到终点的启发式成本
    double f_cost;                           // 总成本：f = g + h
    std::shared_ptr<ONode> parent;           // 父节点指针
    int direction;                           // 移动方向
    
    ONode(cv::Point p) : 
        point(p), 
        g_cost(0), 
        h_cost(0), 
        f_cost(0), 
        parent(nullptr), 
        direction(0) {}
    
    // 用于优先队列的比较
    bool operator<(const ONode& other) const {
        return f_cost > other.f_cost;  // 最小堆
    }
    
    // 比较函数用于智能指针
    struct Compare {
        bool operator()(const std::shared_ptr<ONode>& a,
                       const std::shared_ptr<ONode>& b) const {
            return a->f_cost > b->f_cost;
        }
    };
};

// 独立完整的OAstar类
class OAstar {
public:
    OAstar();
    ~OAstar();
    
    // 初始化
    void initOAstar(const OAstarConfig& config);
    void initOAstar(double heuristic_weight, int inflate_radius, double safety_margin);
    
    // 设置配置
    void setConfig(const OAstarConfig& config);
    
    // 地图更新
    void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& grid);
    
    // 设置起点和终点
    void setStartPoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void setTargetPoint(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setStartPoint(cv::Point point);
    void setTargetPoint(cv::Point point);
    
    // 路径规划
    bool pathPlanning(std::vector<cv::Point>& path);
    bool pathPlanning(cv::Point start_point, cv::Point target_point, std::vector<cv::Point>& path);
    
    // 获取路径度量指标
    OPathMetrics getPathMetrics() const { return path_metrics_; }
    
    // 获取三线表格字符串
    std::string getSimpleThreeLineTable() const;
    
    // 打印度量表格
    void printMetricsTable() const;
    
    // 状态查询
    bool isReadyForPlanning() const;
    bool isStartReady() const;
    bool isTargetReady() const;
    
    // 获取点
    cv::Point getStartPoint() const;
    cv::Point getTargetPoint() const;
    
    // 重置
    void reset();
    
    // 坐标转换
    cv::Point worldToMap(double world_x, double world_y) const;
    cv::Point worldToMap(const geometry_msgs::Point& point) const;
    geometry_msgs::Pose mapToWorldPose(double map_x, double map_y) const;
    geometry_msgs::PoseStamped mapToWorldPose(const cv::Point& map_point) const;
    
    // 路径发布
    nav_msgs::Path convertToWorldPath(const std::vector<cv::Point>& map_path) const;
    
    // 调试信息
    void printDebugInfo() const;
    
    // 控制动态障碍物忽略
    void setIgnoreDynamicObstacles(bool ignore) { ignore_dynamic_obstacles_ = ignore; }
    void setLockOriginalMap(bool lock) { lock_original_map_ = lock; }
    void resetFirstPlan() { first_plan_completed_ = false; first_plan_path_.clear(); }
    
    // 曲率约束和二次选取策略函数
    bool checkCurvatureConstraint(const std::vector<cv::Point>& path, double max_curvature);
    std::vector<cv::Point> adjustControlPointsForCollision(
        const std::vector<cv::Point>& control_points,
        int collision_index);
    double calculateSafetyScore(const cv::Point& point);
    
private:
    // 常量定义
    static const uchar O_UNKNOWN = 128;
    static const uchar O_FREE = 0;
    static const uchar O_OBSTACLE = 255;
    
    // 地图相关
    cv::Mat o_costmap_;
    cv::Mat o_inflated_map_;
    double o_map_resolution_;
    double o_map_origin_x_;
    double o_map_origin_y_;
    int o_width_;
    int o_height_;
    
    // 原始地图存储
    nav_msgs::OccupancyGrid original_map_;
    bool original_map_initialized_;
    
    // 规划相关
    cv::Point o_start_point_;
    cv::Point o_target_point_;
    bool o_map_ready_;
    bool o_start_ready_;
    bool o_target_ready_;
    bool o_is_planning_;
    ros::Time o_last_planning_time_;
    
    // 动态障碍物忽略控制
    bool ignore_dynamic_obstacles_;  // 是否忽略动态障碍物
    bool lock_original_map_;         // 是否锁定原始地图
    bool first_plan_completed_;      // 第一次规划是否完成
    std::vector<cv::Point> first_plan_path_;  // 第一次规划的路径
    
    // 路径度量
    mutable OPathMetrics path_metrics_;
    
    // 配置
    OAstarConfig o_config_;
    
    // 数据结构和算法
    std::priority_queue<std::shared_ptr<ONode>,
                        std::vector<std::shared_ptr<ONode>>,
                        ONode::Compare> o_open_set_;
    std::unordered_set<int> o_close_set_;
    std::unordered_map<int, std::shared_ptr<ONode>> o_all_nodes_;
    
    // 互斥锁
    mutable std::mutex o_map_mutex_;
    mutable std::mutex o_planning_mutex_;
    
    // 内部方法
    void occupancyGridToMat(const nav_msgs::OccupancyGrid& grid);
    void processMap();
    void resetPlanner();
    
    // 清理函数声明
    void clearOpenSet();
    void clearNodes();
    void clearContainers();
    
    // 节点和路径处理
    std::shared_ptr<ONode> findPath(const cv::Point& start, const cv::Point& target);
    void extractPath(const std::shared_ptr<ONode>& end_node, std::vector<cv::Point>& path);
    void smoothPath(std::vector<cv::Point>& path);
    void simplifyPath(std::vector<cv::Point>& path);
    
    // 平滑函数
    void bsplineSmooth(std::vector<cv::Point>& path);
    void constrainedSmoothPath(std::vector<cv::Point>& path);
    std::vector<cv::Point> resamplePath(const std::vector<cv::Point>& path, int num_points);
    
    // 计算B样条
    std::vector<std::pair<double, double>> computeBSpline(const std::vector<cv::Point>& control_points,
                                                          int degree, int samples);
    std::vector<double> computeKnotVector(int n, int p);
    double bsplineBasis(int i, int p, double t, const std::vector<double>& knots);
    
    // 距离计算
    double calculateHeuristic(const cv::Point& a, const cv::Point& b) const;
    double calculateTurnPenalty(const std::shared_ptr<ONode>& current, const cv::Point& next) const;
    double calculateDistance(const cv::Point& p1, const cv::Point& p2) const;
    
    // 邻居和方向
    std::vector<cv::Point> getValidNeighbors(const cv::Point& current) const;
    std::vector<cv::Point> getAllNeighbors(const cv::Point& current) const;
    int getDirection(const cv::Point& from, const cv::Point& to) const;
    
    // 安全性和连通性检查
    bool isValidPoint(const cv::Point& pt) const;
    bool isPointSafe(const cv::Point& pt) const;
    bool isPointSafeDetailed(const cv::Point& pt, double clearance = 0.0) const;
    bool checkCollision(const cv::Point& pt1, const cv::Point& pt2) const;
    bool hasLineOfSight(const cv::Point& a, const cv::Point& b) const;
    bool checkDiagonalSafety(const cv::Point& from, const cv::Point& to) const;
    bool isPathSafe(const std::vector<cv::Point>& path, double clearance = 0.0) const;
    
    // 安全检查函数
    bool checkPathSegmentSafety(const cv::Point& p1, const cv::Point& p2, double clearance = 0.0) const;
    int countUnsafePoints(const std::vector<cv::Point>& path) const;
    
    // 点调整
    cv::Point adjustToFreePoint(const cv::Point& point, int max_radius) const;
    cv::Point adjustToSafePoint(const cv::Point& point, double clearance) const;
    
    // 索引转换
    int pointToIndex(const cv::Point& pt) const;
    cv::Point indexToPoint(int index) const;
    
    // 调试
    void analyzeFailureReason(const cv::Point& start, const cv::Point& target) const;
    
    // 原始地图处理
    void processOriginalMap(const nav_msgs::OccupancyGrid& map);
    void saveOriginalPath(const std::vector<cv::Point>& path);
};

}  // namespace pathplanning

#endif  // OASTAR_H