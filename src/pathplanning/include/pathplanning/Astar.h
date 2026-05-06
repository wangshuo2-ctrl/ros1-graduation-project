#ifndef ASTAR_H
#define ASTAR_H

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

namespace pathplanning {

struct PathMetrics {
    double length;              // 路径长度(米)
    double planning_time;       // 规划时间(秒)
    int iterations;             // 迭代次数
    int points_count;           // 路径点数
    mutable int collision_checks;       // 碰撞检查次数
    mutable int unsafe_points_fixed;    // 修复的不安全点数
    
    PathMetrics() : length(0.0), planning_time(0.0),
                    iterations(0), points_count(0),
                    collision_checks(0), unsafe_points_fixed(0) {}
    
    // 简化的表格
    std::string toSimpleThreeLineTable() const {
        std::stringstream ss;
        ss << "\n+------------------------+--------------------+\n";
        ss << "| Metric                 | Value              |\n";
        ss << "+------------------------+--------------------+\n";
        ss << "| Path Length (m)       | " << std::setw(18) << std::fixed << std::setprecision(3) << length << " |\n";
        ss << "| Planning Time (s)     | " << std::setw(18) << std::fixed << std::setprecision(3) << planning_time << " |\n";
        ss << "| Iterations            | " << std::setw(18) << iterations << " |\n";
        ss << "| Path Points           | " << std::setw(18) << points_count << " |\n";
        ss << "| Collision Checks      | " << std::setw(18) << collision_checks << " |\n";
        ss << "| Unsafe Points Fixed   | " << std::setw(18) << unsafe_points_fixed << " |\n";
        ss << "+------------------------+--------------------+\n";
        return ss.str();
    }
};

struct AstarConfig {
    double heuristic_weight = 1.0;      // 启发式权重
    double obstacle_threshold = 0.65;   // 障碍物阈值
    int inflate_radius = 8;             // 膨胀半径（从5增加到8）
    double safety_margin = 0.4;         // 安全边距（从0.3增加到0.4）
    double obstacle_clearance = 0.5;    // 障碍物清除距离（从0.3增加到0.5）
    double turn_penalty_weight = 0.1;   // 转向惩罚权重
    double min_planning_interval = 1.0; // 最小规划间隔
    int max_iterations = 20000;         // 最大迭代次数
    int max_search_radius = 20;         // 最大搜索半径
    bool euclidean = true;              // 使用欧几里得距离
    bool allow_outside_map = true;      // 允许地图外调整
    bool enable_path_smoothing = true;  // 启用路径平滑
    
    // 平滑相关参数
    bool enable_bspline_smoothing = true;  // 启用B样条平滑
    int bspline_degree = 3;                // B样条阶数
    int bspline_samples = 150;             // 采样点数（从200减少到150）
    double bspline_smoothness = 0.6;       // 平滑度（从0.7减小到0.6）
    
    // 路径重采样参数
    bool enable_path_resample = true;      // 启用路径重采样
    int resample_points = 150;             // 重采样点数
    
    // 新增：安全检查参数
    bool enable_safety_check = true;       // 启用到障碍物安全距离检查
    double safety_check_distance = 0.4;    // 安全检查距离（从0.3增加到0.4）
    int safety_check_resolution = 5;       // 安全检查分辨率
    
    // 新增：路径约束平滑参数
    bool enable_constrained_smoothing = true;  // 启用约束平滑
    int constrained_smoothing_iterations = 3;  // 约束平滑迭代次数
    double constrained_smoothing_weight = 0.2; // 约束平滑权重
    
    // 新增：曲率约束参数
    double min_turn_radius = 0.7;                  // 最小转弯半径（从0.5增加到0.7米）
    double max_curvature = 1.5;                    // 最大允许曲率（从2.0减小到1.5）
    bool enable_curvature_constraint = true;       // 启用曲率约束
    
    // 新增：二次选取策略参数
    int secondary_selection_max_iterations = 8;    // 二次选取最大迭代次数（从5增加到8）
    double secondary_selection_search_radius = 0.3;// 二次选取搜索半径（从0.2增加到0.3米）
    
    // 新增：增强安全模式
    bool enable_enhanced_safety = true;            // 启用增强安全模式
    double enhanced_safety_factor = 1.5;           // 增强安全系数（用于B样条平滑时增加安全距离）
    
    AstarConfig() = default;
};

struct Node {
    cv::Point point;
    double g_cost;      // 从起点到当前节点的实际成本
    double h_cost;      // 从当前节点到终点的启发式成本
    double f_cost;      // 总成本: f = g + h
    std::shared_ptr<Node> parent;  // 使用智能指针
    int direction;      // 移动方向
    
    Node(cv::Point p) : point(p), g_cost(0), h_cost(0), f_cost(0), 
                        parent(nullptr), direction(0) {}
    
    // 用于优先队列的比较
    bool operator<(const Node& other) const {
        return f_cost > other.f_cost;  // 最小堆
    }
    
    // 比较函数用于智能指针
    struct Compare {
        bool operator()(const std::shared_ptr<Node>& a, 
                       const std::shared_ptr<Node>& b) const {
            return a->f_cost > b->f_cost;
        }
    };
};

class Astar {
public:
    Astar();
    ~Astar();
    
    // 初始化
    void initAstar(const AstarConfig& config);
    void initAstar(double heuristic_weight, int inflate_radius, double safety_margin);
    
    // 设置配置
    void setConfig(const AstarConfig& config);
    
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
    PathMetrics getPathMetrics() const { return path_metrics_; }
    
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
    
    // 新增：曲率约束和二次选取策略函数
    bool checkCurvatureConstraint(const std::vector<cv::Point>& path, double max_curvature);
    std::vector<cv::Point> adjustControlPointsForCollision(
        const std::vector<cv::Point>& control_points, 
        int collision_index);
    double calculateSafetyScore(const cv::Point& point);
    
private:
    // 常量定义
    static const uchar UNKNOWN = 128;
    static const uchar FREE = 0;
    static const uchar OBSTACLE = 255;
    
    // 地图相关
    cv::Mat costmap_;
    cv::Mat inflated_map_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    int width_;
    int height_;
    
    // 规划相关
    cv::Point start_point_;
    cv::Point target_point_;
    bool map_ready_;
    bool start_ready_;
    bool target_ready_;
    bool is_planning_;
    ros::Time last_planning_time_;
    
    // 路径度量
    mutable PathMetrics path_metrics_;
    
    // 配置
    AstarConfig config_;
    
    // 数据结构和算法
    std::priority_queue<std::shared_ptr<Node>,
                       std::vector<std::shared_ptr<Node>>,
                       Node::Compare> open_set_;
    std::unordered_set<int> close_set_;
    std::unordered_map<int, std::shared_ptr<Node>> all_nodes_;
    
    // 互斥锁
    mutable std::mutex map_mutex_;
    mutable std::mutex planning_mutex_;
    
    // 内部方法
    void occupancyGridToMat(const nav_msgs::OccupancyGrid& grid);
    void processMap();
    void resetPlanner();
    
    // 清理函数声明
    void clearOpenSet();
    void clearNodes();
    void clearContainers();
    
    // 节点和路径处理
    std::shared_ptr<Node> findPath(const cv::Point& start, const cv::Point& target);
    void extractPath(const std::shared_ptr<Node>& end_node, std::vector<cv::Point>& path);
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
    double calculateTurnPenalty(const std::shared_ptr<Node>& current, const cv::Point& next) const;
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
};

}  // namespace pathplanning

#endif  // ASTAR_H