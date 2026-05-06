#ifndef CTASTAR_H
#define CTASTAR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <queue>
#include <unordered_set>
#include <cmath>

namespace pathplanning {

struct ctAstarConfig {
    double heuristic_weight = 1.0;     // 启发式权重
    int inflate_radius = 3;            // 膨胀半径
    double obstacle_clearance = 0.0;   // 障碍物清除距离
    double min_planning_interval = 0.1; // 最小规划间隔
    bool euclidean = true;             // 使用欧几里得距离
    bool allow_outside_map = false;     // 允许地图外调整
    int max_iterations = 10000;        // 最大迭代次数
    int max_search_radius = 50;        // 最大搜索半径
    bool enable_path_smoothing = false; // 启用路径平滑
    double safety_margin = 0.2;        // 安全边距
    
    ctAstarConfig() = default;
};

// 使用智能指针避免内存泄漏
struct ctNode {
    int x, y;
    double f, g, h;
    std::shared_ptr<ctNode> parent;
    
    ctNode(int x, int y) : x(x), y(y), f(0), g(0), h(0), parent(nullptr) {}
    
    // 比较函数用于优先队列
    struct Compare {
        bool operator()(const std::shared_ptr<ctNode>& a, const std::shared_ptr<ctNode>& b) {
            return a->f > b->f; // 最小堆
        }
    };
};

class ctAstar {
public:
    ctAstar();
    ~ctAstar();
    
    // 设置配置
    void setConfig(const ctAstarConfig& config);
    
    // 初始化函数
    void initAstar(double heuristic_weight = 1.0, int inflate_radius = 3, bool euclidean = true);
    
    // 地图更新
    bool updateMap(const nav_msgs::OccupancyGrid::ConstPtr& map);
    
    // 设置起点和终点
    bool setStartPoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
    bool setTargetPoint(const geometry_msgs::PoseStamped::ConstPtr& goal);
    bool setStartPoint(const cv::Point& point);
    bool setTargetPoint(const cv::Point& point);
    
    // 路径规划
    bool pathPlanning(std::vector<cv::Point>& path);
    
    // 状态查询
    bool isReadyForPlanning() const;
    cv::Point getStartPoint() const;
    cv::Point getTargetPoint() const;
    bool isStartReady() const;
    bool isTargetReady() const;
    
    // 坐标转换
    geometry_msgs::Pose mapToWorldPose(int x, int y) const;
    geometry_msgs::PoseStamped mapToWorldPoseStamped(int x, int y) const;
    nav_msgs::Path convertToWorldPath(const std::vector<cv::Point>& path_points) const;
    
    // 检查是否需要重新规划
    bool shouldReplan() const;
    
    // 获取地图信息
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    
private:
    // 地图数据
    cv::Mat costmap_;
    cv::Mat inflated_map_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    int width_;
    int height_;
    
    // 起点和目标点
    cv::Point start_point_;
    cv::Point target_point_;
    bool has_start_;
    bool has_target_;
    
    // 配置
    ctAstarConfig config_;
    
    // 上次规划时间
    ros::Time last_planning_time_;
    
    // 内部方法
    void inflateObstacles();
    double calculateHeuristic(int x1, int y1, int x2, int y2) const;
    bool isValidPoint(int x, int y) const;
    bool isFree(int x, int y) const;
    std::vector<cv::Point> getNeighbors(int x, int y) const;
    
    // 节点ID计算
    inline int getNodeId(int x, int y) const {
        return y * width_ + x;
    }
    
    // 备用路径生成
    bool generateStraightPath(std::vector<cv::Point>& path) const;
};

} // namespace pathplanning

#endif // CTASTAR_H