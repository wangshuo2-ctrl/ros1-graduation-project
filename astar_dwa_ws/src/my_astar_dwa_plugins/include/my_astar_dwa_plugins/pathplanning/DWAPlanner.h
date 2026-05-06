#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <vector>
#include <memory>

namespace pathplanning {

struct DWAParams {
    // 速度限制
    double max_vel_x = 0.22;             // 最大线速度
    double min_vel_x = 0.0;              // 最小线速度
    double max_vel_x_backwards = -0.1;   // 最大后退速度
    double max_vel_theta = 2.75;         // 最大角速度
    double min_vel_theta = -2.75;        // 最小角速度
    
    // 加速度限制
    double acc_lim_x = 2.5;              // 线加速度限制
    double acc_lim_theta = 3.2;          // 角加速度限制
    
    // 采样参数
    int vx_samples = 20;                 // 线速度采样数
    int vth_samples = 40;                // 角速度采样数
    
    // 模拟参数
    double sim_time = 1.5;               // 模拟时间
    double dt = 0.1;                     // 模拟步长
    double sim_granularity = 0.025;      // 模拟粒度
    double angular_sim_granularity = 0.025;  // 角度模拟粒度
    
    // 目标容差
    double xy_goal_tolerance = 0.15;     // XY目标容差
    double yaw_goal_tolerance = 0.17;    // 航向目标容差
    
    // 代价权重
    double goal_distance_bias = 20.0;    // 目标距离权重
    double path_distance_bias = 32.0;    // 路径距离权重
    double obstacle_cost_bias = 1.0;     // 障碍物代价权重
    double forward_point_bias = 0.8;     // 前向点权重
    double occdist_scale = 0.02;         // 障碍物距离缩放
    double obstacle_cutoff_dist = 2.5;   // 障碍物截止距离
    
    // 控制器参数
    double controller_frequency = 10.0;  // 控制频率
    double oscillation_reset_dist = 0.05; // 振荡重置距离
    double oscillation_reset_angle = 0.2; // 振荡重置角度
    
    // 机器人参数
    double robot_radius = 0.25;          // 机器人半径
    double inflation_radius = 0.5;       // 膨胀半径
    double max_obstacle_dist = 2.5;      // 最大障碍物距离
    
    // 代价函数参数
    double path_distance_bias_front = 0.5;  // 前向路径距离权重
    double goal_distance_bias_front = 1.0;  // 前向目标距离权重
    double occdist_scale_front = 0.3;       // 前向障碍物距离缩放
    
    // 停滞检测
    double oscillation_timeout = 2.0;    // 振荡超时
    double forward_point_distance = 1.5; // 前向点距离
    
    // 代价地图参数
    double inscribed_radius = 0.2;       // 内切半径
    double circumscribed_radius = 0.3;   // 外接半径
    int lethal_cost = 100;               // 致命代价
    int obstacle_threshold = 60;         // 障碍物阈值
    
    // 新增：全局路径方向角评价函数的权重
    double global_heading_bias = 0.5;    // 全局航向权重
    
    // 新增：平滑控制参数
    double smoothness_weight = 0.3;      // 平滑度权重
    double velocity_continuity_weight = 0.2;  // 速度连续性权重
    double max_angular_acceleration = 1.5;    // 最大角加速度
    
    // 新增：路径跟随参数
    double lookahead_distance = 1.0;     // 前瞻距离
    double goal_angle_bias = 0.3;        // 目标角度偏差权重
    double path_angle_bias = 0.2;        // 路径角度偏差权重
    double smoothing_factor = 0.8;       // 平滑因子
    
    // 新增：自适应参数
    bool enable_adaptive_vel = true;     // 启用自适应速度
    double min_safe_distance = 0.3;      // 最小安全距离
    double adaptive_speed_factor = 0.7;  // 自适应速度因子
};

class DWAPlanner {
public:
    DWAPlanner();
    virtual ~DWAPlanner() = default;
    
    // 初始化
    void initialize();
    bool isInitialized() const;
    void reset();
    
    // 设置全局路径
    void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
    
    // 设置/更新代价地图
    void setCostMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap);
    void updateCostMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap);
    
    // 主控制函数
    geometry_msgs::Twist computeVelocityCommands(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::Twist& robot_velocity);
    
    // 目标到达检查
    bool isGoalReached(const geometry_msgs::PoseStamped& robot_pose);
    
    // 设置参数
    void setParams(const DWAParams& params) { params_ = params; }
    DWAParams getParams() const { return params_; }
    
    // 轨迹结构体
    struct Trajectory {
        std::vector<geometry_msgs::PoseStamped> path;
        double xv, yv, thetav;
        double cost;
        bool valid;
        
        Trajectory() : xv(0.0), yv(0.0), thetav(0.0), cost(0.0), valid(true) {}
    };
    
private:
    DWAParams params_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped current_goal_;
    
    // 代价地图相关
    nav_msgs::OccupancyGrid costmap_;
    bool costmap_initialized_;
    double costmap_resolution_;
    double costmap_origin_x_;
    double costmap_origin_y_;
    int costmap_width_;
    int costmap_height_;
    
    // 状态标志
    bool initialized_;
    bool goal_reached_;
    ros::Time last_plan_time_;
    ros::Time last_valid_plan_time_;
    
    // 振荡检测
    ros::Time oscillation_start_time_;
    geometry_msgs::PoseStamped oscillation_pose_;
    
    // 辅助函数
    void calculateDynamicWindow(const geometry_msgs::Twist& robot_velocity,
                               double& min_vel_x, double& max_vel_x,
                               double& min_vel_theta, double& max_vel_theta);
    
    std::vector<Trajectory> generateTrajectories(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::Twist& robot_velocity);
    
    double scoreTrajectory(const Trajectory& traj,
                          const geometry_msgs::PoseStamped& robot_pose);
    
    Trajectory findBestTrajectory(const std::vector<Trajectory>& trajectories,
                                 const geometry_msgs::PoseStamped& robot_pose);
    
    // 障碍物相关函数
    double getObstacleCost(const geometry_msgs::PoseStamped& pose) const;
    double getObstacleDistance(const geometry_msgs::PoseStamped& pose) const;
    bool isTrajectoryColliding(const Trajectory& traj) const;
    
    // 坐标转换函数
    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    double mapToWorldX(int mx) const;
    double mapToWorldY(int my) const;
    
    // 距离计算函数
    double getGoalDistance(const geometry_msgs::PoseStamped& pose) const;
    double getPathDistance(const geometry_msgs::PoseStamped& pose) const;
    
    // 新增：辅助函数 - 查找全局路径上距离机器人最近的点
    bool findNearestPointOnGlobalPath(const geometry_msgs::PoseStamped& robot_pose,
                                     size_t& nearest_idx, double& nearest_dist) const;
    
    // 新增：计算当前位姿与全局路径方向的角度差
    double calculateGlobalHeadingAngle(const geometry_msgs::PoseStamped& robot_pose) const;
    
    // 新增：计算轨迹平滑度
    double calculateTrajectorySmoothness(const Trajectory& traj) const;
    
    // 新增：计算速度连续性
    double calculateVelocityContinuity(const Trajectory& traj, 
                                      const geometry_msgs::Twist& current_vel) const;
    
    // 新增：自适应速度控制
    double calculateAdaptiveVelocity(const geometry_msgs::PoseStamped& robot_pose) const;
    
    // 新增：路径角度跟踪
    double calculatePathAngleError(const geometry_msgs::PoseStamped& robot_pose) const;
    
    // 辅助几何函数
    double distance(double x1, double y1, double x2, double y2) const;
    double normalizeAngle(double angle) const;
    
    // 振荡检测
    void resetOscillationFlags();
    bool isOscillating(const geometry_msgs::Twist& cmd_vel);
    
    // 新增：平滑控制
    geometry_msgs::Twist smoothControl(const geometry_msgs::Twist& cmd_vel,
                                      const geometry_msgs::Twist& prev_vel) const;
};

}  // namespace pathplanning

#endif  // DWA_PLANNER_H