#ifndef CT_DWA_PLANNER_H
#define CT_DWA_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <memory>

namespace pathplanning {

struct ctDWAParams {
    // 速度限制
    double max_vel_x = 0.22;          // 最大线速度
    double min_vel_x = 0.0;           // 最小线速度
    double max_vel_x_backwards = -0.1; // 最大后退速度
    double max_vel_theta = 2.75;      // 最大角速度
    double min_vel_theta = -2.75;     // 最小角速度
    
    // 加速度限制
    double acc_lim_x = 2.5;           // 线加速度限制
    double acc_lim_theta = 3.2;       // 角加速度限制
    
    // 采样参数
    int vx_samples = 20;              // 线速度采样数
    int vth_samples = 40;             // 角速度采样数
    
    // 模拟参数
    double sim_time = 1.5;            // 模拟时间
    double dt = 0.1;                  // 模拟步长
    double sim_granularity = 0.025;   // 模拟粒度
    
    // 目标容差
    double xy_goal_tolerance = 0.15;  // XY目标容差
    double yaw_goal_tolerance = 0.17; // 航向目标容差
    
    // 代价权重
    double goal_distance_bias = 20.0;  // 目标距离权重
    double path_distance_bias = 32.0;  // 路径距离权重
    double obstacle_cost_bias = 1.0;   // 障碍物代价权重
    double occdist_scale = 0.02;       // 障碍物距离缩放
    double obstacle_cutoff_dist = 2.5; // 障碍物截止距离
    
    // 控制器参数
    double controller_frequency = 10.0; // 控制频率
    
    // 机器人参数
    double robot_radius = 0.25;       // 机器人半径
    
    // 代价地图参数
    int obstacle_threshold = 60;      // 障碍物阈值
    
    // 振荡检测
    double oscillation_timeout = 2.0; // 振荡超时
};

class ctDWAPlanner {
public:
    ctDWAPlanner();
    virtual ~ctDWAPlanner() = default;
    
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
    void setParams(const ctDWAParams& params) { params_ = params; }
    ctDWAParams getParams() const { return params_; }
    
    // 轨迹结构体
    struct Trajectory {
        std::vector<geometry_msgs::PoseStamped> path;
        double xv, yv, thetav;
        double cost;
        bool valid;
        
        Trajectory() : xv(0.0), yv(0.0), thetav(0.0), cost(0.0), valid(true) {}
    };
    
private:
    ctDWAParams params_;
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
    std::vector<int8_t> costmap_data_;
    
    // 状态标志
    bool initialized_;
    bool goal_reached_;
    ros::Time last_plan_time_;
    ros::Time last_valid_plan_time_;
    
    // 振荡检测
    ros::Time oscillation_start_time_;
    
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
    double getObstacleDistance(const geometry_msgs::PoseStamped& pose) const;
    bool isTrajectoryColliding(const Trajectory& traj) const;
    
    // 坐标转换函数
    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    
    // 距离计算函数
    double getGoalDistance(const geometry_msgs::PoseStamped& pose) const;
    double getPathDistance(const geometry_msgs::PoseStamped& pose) const;
    
    // 振荡检测
    void resetOscillationFlags();
    bool isOscillating(const geometry_msgs::Twist& cmd_vel);
    
    // 回退策略
    geometry_msgs::Twist fallbackControl(const geometry_msgs::PoseStamped& robot_pose);
};

}  // namespace pathplanning

#endif  // CT_DWA_PLANNER_H
