#include "my_astar_dwa_plugins/pathplanning/DWAPlanner.h"
#include <angles/angles.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <functional>
#include <geometry_msgs/Quaternion.h>

// 修改为使用tf1头文件
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace pathplanning {

// 辅助函数：从四元数获取偏航角 - 使用tf1库
double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(quat, tf_quat);
    
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

// 辅助函数：从偏航角创建四元数 - 使用tf1库
geometry_msgs::Quaternion createQuaternionFromYaw(double yaw) {
    tf::Quaternion tf_quat = tf::createQuaternionFromYaw(yaw);
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(tf_quat, quat);
    return quat;
}

DWAPlanner::DWAPlanner():
    costmap_initialized_(false),
    initialized_(false),
    goal_reached_(false),
    last_plan_time_(ros::Time::now()),
    last_valid_plan_time_(ros::Time::now()) {
    ROS_INFO("DWA Planner created");
}

void DWAPlanner::initialize() {
    initialized_ = true;
    resetOscillationFlags();
    ROS_INFO("DWA Planner initialized");
}

bool DWAPlanner::isInitialized() const {
    return initialized_ && costmap_initialized_;
}

void DWAPlanner::reset() {
    goal_reached_ = false;
    global_plan_.clear();
    resetOscillationFlags();
    last_plan_time_ = ros::Time::now();
    ROS_INFO("DWA Planner reset");
}

void DWAPlanner::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    global_plan_ = global_plan;
    if (!global_plan.empty()) {
        current_goal_ = global_plan.back();
    }
    ROS_INFO("DWA: Global plan set with %zu points", global_plan.size());
}

void DWAPlanner::setCostMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
    updateCostMap(costmap);
}

void DWAPlanner::updateCostMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
    if (!costmap) {
        ROS_WARN("Received null costmap pointer");
        return;
    }
    
    if (costmap->info.width == 0 || costmap->info.height == 0) {
        ROS_WARN("Received empty costmap");
        return;
    }
    
    costmap_ = *costmap;
    costmap_width_ = costmap->info.width;
    costmap_height_ = costmap->info.height;
    costmap_resolution_ = costmap->info.resolution;
    costmap_origin_x_ = costmap->info.origin.position.x;
    costmap_origin_y_ = costmap->info.origin.position.y;
    costmap_initialized_ = true;
    
    ROS_INFO("DWA: Costmap updated: %dx%d, res: %.3f, origin: (%.2f, %.2f)",
             costmap_width_, costmap_height_, costmap_resolution_,
             costmap_origin_x_, costmap_origin_y_);
}

bool DWAPlanner::isGoalReached(const geometry_msgs::PoseStamped& robot_pose) {
    if (global_plan_.empty()) {
        return false;
    }
    
    double dx = robot_pose.pose.position.x - current_goal_.pose.position.x;
    double dy = robot_pose.pose.position.y - current_goal_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    bool reached = distance < params_.xy_goal_tolerance;
    
    if (reached && !goal_reached_) {
        goal_reached_ = true;
        ROS_INFO("DWA: Goal reached! Distance: %.3f", distance);
    }
    
    return reached;
}

void DWAPlanner::calculateDynamicWindow(const geometry_msgs::Twist& robot_velocity,
                                        double& min_vel_x, double& max_vel_x,
                                        double& min_vel_theta, double& max_vel_theta) {
    double dt = 1.0 / params_.controller_frequency;
    
    // 计算速度边界
    min_vel_x = std::max(params_.min_vel_x,
                         robot_velocity.linear.x - params_.acc_lim_x * dt);
    max_vel_x = std::min(params_.max_vel_x,
                         robot_velocity.linear.x + params_.acc_lim_x * dt);
    
    // 如果允许后退，设置最小速度为负
    if (params_.max_vel_x_backwards < 0) {
        min_vel_x = std::max(params_.max_vel_x_backwards, min_vel_x);
    }
    
    min_vel_theta = std::max(params_.min_vel_theta,
                            robot_velocity.angular.z - params_.acc_lim_theta * dt);
    max_vel_theta = std::min(params_.max_vel_theta,
                            robot_velocity.angular.z + params_.acc_lim_theta * dt);
    
    ROS_DEBUG("Dynamic window: vx=[%.3f, %.3f], vth=[%.3f, %.3f]",
              min_vel_x, max_vel_x, min_vel_theta, max_vel_theta);
}

std::vector<DWAPlanner::Trajectory> DWAPlanner::generateTrajectories(
    const geometry_msgs::PoseStamped& robot_pose,
    const geometry_msgs::Twist& robot_velocity) {
    std::vector<Trajectory> trajectories;
    
    if (global_plan_.empty()) {
        ROS_WARN("No global plan available");
        return trajectories;
    }
    
    if (!costmap_initialized_) {
        ROS_WARN("No costmap available");
        return trajectories;
    }
    
    double min_vel_x, max_vel_x, min_vel_theta, max_vel_theta;
    calculateDynamicWindow(robot_velocity, min_vel_x, max_vel_x,
                           min_vel_theta, max_vel_theta);
    
    if (max_vel_x <= min_vel_x && max_vel_theta <= min_vel_theta) {
        ROS_WARN("Dynamic window is empty");
        return trajectories;
    }
    
    double vx_step = (max_vel_x - min_vel_x) / std::max(1, params_.vx_samples - 1);
    double vtheta_step = (max_vel_theta - min_vel_theta) / std::max(1, params_.vth_samples - 1);
    
    for (int i = 0; i < params_.vx_samples; ++i) {
        double vx = min_vel_x + i * vx_step;
        if (vx == 0.0) continue; // 跳过零速度
        
        for (int j = 0; j < params_.vth_samples; ++j) {
            double vtheta = min_vel_theta + j * vtheta_step;
            if (vx < 0 && vtheta != 0) continue; // 后退时不允许旋转
            
            // 创建轨迹
            Trajectory traj;
            traj.xv = vx;
            traj.thetav = vtheta;
            traj.yv = 0.0;
            
            // 模拟轨迹
            geometry_msgs::PoseStamped current_pose = robot_pose;
            double current_yaw = getYawFromQuaternion(current_pose.pose.orientation);
            traj.valid = true;
            double sim_time = 0.0;
            
            while (sim_time <= params_.sim_time && traj.valid) {
                // 计算下一个模拟点
                double dt = std::min(params_.sim_granularity, params_.sim_time - sim_time);
                if (dt <= 0) break;
                
                // 更新航向
                if (std::abs(vtheta) > 0) {
                    double angle_step = vtheta * dt;
                    current_yaw += angle_step;
                    current_yaw = normalizeAngle(current_yaw);
                }
                
                // 更新位置
                double delta_x = vx * cos(current_yaw) * dt;
                double delta_y = vx * sin(current_yaw) * dt;
                current_pose.pose.position.x += delta_x;
                current_pose.pose.position.y += delta_y;
                
                // 使用辅助函数创建四元数
                current_pose.pose.orientation = createQuaternionFromYaw(current_yaw);
                
                // 检查碰撞
                if (isTrajectoryColliding(traj)) {
                    traj.valid = false;
                    break;
                }
                
                traj.path.push_back(current_pose);
                sim_time += dt;
            }
            
            if (traj.valid && !traj.path.empty()) {
                trajectories.push_back(traj);
            }
        }
    }
    
    ROS_DEBUG("Generated %zu valid trajectories", trajectories.size());
    return trajectories;
}

double DWAPlanner::scoreTrajectory(const Trajectory& traj,
                                   const geometry_msgs::PoseStamped& robot_pose) {
    if (traj.path.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    const geometry_msgs::PoseStamped& end_pose = traj.path.back();
    double total_cost = 0.0;
    
    // 1. 目标距离代价
    double goal_dist = getGoalDistance(end_pose);
    double goal_cost = goal_dist * params_.goal_distance_bias;
    total_cost += goal_cost;
    
    // 2. 路径距离代价
    double path_dist = getPathDistance(end_pose);
    double path_cost = path_dist * params_.path_distance_bias;
    total_cost += path_cost;
    
    // 3. 障碍物代价
    double obstacle_cost = 0.0;
    double min_obstacle_dist = std::numeric_limits<double>::max();
    for (const auto& pose : traj.path) {
        double dist = getObstacleDistance(pose);
        if (dist < min_obstacle_dist) {
            min_obstacle_dist = dist;
        }
    }
    if (min_obstacle_dist < params_.obstacle_cutoff_dist) {
        obstacle_cost = params_.obstacle_cost_bias * 
                       (params_.obstacle_cutoff_dist - min_obstacle_dist) / 
                       params_.obstacle_cutoff_dist;
    }
    total_cost += obstacle_cost;
    
    // 4. 全局路径方向角评价代价
    if (!global_plan_.empty() && params_.global_heading_bias > 0) {
        double delta_yaw = calculateGlobalHeadingAngle(end_pose);
        double global_heading_cost = delta_yaw * params_.global_heading_bias;
        total_cost += global_heading_cost;
        
        ROS_DEBUG("Global heading angle delta: %.3f rad, cost: %.3f", 
                  delta_yaw, global_heading_cost);
    }
    
    // 5. 方向代价 - 朝向最终目标
    if (!global_plan_.empty()) {
        double goal_yaw = getYawFromQuaternion(current_goal_.pose.orientation);
        double traj_yaw = getYawFromQuaternion(end_pose.pose.orientation);
        double yaw_diff = std::abs(angles::shortest_angular_distance(traj_yaw, goal_yaw));
        total_cost += yaw_diff * params_.goal_angle_bias;
    }
    
    // 6. 路径角度偏差代价
    double path_angle_error = calculatePathAngleError(end_pose);
    total_cost += path_angle_error * params_.path_angle_bias;
    
    // 7. 平滑度代价
    double smoothness = calculateTrajectorySmoothness(traj);
    total_cost += smoothness * params_.smoothness_weight;
    
    // 8. 速度代价 - 鼓励更高的速度
    total_cost -= std::abs(traj.xv) * 0.1;
    
    // 9. 平滑代价 - 鼓励直线行驶
    if (std::abs(traj.thetav) > 0.1) {
        total_cost += std::abs(traj.thetav) * 0.3;
    }
    
    return total_cost;
}

DWAPlanner::Trajectory DWAPlanner::findBestTrajectory(
    const std::vector<Trajectory>& trajectories,
    const geometry_msgs::PoseStamped& robot_pose) {
    Trajectory best_traj;
    double best_cost = std::numeric_limits<double>::max();
    bool found_valid = false;
    
    for (const auto& traj : trajectories) {
        if (!traj.valid) continue;
        
        double cost = scoreTrajectory(traj, robot_pose);
        if (cost < best_cost) {
            best_cost = cost;
            best_traj = traj;
            best_traj.cost = best_cost;
            found_valid = true;
        }
    }
    
    if (!found_valid) {
        ROS_WARN("No valid trajectory found");
        best_traj.valid = false;
    } else {
        ROS_DEBUG("Best trajectory cost: %.3f, vx: %.3f, vth: %.3f",
                  best_cost, best_traj.xv, best_traj.thetav);
    }
    
    return best_traj;
}

geometry_msgs::Twist DWAPlanner::computeVelocityCommands(
    const geometry_msgs::PoseStamped& robot_pose,
    const geometry_msgs::Twist& robot_velocity) {
    static geometry_msgs::Twist prev_cmd_vel;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    
    if (!isInitialized() || global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "DWA planner not ready: initialized=%d, has_plan=%d, has_costmap=%d",
                         initialized_, !global_plan_.empty(), costmap_initialized_);
        return cmd_vel;
    }
    
    if (isGoalReached(robot_pose)) {
        ROS_INFO("DWA: Goal reached, stopping");
        return cmd_vel;
    }
    
    // 检查是否停滞
    if (isOscillating(cmd_vel)) {
        ROS_WARN("DWA: Robot is oscillating, attempting recovery");
        resetOscillationFlags();
    }
    
    // 限制规划频率
    ros::Time now = ros::Time::now();
    if ((now - last_plan_time_).toSec() < (1.0 / params_.controller_frequency)) {
        return cmd_vel;
    }
    last_plan_time_ = now;
    
    // 自适应速度控制
    double adaptive_vel = calculateAdaptiveVelocity(robot_pose);
    double original_max_vel_x = params_.max_vel_x;
    if (params_.enable_adaptive_vel) {
        params_.max_vel_x = std::min(original_max_vel_x, adaptive_vel);
    }
    
    // 生成并评估轨迹
    std::vector<Trajectory> trajectories = generateTrajectories(robot_pose, robot_velocity);
    
    if (trajectories.empty()) {
        ROS_WARN("DWA: No valid trajectories found, using fallback control");
        
        // 回退策略: 简单的PD控制朝向目标
        double dx = current_goal_.pose.position.x - robot_pose.pose.position.x;
        double dy = current_goal_.pose.position.y - robot_pose.pose.position.y;
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);
        double goal_yaw = std::atan2(dy, dx);
        double current_yaw = getYawFromQuaternion(robot_pose.pose.orientation);
        double yaw_error = angles::shortest_angular_distance(current_yaw, goal_yaw);
        
        double kp_linear = 0.3;
        double kp_angular = 0.8;
        
        // 检查前方是否有障碍物
        double forward_obstacle_dist = getObstacleDistance(robot_pose);
        if (forward_obstacle_dist < params_.min_safe_distance) {
            // 有障碍物，停止
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.5; // 尝试旋转
        } else {
            double linear_vel = kp_linear * std::min(distance_to_goal, params_.max_vel_x);
            double angular_vel = kp_angular * yaw_error;
            
            linear_vel = std::max(params_.min_vel_x, std::min(params_.max_vel_x, linear_vel));
            angular_vel = std::max(params_.min_vel_theta,
                                   std::min(params_.max_vel_theta, angular_vel));
            
            cmd_vel.linear.x = linear_vel;
            cmd_vel.angular.z = angular_vel;
        }
        
        ROS_DEBUG_THROTTLE(2.0, "DWA fallback: linear=%.3f, angular=%.3f, dist=%.3f",
                          cmd_vel.linear.x, cmd_vel.angular.z, distance_to_goal);
    } else {
        // 选择最优轨迹
        Trajectory best_traj = findBestTrajectory(trajectories, robot_pose);
        if (best_traj.valid) {
            cmd_vel.linear.x = best_traj.xv;
            cmd_vel.angular.z = best_traj.thetav;
            last_valid_plan_time_ = now;
            
            // 平滑控制
            cmd_vel = smoothControl(cmd_vel, prev_cmd_vel);
            
            ROS_DEBUG_THROTTLE(2.0, "DWA: linear=%.3f, angular=%.3f, cost=%.3f",
                              best_traj.xv, best_traj.thetav, best_traj.cost);
        } else {
            ROS_WARN("DWA: No valid trajectory selected, stopping");
        }
    }
    
    // 恢复原始最大速度
    if (params_.enable_adaptive_vel) {
        params_.max_vel_x = original_max_vel_x;
    }
    
    // 保存当前控制命令用于平滑
    prev_cmd_vel = cmd_vel;
    
    return cmd_vel;
}

bool DWAPlanner::findNearestPointOnGlobalPath(const geometry_msgs::PoseStamped& robot_pose,
                                              size_t& nearest_idx, double& nearest_dist) const {
    if (global_plan_.empty()) {
        return false;
    }
    
    nearest_dist = std::numeric_limits<double>::max();
    nearest_idx = 0;
    
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    
    for (size_t i = 0; i < global_plan_.size(); ++i) {
        double dx = global_plan_[i].pose.position.x - robot_x;
        double dy = global_plan_[i].pose.position.y - robot_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_idx = i;
        }
    }
    
    return true;
}

double DWAPlanner::calculateGlobalHeadingAngle(const geometry_msgs::PoseStamped& robot_pose) const {
    if (global_plan_.size() < 2) {
        return 0.0; // 路径点太少，无法计算方向
    }
    
    // 1. 查找全局路径上距离机器人最近的点
    size_t nearest_idx = 0;
    double nearest_dist = 0.0;
    if (!findNearestPointOnGlobalPath(robot_pose, nearest_idx, nearest_dist)) {
        return 0.0;
    }
    
    // 2. 计算全局路径在最近点处的方向（取最近点及其后面一个点构成的向量方向）
    size_t next_idx = std::min(nearest_idx + 1, global_plan_.size() - 1);
    
    double global_dx = global_plan_[next_idx].pose.position.x - 
                       global_plan_[nearest_idx].pose.position.x;
    double global_dy = global_plan_[next_idx].pose.position.y - 
                       global_plan_[nearest_idx].pose.position.y;
    double global_path_yaw = std::atan2(global_dy, global_dx);
    
    // 3. 计算机器人航向
    double robot_yaw = getYawFromQuaternion(robot_pose.pose.orientation);
    
    // 4. 计算机器人航向与全局路径方向的绝对角度差 δ
    double delta_yaw = std::abs(angles::shortest_angular_distance(robot_yaw, global_path_yaw));
    
    return delta_yaw; // 单位：弧度
}

double DWAPlanner::calculateTrajectorySmoothness(const Trajectory& traj) const {
    if (traj.path.size() < 2) {
        return 0.0;
    }
    
    double total_angle_change = 0.0;
    int angle_count = 0;
    
    for (size_t i = 1; i < traj.path.size() - 1; ++i) {
        // 计算转角
        double x1 = traj.path[i-1].pose.position.x;
        double y1 = traj.path[i-1].pose.position.y;
        double x2 = traj.path[i].pose.position.x;
        double y2 = traj.path[i].pose.position.y;
        double x3 = traj.path[i+1].pose.position.x;
        double y3 = traj.path[i+1].pose.position.y;
        
        // 向量1: p2 - p1
        double dx1 = x2 - x1;
        double dy1 = y2 - y1;
        
        // 向量2: p3 - p2
        double dx2 = x3 - x2;
        double dy2 = y3 - y2;
        
        double dot = dx1 * dx2 + dy1 * dy2;
        double norm1 = sqrt(dx1 * dx1 + dy1 * dy1);
        double norm2 = sqrt(dx2 * dx2 + dy2 * dy2);
        
        if (norm1 > 1e-6 && norm2 > 1e-6) {
            double cos_angle = dot / (norm1 * norm2);
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            double angle = acos(cos_angle);
            total_angle_change += angle;
            angle_count++;
        }
    }
    
    if (angle_count > 0) {
        return total_angle_change / angle_count;
    }
    
    return 0.0;
}

double DWAPlanner::calculateVelocityContinuity(const Trajectory& traj, 
                                              const geometry_msgs::Twist& current_vel) const {
    double vel_diff = fabs(traj.xv - current_vel.linear.x);
    double ang_vel_diff = fabs(traj.thetav - current_vel.angular.z);
    return (vel_diff + ang_vel_diff) * 0.5;
}

double DWAPlanner::calculateAdaptiveVelocity(const geometry_msgs::PoseStamped& robot_pose) const {
    double obstacle_dist = getObstacleDistance(robot_pose);
    
    if (obstacle_dist < params_.min_safe_distance) {
        return 0.0; // 太近，停止
    } else if (obstacle_dist < params_.robot_radius + 0.5) {
        // 较近距离，降低速度
        double ratio = (obstacle_dist - params_.min_safe_distance) / 
                       (params_.robot_radius + 0.5 - params_.min_safe_distance);
        return params_.max_vel_x * ratio * params_.adaptive_speed_factor;
    } else {
        // 安全距离，使用正常速度
        return params_.max_vel_x;
    }
}

double DWAPlanner::calculatePathAngleError(const geometry_msgs::PoseStamped& robot_pose) const {
    if (global_plan_.size() < 2) {
        return 0.0;
    }
    
    // 1. 查找全局路径上距离机器人最近的点
    size_t nearest_idx = 0;
    double nearest_dist = 0.0;
    if (!findNearestPointOnGlobalPath(robot_pose, nearest_idx, nearest_dist)) {
        return 0.0;
    }
    
    // 2. 查找前瞻点
    size_t lookahead_idx = nearest_idx;
    double accumulated_dist = 0.0;
    
    for (size_t i = nearest_idx; i < global_plan_.size() - 1; ++i) {
        double segment_dist = distance(
            global_plan_[i].pose.position.x, global_plan_[i].pose.position.y,
            global_plan_[i+1].pose.position.x, global_plan_[i+1].pose.position.y
        );
        
        accumulated_dist += segment_dist;
        if (accumulated_dist >= params_.lookahead_distance) {
            lookahead_idx = i + 1;
            break;
        }
    }
    
    if (lookahead_idx >= global_plan_.size()) {
        lookahead_idx = global_plan_.size() - 1;
    }
    
    // 3. 计算机器人到前瞻点的向量
    double dx = global_plan_[lookahead_idx].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_[lookahead_idx].pose.position.y - robot_pose.pose.position.y;
    double target_yaw = std::atan2(dy, dx);
    
    // 4. 计算机器人航向
    double robot_yaw = getYawFromQuaternion(robot_pose.pose.orientation);
    
    // 5. 计算角度差
    double angle_error = std::abs(angles::shortest_angular_distance(robot_yaw, target_yaw));
    
    return angle_error;
}

// 障碍物相关函数实现
bool DWAPlanner::worldToMap(double wx, double wy, int& mx, int& my) const {
    if (!costmap_initialized_) return false;
    
    mx = static_cast<int>((wx - costmap_origin_x_) / costmap_resolution_);
    my = static_cast<int>((wy - costmap_origin_y_) / costmap_resolution_);
    
    return (mx >= 0 && mx < costmap_width_ && my >= 0 && my < costmap_height_);
}

double DWAPlanner::getObstacleCost(const geometry_msgs::PoseStamped& pose) const {
    int mx, my;
    if (!worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        return 0.0; // 地图外视为自由空间
    }
    
    int index = my * costmap_width_ + mx;
    if (index < 0 || index >= static_cast<int>(costmap_.data.size())) {
        return 0.0;
    }
    
    int8_t cost = costmap_.data[index];
    if (cost < 0) {
        return 0.0; // 未知区域
    }
    
    return static_cast<double>(cost) / 100.0; // 归一化到[0,1]
}

double DWAPlanner::getObstacleDistance(const geometry_msgs::PoseStamped& pose) const {
    int mx, my;
    if (!worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        return std::numeric_limits<double>::max();
    }
    
    double min_dist = std::numeric_limits<double>::max();
    int search_radius = static_cast<int>(params_.max_obstacle_dist / costmap_resolution_);
    
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            int check_mx = mx + dx;
            int check_my = my + dy;
            
            if (check_mx < 0 || check_mx >= costmap_width_ ||
                check_my < 0 || check_my >= costmap_height_) {
                continue;
            }
            
            int index = check_my * costmap_width_ + check_mx;
            if (index < 0 || index >= static_cast<int>(costmap_.data.size())) {
                continue;
            }
            
            int8_t cost = costmap_.data[index];
            if (cost >= params_.obstacle_threshold) {
                double dist = std::sqrt(dx * dx + dy * dy) * costmap_resolution_;
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }
    }
    
    return min_dist;
}

bool DWAPlanner::isTrajectoryColliding(const Trajectory& traj) const {
    for (const auto& pose : traj.path) {
        double obstacle_dist = getObstacleDistance(pose);
        if (obstacle_dist < params_.robot_radius) {
            return true;
        }
    }
    return false;
}

double DWAPlanner::mapToWorldX(int mx) const {
    return costmap_origin_x_ + (mx + 0.5) * costmap_resolution_;
}

double DWAPlanner::mapToWorldY(int my) const {
    return costmap_origin_y_ + (my + 0.5) * costmap_resolution_;
}

double DWAPlanner::getGoalDistance(const geometry_msgs::PoseStamped& pose) const {
    if (global_plan_.empty()) return 0.0;
    
    double dx = pose.pose.position.x - current_goal_.pose.position.x;
    double dy = pose.pose.position.y - current_goal_.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double DWAPlanner::getPathDistance(const geometry_msgs::PoseStamped& pose) const {
    if (global_plan_.empty()) return std::numeric_limits<double>::max();
    
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& path_pose : global_plan_) {
        double dx = pose.pose.position.x - path_pose.pose.position.x;
        double dy = pose.pose.position.y - path_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}

double DWAPlanner::distance(double x1, double y1, double x2, double y2) const {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double DWAPlanner::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void DWAPlanner::resetOscillationFlags() {
    oscillation_start_time_ = ros::Time::now();
    oscillation_pose_ = geometry_msgs::PoseStamped();
}

bool DWAPlanner::isOscillating(const geometry_msgs::Twist& cmd_vel) {
    ros::Time now = ros::Time::now();
    
    if ((now - oscillation_start_time_).toSec() > params_.oscillation_timeout) {
        oscillation_start_time_ = now;
        return false;
    }
    
    if (oscillation_pose_.header.stamp == ros::Time(0)) {
        oscillation_pose_.pose.position.x = 0;
        oscillation_pose_.pose.position.y = 0;
        return false;
    }
    
    double dx = std::abs(cmd_vel.linear.x);
    double dy = std::abs(cmd_vel.angular.z);
    
    if (dx < 0.01 && dy < 0.01) {
        if ((now - last_valid_plan_time_).toSec() > params_.oscillation_timeout) {
            ROS_WARN("DWA: Robot is stuck");
            return true;
        }
    }
    
    return false;
}

geometry_msgs::Twist DWAPlanner::smoothControl(const geometry_msgs::Twist& cmd_vel,
                                               const geometry_msgs::Twist& prev_vel) const {
    geometry_msgs::Twist smoothed_vel = cmd_vel;
    
    // 线速度平滑
    double delta_vx = cmd_vel.linear.x - prev_vel.linear.x;
    if (fabs(delta_vx) > params_.acc_lim_x * 0.1) {
        smoothed_vel.linear.x = prev_vel.linear.x + 
                               copysign(params_.acc_lim_x * 0.1, delta_vx);
    }
    
    // 角速度平滑
    double delta_vth = cmd_vel.angular.z - prev_vel.angular.z;
    if (fabs(delta_vth) > params_.acc_lim_theta * 0.1) {
        smoothed_vel.angular.z = prev_vel.angular.z + 
                                copysign(params_.acc_lim_theta * 0.1, delta_vth);
    }
    
    // 应用平滑因子
    smoothed_vel.linear.x = smoothed_vel.linear.x * params_.smoothing_factor + 
                           cmd_vel.linear.x * (1.0 - params_.smoothing_factor);
    smoothed_vel.angular.z = smoothed_vel.angular.z * params_.smoothing_factor + 
                            cmd_vel.angular.z * (1.0 - params_.smoothing_factor);
    
    // 确保速度在限制范围内
    smoothed_vel.linear.x = std::max(params_.min_vel_x, 
                                    std::min(params_.max_vel_x, smoothed_vel.linear.x));
    smoothed_vel.angular.z = std::max(params_.min_vel_theta, 
                                     std::min(params_.max_vel_theta, smoothed_vel.angular.z));
    
    return smoothed_vel;
}

} // namespace pathplanning