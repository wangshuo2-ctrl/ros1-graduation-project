#include "pathplanning/ctDWAPlanner.h"
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <functional>

namespace pathplanning {

ctDWAPlanner::ctDWAPlanner() :
    costmap_initialized_(false),
    initialized_(false),
    goal_reached_(false),
    last_plan_time_(ros::Time::now()),
    last_valid_plan_time_(ros::Time::now())
{
    ROS_INFO("Traditional DWA Planner created");
}

void ctDWAPlanner::initialize() {
    initialized_ = true;
    resetOscillationFlags();
    ROS_INFO("Traditional DWA Planner initialized");
}

bool ctDWAPlanner::isInitialized() const {
    return initialized_ && costmap_initialized_;
}

void ctDWAPlanner::reset() {
    goal_reached_ = false;
    global_plan_.clear();
    resetOscillationFlags();
    last_plan_time_ = ros::Time::now();
    ROS_INFO("Traditional DWA Planner reset");
}

void ctDWAPlanner::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    global_plan_ = global_plan;
    if (!global_plan.empty()) {
        current_goal_ = global_plan.back();
        ROS_INFO("Traditional DWA: Global plan set with %zu points", global_plan.size());
    }
}

void ctDWAPlanner::setCostMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
    updateCostMap(costmap);
}

void ctDWAPlanner::updateCostMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
    if (!costmap) {
        ROS_WARN("Received null costmap pointer");
        return;
    }
    
    if (costmap->info.width == 0 || costmap->info.height == 0) {
        ROS_WARN("Received empty costmap");
        return;
    }
    
    costmap_ = *costmap;
    costmap_data_ = std::vector<int8_t>(costmap->data.begin(), costmap->data.end());
    costmap_width_ = costmap->info.width;
    costmap_height_ = costmap->info.height;
    costmap_resolution_ = costmap->info.resolution;
    costmap_origin_x_ = costmap->info.origin.position.x;
    costmap_origin_y_ = costmap->info.origin.position.y;
    costmap_initialized_ = true;
    
    ROS_INFO("Traditional DWA: Costmap updated: %dx%d, res: %.3f, origin: (%.2f, %.2f)",
             costmap_width_, costmap_height_, costmap_resolution_,
             costmap_origin_x_, costmap_origin_y_);
}

bool ctDWAPlanner::isGoalReached(const geometry_msgs::PoseStamped& robot_pose) {
    if (global_plan_.empty()) {
        return false;
    }
    
    double dx = robot_pose.pose.position.x - current_goal_.pose.position.x;
    double dy = robot_pose.pose.position.y - current_goal_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    bool reached = distance < params_.xy_goal_tolerance;
    
    if (reached && !goal_reached_) {
        goal_reached_ = true;
        ROS_INFO("Traditional DWA: Goal reached! Distance: %.3f", distance);
    }
    
    return reached;
}

void ctDWAPlanner::calculateDynamicWindow(const geometry_msgs::Twist& robot_velocity,
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

std::vector<ctDWAPlanner::Trajectory> ctDWAPlanner::generateTrajectories(
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
        if (vx == 0.0) continue;  // 跳过零速度
        
        for (int j = 0; j < params_.vth_samples; ++j) {
            double vtheta = min_vel_theta + j * vtheta_step;
            if (vx < 0 && vtheta != 0) continue;  // 后退时不允许旋转
            
            // 创建轨迹
            Trajectory traj;
            traj.xv = vx;
            traj.thetav = vtheta;
            traj.yv = 0.0;
            traj.valid = true;
            
            // 模拟轨迹
            geometry_msgs::PoseStamped current_pose = robot_pose;
            double current_yaw = tf::getYaw(current_pose.pose.orientation);
            double sim_time = 0.0;
            
            while (sim_time <= params_.sim_time && traj.valid) {
                // 计算下一个模拟点
                double dt = std::min(params_.sim_granularity, params_.sim_time - sim_time);
                if (dt <= 0) break;
                
                // 更新航向
                if (std::abs(vtheta) > 0) {
                    double angle_step = vtheta * dt;
                    current_yaw += angle_step;
                    
                    // 标准化角度到[-π, π]
                    while (current_yaw > M_PI) current_yaw -= 2.0 * M_PI;
                    while (current_yaw <= -M_PI) current_yaw += 2.0 * M_PI;
                }
                
                // 更新位置
                double delta_x = vx * cos(current_yaw) * dt;
                double delta_y = vx * sin(current_yaw) * dt;
                
                current_pose.pose.position.x += delta_x;
                current_pose.pose.position.y += delta_y;
                current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw);
                
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

double ctDWAPlanner::scoreTrajectory(const Trajectory& traj,
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
    double min_obstacle_dist = std::numeric_limits<double>::max();
    
    for (const auto& pose : traj.path) {
        double dist = getObstacleDistance(pose);
        if (dist < min_obstacle_dist) {
            min_obstacle_dist = dist;
        }
    }
    
    if (min_obstacle_dist < params_.obstacle_cutoff_dist) {
        double obstacle_cost = params_.obstacle_cost_bias *
                               (params_.obstacle_cutoff_dist - min_obstacle_dist) /
                               params_.obstacle_cutoff_dist;
        total_cost += obstacle_cost;
    }
    
    // 4. 方向代价 - 朝向最终目标
    if (!global_plan_.empty()) {
        double goal_yaw = tf::getYaw(current_goal_.pose.orientation);
        double traj_yaw = tf::getYaw(end_pose.pose.orientation);
        double yaw_diff = std::abs(angles::shortest_angular_distance(traj_yaw, goal_yaw));
        total_cost += yaw_diff * 0.3;  // 固定权重
    }
    
    // 5. 速度代价 - 鼓励更高的速度
    total_cost -= std::abs(traj.xv) * 0.1;
    
    return total_cost;
}

ctDWAPlanner::Trajectory ctDWAPlanner::findBestTrajectory(
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

geometry_msgs::Twist ctDWAPlanner::computeVelocityCommands(
    const geometry_msgs::PoseStamped& robot_pose,
    const geometry_msgs::Twist& robot_velocity) {
    
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    
    if (!isInitialized() || global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "Traditional DWA planner not ready: initialized=%d, has_plan=%d, has_costmap=%d",
                          initialized_, !global_plan_.empty(), costmap_initialized_);
        return cmd_vel;
    }
    
    if (isGoalReached(robot_pose)) {
        ROS_INFO("Traditional DWA: Goal reached, stopping");
        return cmd_vel;
    }
    
    // 检查是否停滞
    if (isOscillating(cmd_vel)) {
        ROS_WARN("Traditional DWA: Robot is oscillating, attempting recovery");
        resetOscillationFlags();
    }
    
    // 限制规划频率
    ros::Time now = ros::Time::now();
    if ((now - last_plan_time_).toSec() < (1.0 / params_.controller_frequency)) {
        return cmd_vel;
    }
    
    last_plan_time_ = now;
    
    // 生成并评估轨迹
    std::vector<Trajectory> trajectories = generateTrajectories(robot_pose, robot_velocity);
    
    if (trajectories.empty()) {
        ROS_WARN("Traditional DWA: No valid trajectories found, using fallback control");
        return fallbackControl(robot_pose);
    }
    
    // 选择最优轨迹
    Trajectory best_traj = findBestTrajectory(trajectories, robot_pose);
    
    if (best_traj.valid) {
        cmd_vel.linear.x = best_traj.xv;
        cmd_vel.angular.z = best_traj.thetav;
        last_valid_plan_time_ = now;
        
        ROS_DEBUG_THROTTLE(2.0, "Traditional DWA: linear=%.3f, angular=%.3f, cost=%.3f",
                           best_traj.xv, best_traj.thetav, best_traj.cost);
    } else {
        ROS_WARN("Traditional DWA: No valid trajectory selected, using fallback");
        return fallbackControl(robot_pose);
    }
    
    return cmd_vel;
}

geometry_msgs::Twist ctDWAPlanner::fallbackControl(const geometry_msgs::PoseStamped& robot_pose) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    
    if (global_plan_.empty()) {
        return cmd_vel;
    }
    
    // 简单的PD控制朝向目标
    double dx = current_goal_.pose.position.x - robot_pose.pose.position.x;
    double dy = current_goal_.pose.position.y - robot_pose.pose.position.y;
    double distance_to_goal = std::sqrt(dx * dx + dy * dy);
    
    double goal_yaw = std::atan2(dy, dx);
    double current_yaw = tf::getYaw(robot_pose.pose.orientation);
    double yaw_error = angles::shortest_angular_distance(current_yaw, goal_yaw);
    
    double kp_linear = 0.3;
    double kp_angular = 0.8;
    
    // 检查前方是否有障碍物
    double forward_obstacle_dist = getObstacleDistance(robot_pose);
    double min_safe_distance = 0.3;
    
    if (forward_obstacle_dist < min_safe_distance) {
        // 有障碍物，停止
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.5;  // 尝试旋转
    } else {
        double linear_vel = kp_linear * std::min(distance_to_goal, params_.max_vel_x);
        double angular_vel = kp_angular * yaw_error;
        
        linear_vel = std::max(params_.min_vel_x, std::min(params_.max_vel_x, linear_vel));
        angular_vel = std::max(params_.min_vel_theta, 
                              std::min(params_.max_vel_theta, angular_vel));
        
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
    }
    
    ROS_DEBUG_THROTTLE(2.0, "Traditional DWA fallback: linear=%.3f, angular=%.3f, dist=%.3f",
                       cmd_vel.linear.x, cmd_vel.angular.z, distance_to_goal);
    
    return cmd_vel;
}

bool ctDWAPlanner::worldToMap(double wx, double wy, int& mx, int& my) const {
    if (!costmap_initialized_) return false;
    
    mx = static_cast<int>((wx - costmap_origin_x_) / costmap_resolution_);
    my = static_cast<int>((wy - costmap_origin_y_) / costmap_resolution_);
    
    return (mx >= 0 && mx < costmap_width_ && my >= 0 && my < costmap_height_);
}

double ctDWAPlanner::getObstacleDistance(const geometry_msgs::PoseStamped& pose) const {
    int mx, my;
    if (!worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        return std::numeric_limits<double>::max();
    }
    
    double min_dist = std::numeric_limits<double>::max();
    int search_radius = static_cast<int>(2.5 / costmap_resolution_);  // 2.5m搜索半径
    
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            int check_mx = mx + dx;
            int check_my = my + dy;
            
            if (check_mx < 0 || check_mx >= costmap_width_ || 
                check_my < 0 || check_my >= costmap_height_) {
                continue;
            }
            
            int index = check_my * costmap_width_ + check_mx;
            if (index < 0 || index >= static_cast<int>(costmap_data_.size())) {
                continue;
            }
            
            int8_t cost = costmap_data_[index];
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

bool ctDWAPlanner::isTrajectoryColliding(const Trajectory& traj) const {
    for (const auto& pose : traj.path) {
        double obstacle_dist = getObstacleDistance(pose);
        if (obstacle_dist < params_.robot_radius) {
            return true;
        }
    }
    return false;
}

double ctDWAPlanner::getGoalDistance(const geometry_msgs::PoseStamped& pose) const {
    if (global_plan_.empty()) return 0.0;
    
    double dx = pose.pose.position.x - current_goal_.pose.position.x;
    double dy = pose.pose.position.y - current_goal_.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ctDWAPlanner::getPathDistance(const geometry_msgs::PoseStamped& pose) const {
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

void ctDWAPlanner::resetOscillationFlags() {
    oscillation_start_time_ = ros::Time::now();
}

bool ctDWAPlanner::isOscillating(const geometry_msgs::Twist& cmd_vel) {
    ros::Time now = ros::Time::now();
    
    if ((now - oscillation_start_time_).toSec() > params_.oscillation_timeout) {
        oscillation_start_time_ = now;
        return false;
    }
    
    // 简单的振荡检测：如果长时间速度很小，认为振荡
    double dx = std::abs(cmd_vel.linear.x);
    double dy = std::abs(cmd_vel.angular.z);
    
    if (dx < 0.01 && dy < 0.01) {
        if ((now - last_valid_plan_time_).toSec() > params_.oscillation_timeout) {
            ROS_WARN("Traditional DWA: Robot is stuck");
            return true;
        }
    }
    
    return false;
}

}  // namespace pathplanning