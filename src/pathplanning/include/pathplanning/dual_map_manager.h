#ifndef B8CB3AFD_398D_4999_AA30_F0FDA425484D
#define B8CB3AFD_398D_4999_AA30_F0FDA425484D
#ifndef DUAL_MAP_MANAGER_H
#define DUAL_MAP_MANAGER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace pathplanning {

class DualMapManager {
public:
    enum MapMode {
        AUTO_BUILD_MAP = 0,  // 自主建图模式
        PRELOAD_MAP = 1      // 预加载地图模式
    };

    DualMapManager();
    ~DualMapManager() = default;

    /**
     * @brief 初始化双地图管理器
     * @param map_file_path 预加载地图的文件路径
     * @return 初始化是否成功
     */
    bool initialize(const std::string& map_file_path = "");
    
    /**
     * @brief 切换地图模式
     * @param new_mode 新的地图模式
     * @param map_file_path 预加载地图的文件路径（仅PRELOAD_MAP模式需要）
     * @return 切换是否成功
     */
    bool switchMapMode(MapMode new_mode, const std::string& map_file_path = "");
    
    /**
     * @brief 更新自主建图的地图
     * @param grid 地图数据
     */
    void updateAutoMap(const nav_msgs::OccupancyGrid::ConstPtr& grid);
    
    /**
     * @brief 获取当前地图
     * @return 当前地图数据
     */
    nav_msgs::OccupancyGrid getCurrentMap();
    
    /**
     * @brief 检查地图是否准备就绪
     * @return 准备就绪返回true，否则false
     */
    bool isMapReady() const;
    
    /**
     * @brief 获取当前地图模式
     * @return 当前地图模式
     */
    MapMode getCurrentMode() const;
    
    /**
     * @brief 获取地图参数
     * @param resolution 地图分辨率
     * @param origin_x 地图原点X坐标
     * @param origin_y 地图原点Y坐标
     * @param width 地图宽度
     * @param height 地图高度
     */
    void getMapParameters(double& resolution, double& origin_x, 
                         double& origin_y, int& width, int& height) const;

private:
    /**
     * @brief 加载预定义地图
     * @param map_file_path 地图文件路径
     * @return 加载是否成功
     */
    bool loadPredefinedMap(const std::string& map_file_path);
    
    /**
     * @brief 发布当前地图到ROS话题
     */
    void publishMap();
    
    /**
     * @brief 自主建图回调函数
     * @param grid 地图数据
     */
    void autoMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid);

    // 地图数据
    nav_msgs::OccupancyGrid current_map_;
    nav_msgs::OccupancyGrid auto_build_map_;  // 自主建图的地图
    nav_msgs::OccupancyGrid preload_map_;     // 预加载的地图

    // 状态管理
    MapMode current_mode_;
    bool map_ready_;
    bool auto_map_updated_;
    bool preload_map_loaded_;

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Subscriber auto_map_sub_;

    // 地图参数
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    int map_width_;
    int map_height_;
};

} // namespace pathplanning

#endif // DUAL_MAP_MANAGER_H


#endif /* B8CB3AFD_398D_4999_AA30_F0FDA425484D */
