#ifndef OCC_MAP_TRANSFORM_H
#define OCC_MAP_TRANSFORM_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace pathplanning {

class OccMapTransform {
public:
    OccMapTransform();
    virtual ~OccMapTransform() = default;

    // 地图转换函数
    cv::Mat occupancyGridToCVMat(const nav_msgs::OccupancyGrid& grid);
    nav_msgs::OccupancyGrid cvMatToOccupancyGrid(const cv::Mat& mat, 
                                               double resolution = 0.05,
                                               double origin_x = 0.0,
                                               double origin_y = 0.0);
    
    // 坐标转换函数
    cv::Point worldToMap(double world_x, double world_y, 
                         double resolution, double origin_x, double origin_y);
    cv::Point2d mapToWorld(int map_x, int map_y,
                          double resolution, double origin_x, double origin_y);
    
    // 工具函数
    bool isValidMapPoint(int x, int y, int width, int height);
    double distanceBetweenPoints(const cv::Point& p1, const cv::Point& p2);

private:
    // 内部辅助函数
};

} // namespace pathplanning

#endif // OCC_MAP_TRANSFORM_H