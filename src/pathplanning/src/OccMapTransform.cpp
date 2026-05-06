#include "pathplanning/OccMapTransform.h"

namespace pathplanning {

OccMapTransform::OccMapTransform() {
    // 构造函数
}

cv::Mat OccMapTransform::occupancyGridToCVMat(const nav_msgs::OccupancyGrid& grid) {
    int height = grid.info.height;
    int width = grid.info.width;
    
    cv::Mat mat(height, width, CV_8UC1);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int8_t value = grid.data[y * width + x];
            if (value == -1) {
                mat.at<uchar>(y, x) = 128;  // 未知
            } else if (value == 0) {
                mat.at<uchar>(y, x) = 255;  // 空闲
            } else {
                mat.at<uchar>(y, x) = 0;    // 障碍
            }
        }
    }
    
    return mat;
}

nav_msgs::OccupancyGrid OccMapTransform::cvMatToOccupancyGrid(const cv::Mat& mat, 
                                               double resolution,
                                               double origin_x,
                                               double origin_y) {
    nav_msgs::OccupancyGrid grid;
    
    grid.info.resolution = resolution;
    grid.info.width = mat.cols;
    grid.info.height = mat.rows;
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    
    grid.data.resize(mat.rows * mat.cols);
    
    for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
            uchar value = mat.at<uchar>(y, x);
            int index = y * mat.cols + x;
            
            if (value == 128) {
                grid.data[index] = -1;  // 未知
            } else if (value == 255) {
                grid.data[index] = 0;   // 空闲
            } else {
                grid.data[index] = 100; // 障碍
            }
        }
    }
    
    return grid;
}

cv::Point OccMapTransform::worldToMap(double world_x, double world_y, 
                                     double resolution, double origin_x, double origin_y) {
    int map_x = static_cast<int>((world_x - origin_x) / resolution);
    int map_y = static_cast<int>((world_y - origin_y) / resolution);
    return cv::Point(map_x, map_y);
}

cv::Point2d OccMapTransform::mapToWorld(int map_x, int map_y,
                                      double resolution, double origin_x, double origin_y) {
    double world_x = origin_x + map_x * resolution;
    double world_y = origin_y + map_y * resolution;
    return cv::Point2d(world_x, world_y);
}

bool OccMapTransform::isValidMapPoint(int x, int y, int width, int height) {
    return (x >= 0 && x < width && y >= 0 && y < height);
}

double OccMapTransform::distanceBetweenPoints(const cv::Point& p1, const cv::Point& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

} // namespace pathplanning