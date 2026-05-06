#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdlib>
#include <cstdio>

using namespace cv;
using namespace std;

// 全局变量声明
Mat map_image;
Mat display_image;
ros::Publisher map_pub;

// 地图参数 - 修改为200×200
const int MAP_WIDTH = 200;
const int MAP_HEIGHT = 200;
const double RESOLUTION = 0.05;
const double ORIGIN_X = -5.0;
const double ORIGIN_Y = -5.0;
const int GRID_SPACING = 20;   // 网格间距（像素）- 对应图片中的20px网格

// 函数声明
void publishMap();
void clearMap();
void saveMap(const string& filename);
void loadMap(const string& filename);
void showHelp();

// 鼠标回调函数
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    // 确保坐标在有效范围内
    x = max(0, min(x, MAP_WIDTH - 1));
    y = max(0, min(y, MAP_HEIGHT - 1));
    
    if (event == EVENT_LBUTTONDOWN) {
        // 计算点击的栅格坐标
        int grid_x = (x / GRID_SPACING) * GRID_SPACING + GRID_SPACING / 2;
        int grid_y = (y / GRID_SPACING) * GRID_SPACING + GRID_SPACING / 2;
        
        // 确保栅格坐标在有效范围内
        grid_x = max(0, min(grid_x, MAP_WIDTH - 1));
        grid_y = max(0, min(grid_y, MAP_HEIGHT - 1));
        
        // 计算栅格的边界
        int grid_start_x = (grid_x / GRID_SPACING) * GRID_SPACING;
        int grid_start_y = (grid_y / GRID_SPACING) * GRID_SPACING;
        int grid_end_x = min(grid_start_x + GRID_SPACING, MAP_WIDTH);
        int grid_end_y = min(grid_start_y + GRID_SPACING, MAP_HEIGHT);
        
        // 获取当前栅格中心点的状态
        uchar current_value = map_image.at<uchar>(grid_y, grid_x);
        
        if (current_value == 255) { 
            // 当前是自由空间 → 将整个栅格设置为障碍物
            for (int gy = grid_start_y; gy < grid_end_y; gy++) {
                for (int gx = grid_start_x; gx < grid_end_x; gx++) {
                    map_image.at<uchar>(gy, gx) = 0; // 黑色 = 障碍物
                    display_image.at<Vec3b>(gy, gx) = Vec3b(0, 0, 0); // 黑色
                }
            }
        } else { 
            // 当前是障碍物 → 将整个栅格设置为自由空间
            for (int gy = grid_start_y; gy < grid_end_y; gy++) {
                for (int gx = grid_start_x; gx < grid_end_x; gx++) {
                    map_image.at<uchar>(gy, gx) = 255; // 白色 = 自由空间
                    display_image.at<Vec3b>(gy, gx) = Vec3b(255, 255, 255); // 白色
                }
            }
        }
        
        // 重新绘制网格线
        for (int i = 0; i <= MAP_WIDTH; i += GRID_SPACING) {
            line(display_image, Point(i, 0), Point(i, MAP_HEIGHT), Scalar(100, 100, 100), 1);
        }
        for (int i = 0; i <= MAP_HEIGHT; i += GRID_SPACING) {
            line(display_image, Point(0, i), Point(MAP_WIDTH, i), Scalar(100, 100, 100), 1);
        }
        
        // 清除 GUI 上的文字显示（注释掉 putText）
        // int grid_num_x = grid_start_x / GRID_SPACING;
        // int grid_num_y = grid_start_y / GRID_SPACING;
        // string status = "Grid clicked: (" + to_string(grid_num_x) + "," + to_string(grid_num_y) + 
        //               ") | Position: (" + to_string(grid_x) + "," + to_string(grid_y) + 
        //               ") | Press 'h' for help";
        // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2);
        // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
        
        imshow("200x200 Map Editor", display_image);
        
        // 实时发布地图更新
        publishMap();
    }
}

// 发布地图到ROS
void publishMap() {
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = ros::Time::now();
    map_msg.info.resolution = RESOLUTION;
    map_msg.info.width = MAP_WIDTH;
    map_msg.info.height = MAP_HEIGHT;
    map_msg.info.origin.position.x = ORIGIN_X;
    map_msg.info.origin.position.y = ORIGIN_Y;
    map_msg.info.origin.orientation.w = 1.0;
    
    // 转换OpenCV图像到ROS地图数据
    map_msg.data.resize(MAP_WIDTH * MAP_HEIGHT);
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            uchar pixel_value = map_image.at<uchar>(y, x);
            // 黑色(0)=障碍物(100), 白色(255)=自由空间(0)
            map_msg.data[y * MAP_WIDTH + x] = (pixel_value == 0) ? 100 : 0;
        }
    }
    
    map_pub.publish(map_msg);
    
    // 统计障碍物数量
    int obstacle_count = 0;
    for (int i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++) {
        if (map_msg.data[i] == 100) obstacle_count++;
    }
    
    ROS_INFO("200x200 map published: obstacles %.1f%%", 
             (float)obstacle_count/(MAP_WIDTH*MAP_HEIGHT)*100);
}

// 清除地图
void clearMap() {
    map_image.setTo(Scalar(255));
    
    // 重新创建显示图像
    cvtColor(map_image, display_image, COLOR_GRAY2BGR);
    
    // 绘制网格线
    for (int i = 0; i <= MAP_WIDTH; i += GRID_SPACING) {
        line(display_image, Point(i, 0), Point(i, MAP_HEIGHT), Scalar(100, 100, 100), 1);
    }
    for (int i = 0; i <= MAP_HEIGHT; i += GRID_SPACING) {
        line(display_image, Point(0, i), Point(MAP_WIDTH, i), Scalar(100, 100, 100), 1);
    }
    
    // 清除 GUI 上的文字显示（注释掉 putText）
    // string status = "Map cleared | Click grid to toggle | Press 'h' for help";
    // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2);
    // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
    
    imshow("200x200 Map Editor", display_image);
    ROS_INFO("200x200 map cleared");
    
    // 发布清除后的地图
    publishMap();
}

// 保存地图到文件
void saveMap(const string& filename) {
    imwrite(filename, map_image);
    ROS_INFO("200x200 map saved: %s", filename.c_str());
}

// 从文件加载地图
void loadMap(const string& filename) {
    Mat loaded_image = imread(filename, IMREAD_GRAYSCALE);
    if (!loaded_image.empty() && loaded_image.cols == MAP_WIDTH && loaded_image.rows == MAP_HEIGHT) {
        map_image = loaded_image.clone();
        
        // 重新创建显示图像
        cvtColor(map_image, display_image, COLOR_GRAY2BGR);
        
        // 绘制网格线
        for (int i = 0; i <= MAP_WIDTH; i += GRID_SPACING) {
            line(display_image, Point(i, 0), Point(i, MAP_HEIGHT), Scalar(100, 100, 100), 1);
        }
        for (int i = 0; i <= MAP_HEIGHT; i += GRID_SPACING) {
            line(display_image, Point(0, i), Point(MAP_WIDTH, i), Scalar(100, 100, 100), 1);
        }
        
        // 清除 GUI 上的文字显示（注释掉 putText）
        // string status = "Map loaded: " + filename + " | Click grid to toggle";
        // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2);
        // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
        
        imshow("200x200 Map Editor", display_image);
        ROS_INFO("200x200 map loaded: %s", filename.c_str());
        
        // 发布加载后的地图
        publishMap();
    } else {
        ROS_ERROR("Cannot load map file or size mismatch: %s", filename.c_str());
    }
}

// 显示帮助信息（控制台输出，可保留中文，但为了统一也改为英文）
void showHelp() {
    cout << "\n=== 200x200 Grid Map Editor Help ===" << endl;
    cout << "Features:" << endl;
    cout << "  - Designed for 200x200 RViz grid maps" << endl;
    cout << "  - Click anywhere on a grid to toggle the entire grid cell" << endl;
    cout << "  - Grid size: " << GRID_SPACING << "x" << GRID_SPACING << " pixels" << endl;
    cout << "  - Publishes to ROS topic /map in real-time" << endl;
    cout << endl;
    cout << "Usage:" << endl;
    cout << "  - Left-click on grid: toggle obstacle state of the entire grid" << endl;
    cout << "  - Keyboard shortcuts:" << endl;
    cout << "      c: Clear map (set all to free space)" << endl;
    cout << "      s: Save map to file" << endl;
    cout << "      l: Load map from file" << endl;
    cout << "      h: Show this help" << endl;
    cout << "      ESC: Exit editor" << endl;
    cout << "=====================================" << endl;
}

int main(int argc, char** argv) {
    // 强制设置ROS环境变量
    cout << "=== Interactive Map Editor: Setting ROS environment variables ===" << endl;
    
    // 使用setenv强制设置
    setenv("ROS_MASTER_URI", "http://localhost:11311", 1);
    setenv("ROS_IP", "localhost", 1);
    setenv("ROS_HOSTNAME", "localhost", 1);
    
    // 验证环境变量
    cout << "ROS_MASTER_URI: " << (getenv("ROS_MASTER_URI") ? getenv("ROS_MASTER_URI") : "not set") << endl;
    cout << "ROS_IP: " << (getenv("ROS_IP") ? getenv("ROS_IP") : "not set") << endl;
    cout << "====================================" << endl;
    
    // 初始化ROS节点
    ros::init(argc, argv, "interactive_map_editor");
    ros::NodeHandle nh;
    
    ROS_INFO("200x200 Interactive Map Editor started");
    ROS_INFO("Map size: %dx%d, resolution: %.3f, origin: (%.2f, %.2f)", 
             MAP_WIDTH, MAP_HEIGHT, RESOLUTION, ORIGIN_X, ORIGIN_Y);
    ROS_INFO("Grid size: %dx%d pixels", GRID_SPACING, GRID_SPACING);
    
    // 创建地图发布者
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    
    // 初始化地图（全白 = 自由空间）
    map_image = Mat(MAP_HEIGHT, MAP_WIDTH, CV_8UC1, Scalar(255));
    
    // 初始化显示图像
    cvtColor(map_image, display_image, COLOR_GRAY2BGR);
    
    // 绘制网格线
    for (int i = 0; i <= MAP_WIDTH; i += GRID_SPACING) {
        line(display_image, Point(i, 0), Point(i, MAP_HEIGHT), Scalar(100, 100, 100), 1);
    }
    for (int i = 0; i <= MAP_HEIGHT; i += GRID_SPACING) {
        line(display_image, Point(0, i), Point(MAP_WIDTH, i), Scalar(100, 100, 100), 1);
    }
    
    // 创建编辑器窗口（英文标题）
    namedWindow("200x200 Map Editor", WINDOW_NORMAL);
    resizeWindow("200x200 Map Editor", 600, 600);
    setMouseCallback("200x200 Map Editor", mouseCallback);
    
    // 清除 GUI 上的文字显示（注释掉 putText）
    // string status = "Click anywhere on a grid to toggle | Grid: " + 
    //                to_string(GRID_SPACING) + "px | Press 'h' for help";
    // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2);
    // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
    
    // 显示初始地图
    imshow("200x200 Map Editor", display_image);
    
    // 显示帮助信息
    showHelp();
    
    ROS_INFO("Usage:");
    ROS_INFO(" - Left-click: toggle obstacle state of the entire grid");
    ROS_INFO(" - Shortcuts: c=clear, s=save, l=load, h=help, ESC=exit");
    
    // 初始发布空地图
    publishMap();
    
    // 主循环
    while (ros::ok()) {
        int key = waitKey(30) & 0xFF;
        
        if (key == 27) { // ESC键退出
            break;
        } else if (key == 'c' || key == 'C') {
            clearMap();
        } else if (key == 's' || key == 'S') {
            saveMap("200x200_grid_map.png");
            
            // 清除 GUI 上的文字显示（注释掉 putText）
            // string status = "Map saved: 200x200_grid_map.png | Click grid to toggle";
            // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2);
            // putText(display_image, status, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
            imshow("200x200 Map Editor", display_image);
        } else if (key == 'l' || key == 'L') {
            loadMap("200x200_grid_map.png");
        } else if (key == 'h' || key == 'H') {
            showHelp();
        }
        
        ros::spinOnce();
    }
    
    destroyAllWindows();
    ROS_INFO("200x200 Interactive Map Editor closed");
    return 0;
}