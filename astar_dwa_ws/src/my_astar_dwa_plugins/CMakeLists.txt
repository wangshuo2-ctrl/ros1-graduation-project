cmake_minimum_required(VERSION 3.0.2)
project(my_astar_dwa_plugins)

# 使用C++11标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找catkin包 - 适配ROS Kinetic
find_package(catkin REQUIRED COMPONENTS
  nav_core
  costmap_2d
  tf
  roscpp
  pluginlib
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  angles
  # ROS Kinetic使用tf而不是tf2
  # tf2_ros
  # tf2_geometry_msgs
  # tf2
)

# 查找系统依赖 - 适配ROS Kinetic
# OpenCV 3 for ROS Kinetic
find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc)
# Eigen3
find_package(Eigen3 REQUIRED)

# 设置catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_astar_dwa_plugins
  CATKIN_DEPENDS nav_core costmap_2d tf roscpp pluginlib std_msgs geometry_msgs nav_msgs visualization_msgs angles
  # 移除了tf2相关依赖
  # CATKIN_DEPENDS nav_core costmap_2d tf roscpp pluginlib std_msgs geometry_msgs nav_msgs visualization_msgs angles tf2_ros tf2_geometry_msgs tf2
  # DEPENDS 注释掉Eigen3，因为它只是头文件库
  # DEPENDS OpenCV Eigen3
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# 列出所有源文件
set(ASTAR_SOURCE_FILES
  src/improved_astar_global_planner.cpp
  src/improved_dwa_local_planner.cpp
  # 从文档中提取的算法代码
  src/pathplanning/OAstar.cpp
  src/pathplanning/DWAPlanner.cpp
  # 如果需要，添加其他算法文件
  # src/pathplanning/astar_dwa_navigation.cpp
)

# 添加库
add_library(${PROJECT_NAME} ${ASTAR_SOURCE_FILES})

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  # Eigen3 是纯头文件库，不需要链接
)

# 添加库依赖
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# 安装库文件
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)

# 安装插件描述文件
install(FILES my_astar_dwa_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN "*.yml"
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*.launch.xml"
)