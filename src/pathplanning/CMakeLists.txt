cmake_minimum_required(VERSION 3.0.2)
project(pathplanning)

# 启用 C++11 支持
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# 查找catkin和所需组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  cv_bridge
  sensor_msgs
  costmap_2d
  dynamic_reconfigure
  map_server
  topic_tools
  visualization_msgs
  angles
  message_generation
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 声明catkin包
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    tf
    cv_bridge
    sensor_msgs
    costmap_2d
    dynamic_reconfigure
    map_server
    topic_tools
    visualization_msgs
    angles
    message_runtime
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  INCLUDE_DIRS include
  LIBRARIES pathplanning
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 编译astar_ros_node (保留原A*)
add_executable(astar_ros_node
  src/astar_ros_node.cpp
  src/Astar.cpp
  src/OccMapTransform.cpp
)
target_link_libraries(astar_ros_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 编译ctAstar_ros_node (保留原对比A*)
add_executable(ctAstar_ros_node
  src/ctAstar_ros_node.cpp
  src/ctAstar.cpp
  src/OccMapTransform.cpp
)
target_link_libraries(ctAstar_ros_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 编译oastar_ros_node (新增：忽略动态障碍物的OA*)
add_executable(oastar_ros_node
  src/Oastar_ros_node.cpp
  src/OAstar.cpp
  src/OccMapTransform.cpp
)
target_link_libraries(oastar_ros_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 编译astar_dwa_navigation
add_executable(astar_dwa_navigation
  src/astar_dwa_navigation.cpp
  src/Astar.cpp
  src/DWAPlanner.cpp
  src/OccMapTransform.cpp
  src/dual_map_manager.cpp
)
target_link_libraries(astar_dwa_navigation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 编译ctastar_dwa_navigation
add_executable(ctastar_dwa_navigation
  src/ctastar_dwa_navigation.cpp
  src/Astar.cpp
  src/ctDWAPlanner.cpp
  src/OccMapTransform.cpp
  src/dual_map_manager.cpp
)
target_link_libraries(ctastar_dwa_navigation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 新增：编译AB_oastar_ros_node (AB版本改进A*)
add_executable(AB_oastar_ros_node
  src/AB_oastar_ros_node.cpp
  src/OAstar.cpp
  src/OccMapTransform.cpp
)
target_link_libraries(AB_oastar_ros_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 新增：编译AB_astar_dwa_navigation (AB版本改进A*+改进DWA)
add_executable(AB_astar_dwa_navigation
  src/AB_astar_dwa_navigation.cpp
  src/Astar.cpp
  src/DWAPlanner.cpp
  src/OccMapTransform.cpp
  src/dual_map_manager.cpp
)
target_link_libraries(AB_astar_dwa_navigation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 新增：编译AB_ctastar_dwa_navigation (AB版本改进A*+传统DWA)
add_executable(AB_ctastar_dwa_navigation
  src/AB_ctastar_dwa_navigation.cpp
  src/Astar.cpp
  src/ctDWAPlanner.cpp
  src/OccMapTransform.cpp
  src/dual_map_manager.cpp
)
target_link_libraries(AB_ctastar_dwa_navigation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 编译interactive_map_editor
add_executable(interactive_map_editor
  src/interactive_map_editor.cpp
)
target_link_libraries(interactive_map_editor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 编译random_obstacle_publisher
add_executable(random_obstacle_publisher
  src/random_obstacle_publisher.cpp
)
target_link_libraries(random_obstacle_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 编译interactive_obstacle_publisher
add_executable(interactive_obstacle_publisher
  src/interactive_obstacle_publisher.cpp
)
target_link_libraries(interactive_obstacle_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 安装规则
install(TARGETS
  astar_ros_node
  ctAstar_ros_node
  oastar_ros_node
  astar_dwa_navigation
  ctastar_dwa_navigation
  AB_oastar_ros_node
  AB_astar_dwa_navigation
  AB_ctastar_dwa_navigation
  interactive_map_editor
  random_obstacle_publisher
  interactive_obstacle_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装Python脚本
install(PROGRAMS
  scripts/obstacle_control_simple.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)