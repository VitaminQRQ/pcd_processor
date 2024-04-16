# 版本要求和项目名称
cmake_minimum_required(VERSION 2.8.3)
project(pcd_processor)

# 添加编译器标志
add_compile_options(-std=c++17)

set(catkin_deps
  roscpp
  roslib
  pcl_conversions
  pcl_ros
  sensor_msgs
  grid_map_msgs
  grid_map_core
  grid_map_ros
  grid_map_pcl
  filters
  grid_map_filters  
)

# 寻找所需要的 catkin 包
find_package(
  catkin REQUIRED COMPONENTS
  ${catkin_deps}
)

# 寻找 Eigen3 包
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 定义这个 catkin 包的属性
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${catkin_deps}
  DEPENDS OpenCV
)

# 添加 include 目录
include_directories(
  include
  include/utils
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# 定义可执行程序
add_executable(${PROJECT_NAME}_node 
  src/read_point_cloud.cpp
)

# 添加可执行程序的依赖
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# 链接可执行程序到 catkin 库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)