#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <cstdlib>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filters/filter_chain.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "utils/eigen2cv.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <grid_map_msgs/GetGridMap.h>

/**
 * @brief 读取功能包子目录 data 下的 .pcd 文件并返回点云数据以及 ROS 消息
 * 
 * @param pcd_file        pcd 文件的名称，不包含 `.pcd` 后缀
 * @param pointCloud      从 pcd 文件读取出来的点云数据
 * @param pointCloudMsg   转换为 ROS 消息格式的点云数据
 * @param frame_id        坐标系名称
 * @return true           读取 pcd 文件成功 
 * @return false          读取 pcd 文件失败
 */
bool read_point_cloud_from_file(std::string &pcd_file,
                                pcl::PointCloud<pcl::PointXYZ> &pointCloud,
                                sensor_msgs::PointCloud2 &pointCloudMsg,
                                std::string &frame_id) {
    // 获取功能包的根目录，并设置 pcd_dir/data/pcd_file 为文件目录
    std::string pcd_dir = ros::package::getPath("pcd_processor");
    pcd_dir.append("/data/" + pcd_file);

    // 如果读取 PCL 库读取 .pcd 文件失败则报错
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_dir, pointCloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", pcd_dir.c_str());
        return false;
    }    

    // 将 PCL 点云数据转换为 sensor_msgs/PointCloud2
    pcl::toROSMsg(pointCloud, pointCloudMsg);

    // 设置 point_cloud_msg 的 header 信息
    pointCloudMsg.header.frame_id = frame_id;
    pointCloudMsg.header.stamp = ros::Time::now();

    return true;
}

/**
 * @brief 读取功能包子目录 config 下的 grid_map_pcl 配置文件并返回高程图及 ROS 消息
 * 
 * @param config_file        配置文件名称，不包含 `.yaml` 后缀
 * @param gridMapPclLoader   实例化的 gridMapPclLoader 对象，用于处理点云文件
 * @param pointCloud         用 pcl 库读到的点云文件 
 * @param nh                 ROS 句柄，用于获取参数
 * @param frame_id           地图坐标系名称 
 * @param gridMapMsg         转化为ROS 消息格式的 grid_map
 * @return grid_map::GridMap 从点云中读取到的栅格高程图
 */
grid_map::GridMap readRawElevation(std::string &config_file, 
                                   grid_map::GridMapPclLoader &gridMapPclLoader,
                                   pcl::PointCloud<pcl::PointXYZ> &pointCloud, 
                                   ros::NodeHandle &nh, 
                                   std::string &frame_id, 
                                   grid_map_msgs::GridMap &gridMapMsg) {
    // 获取功能包的根目录，并设置 config_dir/data/pcd_file 为文件目录
    std::string config_dir = ros::package::getPath("pcd_processor");
    config_dir.append("/config/" + config_file + ".yaml");    
    
    // 加载 GridMapPclLoader 的配置文件
    try {
        gridMapPclLoader.loadParameters(config_dir);
    } catch (...) {
        ROS_ERROR("Unable to load parameters from file");
        std::exit(1);
    }

    // 创建一个指向 pointCloud 的智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(
        new pcl::PointCloud<pcl::PointXYZ>(pointCloud)
    );

    // GridMapPclLoader 设置点云数据
    gridMapPclLoader.setInputCloud(pointCloudPtr);

    // 调用 grid_map 封装的 pcl 工具处理点云数据
    grid_map::grid_map_pcl::processPointcloud(&gridMapPclLoader, nh);

    // 转换为 "elevation" 层的高程地图
    grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
    gridMap.setFrameId(frame_id);

    // 发布 grid map
    grid_map::GridMapRosConverter::toMessage(gridMap, gridMapMsg);

    return gridMap;
}

/**
 * @brief 根据参数空间中 grid_map_filters 的设置对地图进行滤波处理，得到通行度层
 *        以及障碍层信息
 * 
 * @param gridMap            栅格高程图
 * @param gridMapFilteredMsg ROS 消息格式的滤波结果
 * @return grid_map::GridMap 滤波后的 grid_map，新建了很多层
 */
grid_map::GridMap get_filtered_grid_map(grid_map::GridMap &gridMap,
                                        grid_map_msgs::GridMap &gridMapFilteredMsg) {
    // 创建一个 FilterChain 对象，参考 grid_map_demo 的示例命名为 grid_map::GridMap
    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
    bool config_success_flag = filterChain.configure("grid_map_filters");
    // grid_map_msgs::GridMap gridMapFilteredMsg;

    // FilterChian 配置错误则直接退出
    if (!config_success_flag) {
        ROS_ERROR("Unable to load config");
        std::exit(1);
    }

    // 根据配置文件信息对高程图进行滤波，将结果保存到 gridMapFiltered
    grid_map::GridMap gridMapFiltered;
    bool filter_success_flag = filterChain.update(gridMap, gridMapFiltered);

    // 如果滤波失败则直接退出
    if (!filter_success_flag) {
        ROS_ERROR("Unable to update filtered grid map");
        std::exit(1);
    }

    // 增加障碍层信息
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> 
        obstacle_matrix = gridMapFiltered.get("slope");

    const int OCCUPY = 0;
    const int FREE   = 255;
    const double max_slope = M_PI / 6;

    // 遍历 obstacle_matrix，设置通行度为 0 的栅格为占据，其余为可行空间
    for (int i = 0; i < obstacle_matrix.rows(); ++i) {
        for (int j = 0; j < obstacle_matrix.cols(); ++j) {
            if (obstacle_matrix(i, j) >= max_slope) {
                obstacle_matrix(i, j) = OCCUPY;
            } else {
                obstacle_matrix(i, j) = FREE;
            }
        }
    }

    // 为 gridMapFiltered 添加 obstacle 层
    gridMapFiltered.add("obstacle", obstacle_matrix);

    // 用 obstacle_matrix 初始化 distance 层
    gridMapFiltered.add("distance", obstacle_matrix);

    // 将 obstacle_matrix 转换为 unsigned char 格式的矩阵，用于后续处理
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> 
        binary = obstacle_matrix.cast<unsigned char>();

    // 使用 distanceTransform 函数，更新每个栅格到最近障碍物的距离，并存储到 distance 层
    cv::distanceTransform(
        eigen2cv(binary),                          // unsigned char 格式的障碍层栅格地图
        eigen2cv(gridMapFiltered.get("distance")), // 距离层的地图
        CV_DIST_L2,                                // 使用欧式距离
        CV_DIST_MASK_PRECISE                       // 使用精确的距离计算
    );

    // 添加一个用于越野路径规划器的空白 obstacle 和 distance
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> zero_matrix = 
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Constant(
            obstacle_matrix.rows(), 
            obstacle_matrix.cols(),
            FREE
        );

    gridMapFiltered.add("obstacle_empty", zero_matrix);
    gridMapFiltered.add("distance_empty", zero_matrix);

    // 发布 grid map
    grid_map::GridMapRosConverter::toMessage(gridMapFiltered, gridMapFilteredMsg);

    return gridMapFiltered;
}

// Main Loop
int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "pcd_processor");
    ros::NodeHandle nh("~");

    // 创建一个用于发布 PointCloud2 消息的 Publisher
    ros::Publisher pclPub = nh.advertise<sensor_msgs::PointCloud2>(
        "pcd_to_pointcloud", // Topic
        1,                   // Queue size
        true                 // Latch flag
    );

    // 创建一个发布 GridMap 消息的 Publisher
    ros::Publisher gridMapPub = nh.advertise<grid_map_msgs::GridMap>(
        "grid_map_from_pointcloud", // Topic
        1,                          // Queue size
        true                        // Latch flag
    );

    // 创建一个发布滤波后的 GridMap 消息的 Publisher
    ros::Publisher gridMapFilteredPub = nh.advertise<grid_map_msgs::GridMap>(
        "grid_map_filtered_from_elevation",
        1,
        true
    );

    // 创建一个 PCL PointCloud 对象
    pcl::PointCloud<pcl::PointXYZ> pointCloud;

    // 创建一个 sensor_msgs/PointCloud2 对象
    sensor_msgs::PointCloud2 pointCloudMsg;
    std::string frame_id = "world";

    // 读取 PCD 文件
    std::string pcd_file = "p2at_met_downsampled_limited_area.pcd";

    // 读取 .pcd 文件并转换成点云数据
    bool read_pcd_success_flag = read_point_cloud_from_file(pcd_file, 
                                                            pointCloud,
                                                            pointCloudMsg,
                                                            frame_id);

    // 如果读取不成功，直接终止程序
    if (!read_pcd_success_flag) {
        return (-1);
    }
    
    // 用 GridMapPclLoader 读取点云数据
    grid_map::GridMapPclLoader gridMapPclLoader;
    grid_map_msgs::GridMap gridMapMsg;

    // 确定参数配置文件的路径
    std::string config_file = "grid_map_pcl_params";

    // 将点云数据转换为 grid_map 的
    grid_map::GridMap gridMap = readRawElevation(config_file, 
                                                 gridMapPclLoader, 
                                                 pointCloud, 
                                                 nh, 
                                                 frame_id, 
                                                 gridMapMsg);

    // 对 grid_map 高程层进行滤波处理，得到通行度层和障碍层
    grid_map_msgs::GridMap gridMapFilteredMsg;
    grid_map::GridMap gridMapFiltered = get_filtered_grid_map(gridMap, gridMapFilteredMsg);

    // 循环发布 PointCloud2 和 GridMap 消息
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ROS_INFO("Map Published");
        
        pclPub.publish(pointCloudMsg);
        gridMapPub.publish(gridMapMsg);
        gridMapFilteredPub.publish(gridMapFilteredMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}