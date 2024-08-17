#include <Eigen/Core>
#include <FOV_Checker/RPAFOVChecker.h>
#include <common_lib.h>
#include <csignal>
#include <geometry_msgs/Vector3.h>
#include <kd_tree/ikd_Tree.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "r3live.hpp"
#include "tools_logger.hpp"
#include <unistd.h>
#include <visualization_msgs/Marker.h>

Camera_Lidar_queue g_camera_lidar_queue;
MeasureGroup Measures;
StatesGroup g_lio_state;

int main(int argc, char **argv)
{
    printf_program("R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package");
    Common_tools::printf_software_version();
    Eigen::initParallel();
    ros::init(argc, argv, "R3LIVE_main");
    IMUFusion * fast_lio_instance = new IMUFusion();
    ros::Rate rate(5000);
    bool status = ros::ok();
    ros::spin();
}
