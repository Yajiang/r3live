#pragma once
#include <cmath>
#include <math.h>
#include <deque>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>

#define MAX_INI_COUNT (200)
const inline bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};
bool CheckAbnormalState(StatesGroup &state_inout);
void CheckInOutState(const StatesGroup &state_in, StatesGroup &state_inout);
float SigmoidPenalty(float x, float range);

extern double g_imu_scale_factor;

class ImuHandler
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float Scale_factor;

  ImuHandler();
  ~ImuHandler();

  void Process(MeasureGroup &meas, StatesGroup &state, PointCloudXYZINormal::Ptr pcl_un_);
  void Reset();
  void IMUInitial(const MeasureGroup &meas, StatesGroup &state, int &N);

  // Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPointcloud(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZINormal &pcl_in_out);
  void StatePropagate(const MeasureGroup &meas, StatesGroup &state_inout);
  void PointcloudUndistort(const MeasureGroup &meas,  const StatesGroup &state_inout, PointCloudXYZINormal &pcl_out);
  StatesGroup imuPreintegration(const StatesGroup & state_inout, std::deque<sensor_msgs::Imu::ConstPtr> & v_imu,  double end_pose_dt = 0);
  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  Eigen::Vector3d angvel_last;
  Eigen::Vector3d acc_s_last;

  Eigen::Matrix<double,DIM_OF_PROC_N,1> cov_proc_noise;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;

  // std::ofstream fout;

 public:
  /*** Whether is the first frame, init for first frame ***/
  bool m_IsFirstFrame = true;
  bool m_NeedInit = true;

  int init_iter_num = 1;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;

  /*** Undistorted pointcloud ***/
  PointCloudXYZINormal::Ptr cur_pcl_un_;

  //// For timestamp usage
  sensor_msgs::ImuConstPtr m_lastImu;

  /*** For gyroscope integration ***/
  double m_startTimestamp;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Eigen::Matrix3d> v_rot_pcl_;
  std::vector<Pose6D> IMU_pose;
};
