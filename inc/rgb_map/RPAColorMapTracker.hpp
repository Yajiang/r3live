#pragma once
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "../optical_flow/lkpyramid.hpp"
#include "RPAImageFrame.hpp"
#include "RPAPointcloudRgbd.hpp"
#include "opencv2/calib3d.hpp"
#include "tools_color_printf.hpp"
#include "tools_data_io.hpp"
#include "tools_eigen.hpp"
#include "tools_logger.hpp"
#include "tools_timer.hpp"

class ColorMapTracker {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<cv::Mat> m_img_vec;
  char m_temp_char[1024];
  cv::Mat m_old_frame, m_old_gray;
  cv::Mat frame_gray, m_current_frame;
  cv::Mat m_current_mask;
  unsigned int m_frame_idx = 0;
  double m_last_frame_time, m_current_frame_time;
  std::vector<int> m_current_ids, m_oldIds;
  int if_debug_match_img = 0;
  unsigned int m_maximum_vio_tracked_pts = 300;
  cv::Mat m_ud_map1, m_ud_map2;
  cv::Mat m_intrinsic, m_dist_coeffs;
  std::vector<cv::Point2f> m_lastTrackedPoints, m_current_tracked_pts;
  std::vector<cv::Scalar> m_colors;
  std::vector<void *> m_colorPointsPtrVecInLastFrame;
  std::map<void *, cv::Point2f> m_map_rgb_pts_in_last_frame_pos;
  std::map<void *, cv::Point2f> m_map_rgb_pts_in_current_frame_pos;

  std::map<int, std::vector<cv::Point2f>> m_map_id_pts_vec;
  std::map<int, std::vector<int>> m_map_id_pts_frame;
  std::map<int, std::vector<cv::Point2f>> m_map_frame_pts;
  cv::Mat m_debug_track_img;
  eigen_q q_last_estimated_q = eigen_q::Identity();
  vec_3 t_last_estimated = vec_3(0, 0, 0);
  std::shared_ptr<LK_optical_flow_kernel> m_lk_optical_flow_kernel;
  ColorMapTracker();
  ~ColorMapTracker(){};

  void setIntrinsic(Eigen::Matrix3d cam_K, Eigen::Matrix<double, 5, 1> dist,
                    cv::Size imageSize) {
    cv::eigen2cv(cam_K, m_intrinsic);
    cv::eigen2cv(dist, m_dist_coeffs);
    initUndistortRectifyMap(m_intrinsic, m_dist_coeffs, cv::Mat(), m_intrinsic,
                            imageSize, CV_16SC2, m_ud_map1, m_ud_map2);
  }

  void updateLastTrackingVectorAndIds() {
    int idx = 0;
    m_lastTrackedPoints.clear();
    m_colorPointsPtrVecInLastFrame.clear();
    m_colors.clear();
    m_oldIds.clear();
    for (auto & m_map_rgb_pts_in_last_frame_po : m_map_rgb_pts_in_last_frame_pos) {
      m_colorPointsPtrVecInLastFrame.push_back(m_map_rgb_pts_in_last_frame_po.first);
      m_lastTrackedPoints.push_back(m_map_rgb_pts_in_last_frame_po.second);
      m_colors.push_back(((RGBPoints *)m_map_rgb_pts_in_last_frame_po.first)->m_dbg_color);
      m_oldIds.push_back(idx);
      idx++;
    }
  }

  void setTrackPoints(cv::Mat &img,
                      std::vector<std::shared_ptr<RGBPoints>> &rgb_pts_vec,
                      std::vector<cv::Point2f> &pts_proj_img_vec) {
    m_old_frame = img.clone();
    cv::cvtColor(m_old_frame, m_old_gray, cv::COLOR_BGR2GRAY);
    m_map_rgb_pts_in_last_frame_pos.clear();
    for (unsigned int i = 0; i < rgb_pts_vec.size(); i++) {
      m_map_rgb_pts_in_last_frame_pos[(void *)rgb_pts_vec[i].get()] =
          pts_proj_img_vec[i];
    }
    updateLastTrackingVectorAndIds();
  }

  void init(const std::shared_ptr<ImageFrame> &img_with_pose,
            std::vector<std::shared_ptr<RGBPoints>> &rgb_pts_vec,
            std::vector<cv::Point2f> &pts_proj_img_vec) {
    setTrackPoints(img_with_pose->m_img, rgb_pts_vec, pts_proj_img_vec);
    m_current_frame_time = img_with_pose->m_timestamp;
    m_last_frame_time = m_current_frame_time;
    std::vector<uchar> status;
    m_lk_optical_flow_kernel->track_image(img_with_pose->m_img_gray,
                                          m_lastTrackedPoints,
                                          m_current_tracked_pts, status);
  }

  void updatePoints(std::vector<cv::Point2f> &pts_vec,
                    std::vector<int> &pts_ids) {
    for (unsigned int i = 0; i < pts_vec.size(); i++) {
      m_map_id_pts_vec[pts_ids[i]].push_back(pts_vec[i]);
      m_map_id_pts_frame[pts_ids[i]].push_back(m_frame_idx);
      m_map_frame_pts[m_frame_idx].push_back(pts_vec[i]);
    }
  }

  void undistortImage(cv::Mat &img) {
    cv::Mat temp_img;
    temp_img = img.clone();
    remap(temp_img, img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR);
  }

  void update_and_append_track_pts(std::shared_ptr<ImageFrame> &img_pose,
                                   GlobalMap &map_rgb, double mini_dis = 10.0,
                                   int minimum_frame_diff = 3e8);
  void reject_error_tracking_pts(std::shared_ptr<ImageFrame> &img_pose,
                                 double dis = 2.0);

  template <typename T>
  void reduce_vector(std::vector<T> &v, std::vector<uchar> status) {
    int j = 0;
    for (unsigned int i = 0; i < v.size(); i++)
      if (status[i])
        v[j++] = v[i];
    v.resize(j);
  }

  void trackImg(std::shared_ptr<ImageFrame> &img_pose, double dis = 2.0,
                int if_use_opencv = 1);
  auto getAllTrackedPoints(
      std::vector<std::vector<cv::Point2f>> *img_pt_vec = nullptr) -> int;
  auto removeOutlierRansac(std::shared_ptr<ImageFrame> &img_pose,
                           int if_remove_ourlier = 1) -> int;
};
