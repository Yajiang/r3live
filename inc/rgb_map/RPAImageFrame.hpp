#pragma once
#include "string"
#include <mutex>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
// #include "opencv2/xfeatures2d.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "tools_eigen.hpp"
//https://www.opencv-srf.com/2018/02/histogram-equalization.html
#include <boost/serialization/access.hpp>

inline char cv_wait_key(int ms )
{
    static std::mutex cv_wait_key_mutex;
    cv_wait_key_mutex.lock();
    char c = cv::waitKey(ms);
    if (c == 'p' || c == 'P' || c == ' ')
    {
        c = cv::waitKey(0);
    }
    cv_wait_key_mutex.unlock();
    return c;
}

inline cv::Mat equalize_color_image(cv::Mat &image)
{
    cv::Mat hist_equalized_image;
    cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);

    //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
    std::vector<cv::Mat> vec_channels;
    cv::split(hist_equalized_image, vec_channels);

    //Equalize the histogram of only the Y channel
    cv::equalizeHist(vec_channels[0], vec_channels[0]);
    // vec_channels[0] =vec_channels[0] * 0.2;
    //Merge 3 channels in the vector to form the color image in YCrCB color space.
    cv::merge(vec_channels, hist_equalized_image);
    cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
    return hist_equalized_image;
}

inline cv::Mat colormap_depth_img(cv::Mat depth_mat)
{
    cv::Mat cm_img0;
    cv::Mat adjMap;
    double  min;
    double  max;
    // expand your range to 0..255. Similar to histEq();
    cv::minMaxIdx(depth_mat, &min, &max);
    // min = 2;
    // max = 30;
    depth_mat.convertTo( adjMap, CV_8UC1, 255 / ( max - min ), -min );
    // cv::applyColorMap( adjMap, cm_img0, cv::COLORMAP_JET);
    cv::applyColorMap(255-adjMap, cm_img0, cv::COLORMAP_JET);
    return cm_img0;
}

template <typename T = float>
inline cv::Mat img_merge_with_depth(cv::Mat raw_img, cv::Mat depth_mat, float raw_ratio = 0.75)
{
    cv::Mat res_img = raw_img.clone();
    cv::Mat cImg = colormap_depth_img( depth_mat);
   
    for( int i = 0; i < depth_mat.rows; i++)
    {
        for( int j = 0; j < depth_mat.cols; j++)
        {
            if(depth_mat.at<T>(i, j) != 0)
            {
                res_img.at<cv::Vec3b>(i, j) = res_img.at<cv::Vec3b>(i, j) * raw_ratio + cImg.at<cv::Vec3b>(i, j) * (1 - raw_ratio);
            }
        }
    }
    return res_img;
}

template <typename T>
inline void reduce_vector(std::vector<T> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
};

const int MAX_DS_LAY = 7;

struct ImageFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using data_type = double;
    using PointType = pcl::PointXYZI;
    int m_if_have_set_intrinsic = 0;
    Eigen::Matrix3d m_cam_K;
    double fx, fy, cx, cy;
    eigen_q m_pose_w2c_q = eigen_q::Identity();
    vec_3 m_pose_w2c_t = vec_3(0, 0, 0);
    eigen_q m_pose_c2w_q = eigen_q::Identity();
    vec_3 m_pose_c2w_t = vec_3(0, 0, 0);
    int m_if_have_set_pose = 0;
    double m_timestamp = 0.0;
    int m_have_solved_pnp = 0;
    eigen_q m_pnp_pose_w2c_q = eigen_q::Identity();
    vec_3 m_pnp_pose_w2c_t = vec_3(0,0,0);
    
    vec_3 m_pose_t;
    mat_3_3 m_pose_w2c_R;
    int m_img_rows = 0;
    int m_img_cols = 0;
    int m_frame_idx = 0;
    Eigen::Matrix<double, 2, 1> m_gama_para;
    // double m_downsample_step[MAX_DS_LAY] = {1.0, 0.5, 0.25 };
    // double m_downsample_step[10] = {1.0, 0.5, 0.25, 1.0/8.0, 1.0/16.0, 1.0/32.0, 1.0/64.0, 1.0/128 };
    double m_downsample_step[10] = {1.0, 0.5, 0.25, 1.0/8.0, 1.0/16.0, 1.0/24.0, 1.0/32.0, 1.0/64.0 };

    cv::Mat m_img;
    cv::Mat m_raw_img;
    cv::Mat m_img_gray;

    double m_fov_margin = 0.005;
    
    ImageFrame();   
    ~ImageFrame();
    void refresh_pose_for_projection();
    void set_pose(const eigen_q & pose_w2c_q, const vec_3 & pose_w2c_t );
    int set_frame_idx(int frame_idx);
    void set_intrinsic(Eigen::Matrix3d & camera_K);
    ImageFrame(Eigen::Matrix3d &camera_K);
    void init_cubic_interpolation();
    // void inverse_pose();    
    void release_image();
    bool project_3d_to_2d( const pcl::PointXYZI & in_pt, Eigen::Matrix3d & cam_K, double &u, double &v, const double  & scale = 1.0);
    bool if_2d_points_available(const double &u, const double &v, const double &scale = 1.0, double fov_mar = -1.0);
    vec_3 get_rgb(double &u, double v, int layer = 0, vec_3 *rgb_dx = nullptr, vec_3 *rgb_dy = nullptr);
    double get_grey_color(double & u ,double & v, int layer= 0 );
    bool get_rgb( const double & u,  const double & v, int & r, int & g, int & b  );
    void display_pose();
    void image_equalize(cv::Mat &img, int amp = 10.0);
    void image_equalize();
    bool project_3d_point_in_this_img(const pcl::PointXYZI & in_pt, double &u, double &v,   pcl::PointXYZRGB * rgb_pt = nullptr, double intrinsic_scale = 1.0);
    bool project_3d_point_in_this_img(const vec_3 & in_pt, double &u, double &v, pcl::PointXYZRGB *rgb_pt = nullptr, double intrinsic_scale = 1.0);
    void dump_pose_and_image( const std::string name_prefix );
    int load_pose_and_image( const std::string name_prefix, const double image_scale = 1.0, int if_load_image = 1 );

  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        ar &fx;
        ar &fy;
        ar &cx;
        ar &cy;
        ar & m_if_have_set_intrinsic;
        ar & m_timestamp;
        ar & m_img_rows;
        ar & m_img_cols;
        ar & m_frame_idx;
        ar & m_pose_c2w_q;
        ar & m_pose_c2w_t;
    }
};

inline std::shared_ptr< ImageFrame > soft_copy_image_frame( const std::shared_ptr< ImageFrame > &img_ptr )
{
    std::shared_ptr< ImageFrame > res_img_ptr = std::make_shared<ImageFrame>();
    res_img_ptr->fx = img_ptr->fx;
    res_img_ptr->fy = img_ptr->fy;
    res_img_ptr->cx = img_ptr->cx;
    res_img_ptr->cy = img_ptr->cy;
    res_img_ptr->m_if_have_set_intrinsic = img_ptr->m_if_have_set_intrinsic;
    res_img_ptr->m_timestamp = img_ptr->m_timestamp;
    res_img_ptr->m_img_rows = img_ptr->m_img_rows;
    res_img_ptr->m_img_cols = img_ptr->m_img_cols;
    res_img_ptr->m_frame_idx = img_ptr->m_frame_idx;
    res_img_ptr->m_pose_c2w_q = img_ptr->m_pose_c2w_q;
    res_img_ptr->m_pose_c2w_t = img_ptr->m_pose_c2w_t;
    return res_img_ptr;
}