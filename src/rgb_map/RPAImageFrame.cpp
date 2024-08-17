
#include <thread>
#include <iostream>
#include "RPAImageFrame.hpp"
#include "tools_color_printf.hpp"
#include "tools_data_io.hpp"

ImageFrame::ImageFrame()
{
    m_gama_para( 0 ) = 1.0;
    m_gama_para( 1 ) = 0.0;
    m_pose_w2c_q.setIdentity();
    m_pose_w2c_t.setZero();
};

ImageFrame::~ImageFrame()
{
    release_image();
}

void ImageFrame::release_image()
{
    m_raw_img.release();
    m_img.release();
    m_img_gray.release();
}

void ImageFrame::refresh_pose_for_projection()
{
    m_pose_c2w_q = m_pose_w2c_q.inverse();
    m_pose_c2w_t = -(m_pose_w2c_q.inverse() * m_pose_w2c_t);
    std::cout << "pose refresh: " << m_pose_c2w_t.transpose() << std::endl;
    m_if_have_set_pose = 1;
}

void ImageFrame::set_pose(const eigen_q &pose_w2c_q, const vec_3 &pose_w2c_t)
{
    m_pose_w2c_q = pose_w2c_q;
    m_pose_w2c_t = pose_w2c_t;

    refresh_pose_for_projection();
}

int ImageFrame::set_frame_idx(int frame_idx)
{
    m_frame_idx = frame_idx;
    return m_frame_idx;
}

void ImageFrame::set_intrinsic(Eigen::Matrix3d &camera_K)
{
    m_cam_K = camera_K;
    m_if_have_set_intrinsic = 1;
    fx = camera_K(0, 0);
    fy = camera_K(1, 1);
    cx = camera_K(0, 2);
    cy = camera_K(1, 2);
    m_gama_para(0) = 1.0;
    m_gama_para(1) = 0.0;
}

ImageFrame::ImageFrame(Eigen::Matrix3d &camera_K)
{
    m_pose_w2c_q.setIdentity();
    m_pose_w2c_t.setZero();
    set_intrinsic(camera_K);
};


void ImageFrame::init_cubic_interpolation()
{
    m_pose_w2c_R = m_pose_w2c_q.toRotationMatrix();
    m_img_rows = m_img.rows;
    m_img_cols = m_img.cols;
#if (CV_MAJOR_VERSION >= 4)
    cv::cvtColor(m_img, m_img_gray, cv::COLOR_RGB2GRAY);
#else
    cv::cvtColor(m_img, m_img_gray, CV_RGB2GRAY);
#endif
}

// void Image_frame::inverse_pose()
// {
//     m_pose_w2c_t = -(m_pose_w2c_q.inverse() * m_pose_w2c_t);
//     m_pose_w2c_q = m_pose_w2c_q.inverse();
//     m_pose_w2c_R = m_pose_w2c_q.toRotationMatrix();

//     m_pose_c2w_q = m_pose_w2c_q.inverse();
//     m_pose_c2w_t = -(m_pose_w2c_q.inverse() * m_pose_w2c_t);
// }

bool ImageFrame::project_3d_to_2d(const pcl::PointXYZI & in_pt, Eigen::Matrix3d &cam_K, double &u, double &v, const double &scale)
{
    if (!m_if_have_set_pose)
    {
        cout << ANSI_COLOR_RED_BOLD << "You have not set the camera pose yet!" << ANSI_COLOR_RESET << endl;
        // refresh_pose_for_projection();
        while (1)
        {};
    }
    if (m_if_have_set_intrinsic == 0)
    {
        cout << "You have not set the intrinsic yet!!!" << endl;
        while (1)
        {} ;
        return false;
    }

    vec_3 pt_w(in_pt.x, in_pt.y, in_pt.z), pt_cam;
    // pt_cam = (m_pose_w2c_q.inverse() * pt_w - m_pose_w2c_q.inverse()*m_pose_w2c_t);
    // std::cout << "m_pose_c2w_t: " << m_pose_c2w_t.transpose() << std::endl;
    // pt_cam = (m_pose_c2w_q * pt_w + m_pose_c2w_t);
    pt_cam = (m_pose_w2c_q * pt_w + m_pose_w2c_t);
    if (pt_cam(2) < 0.001)
    {
        return false;
    }
    u = (pt_cam(0) * fx / pt_cam(2) + cx) * scale;
    v = (pt_cam(1) * fy / pt_cam(2) + cy) * scale;
    return true;
}

bool ImageFrame::if_2d_points_available(const double &u, const double &v, const double &scale, double fov_mar)
{
    double used_fov_margin = m_fov_margin;
    if (fov_mar > 0.0)
    {
        used_fov_margin = fov_mar;
    }
    if ((u / scale >= (used_fov_margin * m_img_cols + 1)) && (std::ceil(u / scale) < ((1 - used_fov_margin) * m_img_cols)) &&
        (v / scale >= (used_fov_margin * m_img_rows + 1)) && (std::ceil(v / scale) < ((1 - used_fov_margin) * m_img_rows)))
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<typename T>
inline T getSubPixel(cv::Mat & mat, const double & row, const  double & col, double pyramid_layer = 0)
{
	int floor_row = floor(row);
	int floor_col = floor(col);
	double frac_row = row - floor_row;
	double frac_col = col - floor_col;
	int ceil_row = floor_row + 1;
	int ceil_col = floor_col + 1;
    if (pyramid_layer != 0)
    {
        int pos_bias = pow(2, pyramid_layer - 1);
        floor_row -= pos_bias;
        floor_col -= pos_bias;
        ceil_row += pos_bias;
        ceil_row += pos_bias;
    }
    return ((1.0 - frac_row) * (1.0 - frac_col) * (T)mat.ptr<T>(floor_row)[floor_col]) +
               (frac_row * (1.0 - frac_col) * (T)mat.ptr<T>(ceil_row)[floor_col]) +
               ((1.0 - frac_row) * frac_col * (T)mat.ptr<T>(floor_row)[ceil_col]) +
               (frac_row * frac_col * (T)mat.ptr<T>(ceil_row)[ceil_col]);
}

vec_3 ImageFrame::get_rgb( double &u, double v, int layer, vec_3 *rgb_dx, vec_3 *rgb_dy )
{
    const int ssd = 5;
    cv::Vec3b rgb = getSubPixel< cv::Vec3b >( m_img, v, u, layer );
    if ( rgb_dx != nullptr )
    {
        cv::Vec3f rgb_left( 0, 0, 0 ), rgb_right( 0, 0, 0 );
        float     pixel_dif = 0;
        for ( int bias_idx = 1; bias_idx < ssd; bias_idx++ )
        {
            rgb_left += getSubPixel< cv::Vec3b >( m_img, v, u - bias_idx, layer );
            rgb_right += getSubPixel< cv::Vec3b >( m_img, v, u + bias_idx, layer );
            pixel_dif += 2 * bias_idx;
        }
        cv::Vec3f cv_rgb_dx = rgb_right - rgb_left;
        *rgb_dx = vec_3( cv_rgb_dx( 0 ), cv_rgb_dx( 1 ), cv_rgb_dx( 2 ) ) / pixel_dif;
    }
    if ( rgb_dy != nullptr )
    {
        cv::Vec3f rgb_down( 0, 0, 0 ), rgb_up( 0, 0, 0 );
        float     pixel_dif = 0;
        for ( int bias_idx = 1; bias_idx < ssd; bias_idx++ )
        {
            rgb_down += getSubPixel< cv::Vec3b >( m_img, v - bias_idx, u, layer );
            rgb_up += getSubPixel< cv::Vec3b >( m_img, v + bias_idx, u, layer );
            pixel_dif += 2 * bias_idx;
        }
        cv::Vec3f cv_rgb_dy = rgb_up - rgb_down;
        *rgb_dy = vec_3( cv_rgb_dy( 0 ), cv_rgb_dy( 1 ), cv_rgb_dy( 2 ) ) / pixel_dif;
    }
    return vec_3( rgb( 0 ), rgb( 1 ), rgb( 2 ) );
}

double ImageFrame::get_grey_color( double &u, double &v, int layer )
{
    double val = 0;

    if ( layer == 0 )
    {
        double gray_val = getSubPixel< uchar >( m_img, v, u );
        return gray_val;
    }
    else
    {
        // TODO
        while ( 1 )
        {
            cout << "To be process here" << __LINE__ << endl;
            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        };
    }

    return m_gama_para( 0 ) * val + m_gama_para( 1 );
}

bool ImageFrame::get_rgb(const double &u, const double &v, int &r, int &g, int &b)
{
    r = m_img.at<cv::Vec3b>(v, u)[2];
    g = m_img.at<cv::Vec3b>(v, u)[1];
    b = m_img.at<cv::Vec3b>(v, u)[0];
    return true;
}

void ImageFrame::display_pose()
{
    cout << "Frm [" << m_frame_idx << "], pose: " << m_pose_w2c_q.coeffs().transpose() << " | " << m_pose_w2c_t.transpose() << " | ";
    cout << fx << ", " << cx << ", " << fy << ", " << cy << ", ";
    cout << endl;
}

void ImageFrame::image_equalize(cv::Mat &img, int amp)
{
    cv::Mat img_temp;
    cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
    // Equalize gray image.
    clahe->apply(img, img_temp);
    img = img_temp;
}

inline void image_equalize(cv::Mat &img, int amp)
{
    cv::Mat img_temp;
    cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
    // Equalize gray image.
    clahe->apply(img, img_temp);
    img = img_temp;
}

inline cv::Mat equalize_color_image_Ycrcb(cv::Mat &image)
{
    cv::Mat hist_equalized_image;
    cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);

    //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
    std::vector<cv::Mat> vec_channels;
    cv::split(hist_equalized_image, vec_channels);

    //Equalize the histogram of only the Y channel
    // cv::equalizeHist(vec_channels[0], vec_channels[0]);
    image_equalize( vec_channels[0], 1 );
    cv::merge(vec_channels, hist_equalized_image);
    cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
    return hist_equalized_image;
}

void ImageFrame::image_equalize()
{
    image_equalize(m_img_gray, 3.0);
    // cv::imshow("before", m_img.clone());
    m_img = equalize_color_image_Ycrcb(m_img);
    // cv::imshow("After", m_img.clone());
}

bool ImageFrame::project_3d_point_in_this_img(const pcl::PointXYZI & in_pt, double &u, double &v, pcl::PointXYZRGB *rgb_pt, double intrinsic_scale)
{
    if (project_3d_to_2d(in_pt, m_cam_K, u, v, intrinsic_scale) == false)
    {
        return false;
    }
    if (if_2d_points_available(u, v, intrinsic_scale) == false)
    {
        // printf_line;
        return false;
    }
    if (rgb_pt != nullptr)
    {
        int r = 0;
        int g = 0;
        int b = 0;
        get_rgb(u, v, r, g, b);
        rgb_pt->x = in_pt.x;
        rgb_pt->y = in_pt.y;
        rgb_pt->z = in_pt.z;
        rgb_pt->r = r;
        rgb_pt->g = g;
        rgb_pt->b = b;
        rgb_pt->a = 255;
    }
    return true;
}

bool ImageFrame::project_3d_point_in_this_img(const vec_3 & in_pt, double &u, double &v, pcl::PointXYZRGB *rgb_pt, double intrinsic_scale)
{
    pcl::PointXYZI temp_pt;
    temp_pt.x = in_pt(0);
    temp_pt.y = in_pt(1);
    temp_pt.z = in_pt(2);
    return project_3d_point_in_this_img(temp_pt, u, v, rgb_pt, intrinsic_scale);
}

void ImageFrame::dump_pose_and_image(const std::string name_prefix)
{
    std::string txt_file_name = std::string(name_prefix).append(".txt");
    std::string image_file_name = std::string(name_prefix).append(".png");
    FILE *fp = fopen(txt_file_name.c_str(), "w+");
    if (fp)
    {
        fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf\r\n", m_pose_w2c_q.w(), m_pose_w2c_q.x(), m_pose_w2c_q.y(), m_pose_w2c_q.z(),
                m_pose_w2c_t(0), m_pose_w2c_t(1), m_pose_w2c_t(2));
        fclose(fp);
    }
    cv::imwrite(image_file_name, m_img);
}

int ImageFrame::load_pose_and_image(const std::string name_prefix, const double image_scale, int if_load_image)
{
    // cout << "Load data from " << name_prefix << ".X" << endl;
    std::string txt_file_name = std::string(name_prefix).append(".txt");
    std::string image_file_name = std::string(name_prefix).append(".png");
    Eigen::MatrixXd pose_data = Common_tools::load_mat_from_txt<double>(txt_file_name);
    if (pose_data.size() == 0)
    {
        // cout << "Load offline data return fail." << endl;
        return 0;
    }
    // cout << "Pose data = " << pose_data << endl;
    m_pose_w2c_q = Eigen::Quaterniond(pose_data(0), pose_data(1), pose_data(2), pose_data(3));
    if (if_load_image)
    {
        m_img = cv::imread(image_file_name.c_str());
        if (image_scale != 1.0)
        {
            cv::resize(m_img, m_img, cv::Size(0, 0), image_scale, image_scale);
        }
        m_img_rows = m_img.rows;
        m_img_cols = m_img.cols;
    }
    // m_pose_w2c_q = Eigen::Map<Eigen::Quaterniond>(&pose_data.data()[0]);
    m_pose_w2c_R = m_pose_w2c_q.toRotationMatrix();
    m_pose_w2c_t = Eigen::Map<Eigen::Vector3d>(&pose_data.data()[4]);
    return 1;
}
