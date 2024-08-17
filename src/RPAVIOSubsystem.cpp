#include "lib_sophus/so3.hpp"
#include "RPAIMUFusion.hpp"
#include "tools_logger.hpp"
#include "tools_mem_used.h"

Common_tools::Cost_time_logger              g_cost_time_logger;
std::shared_ptr< Common_tools::ThreadPool > m_thread_pool_ptr;
double                                      g_vio_frame_cost_time = 0;
double                                      g_lio_frame_cost_time = 0;
int                                         g_flag_if_first_rec_img = 1;
#define DEBUG_PHOTOMETRIC 1
#define USING_CERES 0
void dump_lio_state_to_log( FILE *fp )
{
    if ( fp != nullptr && g_camera_lidar_queue.m_if_dump_log )
    {
        Eigen::Vector3d rot_angle = Sophus::SO3d( Eigen::Quaterniond( g_lio_state.rot_end ) ).log();
        Eigen::Vector3d rot_ext_c2i_angle = Sophus::SO3d( Eigen::Quaterniond( g_lio_state.rot_ext_c2i ) ).log();
        fprintf( fp, "%lf ", g_lio_state.lastUpdateTime - g_camera_lidar_queue.m_first_imu_time ); // Time   [0]
        fprintf( fp, "%lf %lf %lf ", rot_angle( 0 ), rot_angle( 1 ), rot_angle( 2 ) );               // Angle  [1-3]
        fprintf( fp, "%lf %lf %lf ", g_lio_state.pos_end( 0 ), g_lio_state.pos_end( 1 ),
                 g_lio_state.pos_end( 2 ) );          // Pos    [4-6]
        fprintf( fp, "%lf %lf %lf ", 0.0, 0.0, 0.0 ); // omega  [7-9]
        fprintf( fp, "%lf %lf %lf ", g_lio_state.vel_end( 0 ), g_lio_state.vel_end( 1 ),
                 g_lio_state.vel_end( 2 ) );          // Vel    [10-12]
        fprintf( fp, "%lf %lf %lf ", 0.0, 0.0, 0.0 ); // Acc    [13-15]
        fprintf( fp, "%lf %lf %lf ", g_lio_state.bias_g( 0 ), g_lio_state.bias_g( 1 ),
                 g_lio_state.bias_g( 2 ) ); // Bias_g [16-18]
        fprintf( fp, "%lf %lf %lf ", g_lio_state.bias_a( 0 ), g_lio_state.bias_a( 1 ),
                 g_lio_state.bias_a( 2 ) ); // Bias_a [19-21]
        fprintf( fp, "%lf %lf %lf ", g_lio_state.gravity( 0 ), g_lio_state.gravity( 1 ),
                 g_lio_state.gravity( 2 ) ); // Gravity[22-24]
        fprintf( fp, "%lf %lf %lf ", rot_ext_c2i_angle( 0 ), rot_ext_c2i_angle( 1 ),
                 rot_ext_c2i_angle( 2 ) ); // Rot_ext_i2c[25-27]
        fprintf( fp, "%lf %lf %lf ", g_lio_state.pos_ext_c2i( 0 ), g_lio_state.pos_ext_c2i( 1 ),
                 g_lio_state.pos_ext_c2i( 2 ) ); // pos_ext_c2i [28-30]
        fprintf( fp, "%lf %lf %lf %lf ", g_lio_state.camIntrinsic( 0 ), g_lio_state.camIntrinsic( 1 ), g_lio_state.camIntrinsic( 2 ),
                 g_lio_state.camIntrinsic( 3 ) );     // Camera Intrinsic [31-34]
        fprintf( fp, "%lf ", g_lio_state.td_ext_c2i ); // Camera Intrinsic [35]
        // cout <<  g_lio_state.cov.diagonal().transpose() << endl;
        // cout <<  g_lio_state.cov.block(0,0,3,3) << endl;
        for ( int idx = 0; idx < DIM_OF_STATES; idx++ ) // Cov    [36-64]
        {
            fprintf( fp, "%.9f ", sqrt( g_lio_state.cov( idx, idx ) ) );
        }
        fprintf( fp, "%lf %lf ", g_lio_frame_cost_time, g_vio_frame_cost_time ); // costime [65-66]
        fprintf( fp, "\r\n" );
        fflush( fp );
    }
}

double g_last_stamped_mem_mb = 0;
std::string append_space_to_bits( std::string & in_str, int bits )
{
    while( in_str.length() < bits )
    {
        in_str.append(" ");
    }
    return in_str;
}
void IMUFusion::print_dash_board()
{
#if DEBUG_PHOTOMETRIC
    return;
#endif
    int mem_used_mb = ( int ) ( Common_tools::get_RSS_Mb() );
    // clang-format off
    if( (mem_used_mb - g_last_stamped_mem_mb < 1024 ) && g_last_stamped_mem_mb != 0 )
    {
        cout  << ANSI_DELETE_CURRENT_LINE << ANSI_DELETE_LAST_LINE ;
    }
    else
    {
        cout << "\r\n" << endl;
        cout << ANSI_COLOR_WHITE_BOLD << "======================= R3LIVE Dashboard ======================" << ANSI_COLOR_RESET << endl;
        g_last_stamped_mem_mb = mem_used_mb ;
    }
    std::string out_str_line_1, out_str_line_2;
    out_str_line_1 = std::string(        "| System-time | LiDAR-frame | Camera-frame |  Pts in maps | Memory used (Mb) |") ;
    //                                    1             16            30             45             60     
    // clang-format on
    out_str_line_2.reserve( 1e3 );
    out_str_line_2.append( "|   " ).append( Common_tools::get_current_time_str() );
    append_space_to_bits( out_str_line_2, 14 );
    out_str_line_2.append( "|    " ).append( std::to_string( g_LiDAR_frame_index ) );
    append_space_to_bits( out_str_line_2, 28 );
    out_str_line_2.append( "|    " ).append( std::to_string( g_camera_frame_idx ) );
    append_space_to_bits( out_str_line_2, 43 );
    out_str_line_2.append( "| " ).append( std::to_string( m_mapColorPoints.m_ColorPointsVec.size() ) );
    append_space_to_bits( out_str_line_2, 58 );
    out_str_line_2.append( "|    " ).append( std::to_string( mem_used_mb ) );

    out_str_line_2.insert( 58, ANSI_COLOR_YELLOW, 7 );
    out_str_line_2.insert( 43, ANSI_COLOR_BLUE, 7 );
    out_str_line_2.insert( 28, ANSI_COLOR_GREEN, 7 );
    out_str_line_2.insert( 14, ANSI_COLOR_RED, 7 );
    out_str_line_2.insert( 0, ANSI_COLOR_WHITE, 7 );

    out_str_line_1.insert( 58, ANSI_COLOR_YELLOW_BOLD, 7 );
    out_str_line_1.insert( 43, ANSI_COLOR_BLUE_BOLD, 7 );
    out_str_line_1.insert( 28, ANSI_COLOR_GREEN_BOLD, 7 );
    out_str_line_1.insert( 14, ANSI_COLOR_RED_BOLD, 7 );
    out_str_line_1.insert( 0, ANSI_COLOR_WHITE_BOLD, 7 );

    cout << out_str_line_1 << endl;
    cout << out_str_line_2 << ANSI_COLOR_RESET << "          ";
    ANSI_SCREEN_FLUSH;
}

void IMUFusion::setInitialStateCov( StatesGroup &state )
{
    // Set cov
    scope_color( ANSI_COLOR_RED_BOLD );
    state.cov = state.cov.setIdentity() * INIT_COV;
    state.cov.block(18, 18, 6 , 6 ) = state.cov.block(18, 18, 6 , 6 ) .setIdentity() * 0.1;
    state.cov.block(24, 24, 5 , 5 ) = state.cov.block(24, 24, 5 , 5 ).setIdentity() * 0.001;
    state.cov.block( 0, 0, 3, 3 ) = mat_3_3::Identity() * 1e-5;   // R
    state.cov.block( 3, 3, 3, 3 ) = mat_3_3::Identity() * 1e-5;   // T
    state.cov.block( 6, 6, 3, 3 ) = mat_3_3::Identity() * 1e-5;   // vel
    state.cov.block( 9, 9, 3, 3 ) = mat_3_3::Identity() * 1e-3;   // bias_g
    state.cov.block( 12, 12, 3, 3 ) = mat_3_3::Identity() * 1e-1; // bias_a
    state.cov.block( 15, 15, 3, 3 ) = mat_3_3::Identity() * 1e-5; // Gravity
    state.cov( 24, 24 ) = 0.00001;
    state.cov.block( 18, 18, 6, 6 ) = state.cov.block( 18, 18, 6, 6 ).setIdentity() *  1e-3; // Extrinsic between camera and IMU.
    state.cov.block( 25, 25, 4, 4 ) = state.cov.block( 25, 25, 4, 4 ).setIdentity() *  1e-3; // Camera intrinsic.
    // // Set cov
    // scope_color( ANSI_COLOR_RED_BOLD );
    // state.cov = state.cov.setIdentity() * INIT_COV;
    // state.cov.block(18, 18, 6 , 6 ) = state.cov.block(18, 18, 6 , 6 ) .setIdentity() * 0.1;
    // state.cov.block(24, 24, 5 , 5 ) = state.cov.block(24, 24, 5 , 5 ).setIdentity() * 0.001;
    // state.cov.block( 0, 0, 3, 3 ) = mat_3_3::Identity() * 1e-5;   // R
    // state.cov.block( 3, 3, 3, 3 ) = mat_3_3::Identity() * 0.1;   // T
    // state.cov.block( 6, 6, 3, 3 ) = mat_3_3::Identity() * 0.00001;   // vel
    // state.cov.block( 9, 9, 3, 3 ) = mat_3_3::Identity() * 1e-4;   // bias_g
    // state.cov.block( 12, 12, 3, 3 ) = mat_3_3::Identity() * 0.1; // bias_a
    // state.cov.block( 15, 15, 3, 3 ) = mat_3_3::Identity() * 1e-2; // Gravity
    // state.cov( 24, 24 ) = 0.01;
    // state.cov.block( 18, 18, 6, 6 ) = state.cov.block( 18, 18, 6, 6 ).setIdentity() *  1; // Extrinsic between camera and IMU.
    // state.cov.block( 25, 25, 4, 4 ) = state.cov.block( 25, 25, 4, 4 ).setIdentity() *  1e-3; // Camera intrinsic.
}


void IMUFusion::setInitialCameraParameter( StatesGroup &state, double *intrinsic_data, double *camera_dist_data, double *imu_camera_ext_R,
                                           double *imu_camera_ext_t, double cam_k_scale )
{
    scope_color( ANSI_COLOR_YELLOW_BOLD );
    // g_cam_K << 863.4241 / cam_k_scale, 0, 625.6808 / cam_k_scale,
    //     0, 863.4171 / cam_k_scale, 518.3392 / cam_k_scale,
    //     0, 0, 1;

    g_cam_K << intrinsic_data[ 0 ] / cam_k_scale, intrinsic_data[ 1 ], intrinsic_data[ 2 ] / cam_k_scale, intrinsic_data[ 3 ],
        intrinsic_data[ 4 ] / cam_k_scale, intrinsic_data[ 5 ] / cam_k_scale, intrinsic_data[ 6 ], intrinsic_data[ 7 ], intrinsic_data[ 8 ];
    g_cam_dist = Eigen::Map< Eigen::Matrix< double, 5, 1 > >( camera_dist_data );
    state.rot_ext_c2i = Eigen::Map< Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( imu_camera_ext_R );
    state.pos_ext_c2i = Eigen::Map< Eigen::Matrix< double, 3, 1 > >( imu_camera_ext_t );
    // state.pos_ext_c2i.setZero();

    // Lidar to camera parameters.
    m_mutex_lio_process.lock();

    m_inital_rot_ext_c2i = state.rot_ext_c2i;
    m_inital_pos_ext_c2i = state.pos_ext_c2i;
    state.camIntrinsic( 0 ) = g_cam_K( 0, 0 ) ;
    state.camIntrinsic( 1 ) = g_cam_K( 1, 1 ) ;
    state.camIntrinsic( 2 ) = g_cam_K( 0, 2 ) ;
    state.camIntrinsic( 3 ) = g_cam_K( 1, 2 ) ;
    setInitialStateCov( state );
    m_mutex_lio_process.unlock();
}

void IMUFusion::publishTrackImg( cv::Mat &img, double frame_cost_time = -1 )
{
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    cv::Mat pub_image = img.clone();
    std::cout << "publish image " << pub_image.cols << " rows " << pub_image.rows << std::endl; 
    if ( frame_cost_time > 0 )
    {
        char fps_char[ 100 ];
        sprintf( fps_char, "Per-frame cost time: %.2f ms", frame_cost_time );
        // sprintf(fps_char, "%.2f ms", frame_cost_time);

        if ( pub_image.cols <= 640 )
        {
            cv::putText( pub_image, std::string( fps_char ), cv::Point( 30, 30 ), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar( 255, 255, 255 ), 2, 8,
                         0 ); // 640 * 480
        }
        else if ( pub_image.cols > 640 )
        {
            cv::putText( pub_image, std::string( fps_char ), cv::Point( 30, 50 ), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar( 255, 255, 255 ), 2, 8,
                         0 ); // 1280 * 1080
        }
    }
    out_msg.image = pub_image; // Your cv::Mat
    pub_track_img.publish( out_msg );
}

// void R3LIVE::publish_track_img_with_pts( cv::Mat &img, double frame_cost_time = -1 )
// {
//     cv_bridge::CvImage out_msg;
//     out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
//     out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
//     cv::Mat pub_image = img.clone();
//     if ( frame_cost_time > 0 )
//     {
//         char fps_char[ 100 ];
//         sprintf( fps_char, "Per-frame cost time: %.2f ms", frame_cost_time );
//         // sprintf(fps_char, "%.2f ms", frame_cost_time);

//         if ( pub_image.cols <= 640 )
//         {
//             cv::putText( pub_image, std::string( fps_char ), cv::Point( 30, 30 ), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar( 255, 255, 255 ), 2, 8,
//                          0 ); // 640 * 480
//         }
//         else if ( pub_image.cols > 640 )
//         {
//             cv::putText( pub_image, std::string( fps_char ), cv::Point( 30, 50 ), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar( 255, 255, 255 ), 2, 8,
//                          0 ); // 1280 * 1080
//         }
//     }
//     out_msg.image = pub_image; // Your cv::Mat
//     pub_track_img.publish( out_msg );
// }

void IMUFusion::publishRawImg( cv::Mat &img )
{
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image = img;                                   // Your cv::Mat
    pub_raw_img.publish( out_msg );
}

int        sub_image_typed = 0; // 0: TBD 1: sub_raw, 2: sub_comp
std::mutex mutex_image_callback;

std::deque< sensor_msgs::CompressedImageConstPtr > g_received_compressed_img_msg;
std::deque< sensor_msgs::ImageConstPtr >           g_received_img_msg;
std::shared_ptr< std::thread >                     g_thr_process_image;

void IMUFusion::serviceProcessImgBuffer()
{
    while ( 1 )
    {
        // To avoid uncompress so much image buffer, reducing the use of memory.
        if ( m_queue_image_with_pose.size() > 40000 )
        {
            while ( m_queue_image_with_pose.size() > 400000 )
            {
                ros::spinOnce();
                std::this_thread::sleep_for( std::chrono::milliseconds( 2 ) );
                std::this_thread::yield();
            }
        }
        cv::Mat image_get;
        double  img_rec_time;
        if ( sub_image_typed == 2 )
        {
            while ( g_received_compressed_img_msg.size() == 0 )
            {
                ros::spinOnce();
                std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
                std::this_thread::yield();
            }
            sensor_msgs::CompressedImageConstPtr msg = g_received_compressed_img_msg.front();
            try
            {
                cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
                img_rec_time = msg->header.stamp.toSec();
                image_get = cv_ptr_compressed->image;
                cv_ptr_compressed->image.release();
            }
            catch ( cv_bridge::Exception &e )
            {
                printf( "Could not convert from '%s' to 'bgr8' !!! ", msg->format.c_str() );
            }
            mutex_image_callback.lock();
            g_received_compressed_img_msg.pop_front();
            mutex_image_callback.unlock();
        }
        else
        {
            while ( g_received_img_msg.size() == 0 )
            {
                ros::spinOnce();
                std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
                std::this_thread::yield();
            }
            sensor_msgs::ImageConstPtr msg = g_received_img_msg.front();
            image_get = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
            img_rec_time = msg->header.stamp.toSec();
            mutex_image_callback.lock();
            g_received_img_msg.pop_front();
            mutex_image_callback.unlock();
        }
        processImage( image_get, img_rec_time );
    }
}

void IMUFusion::imageCompressedCallback( const sensor_msgs::CompressedImageConstPtr &msg )
{
    std::unique_lock< std::mutex > lock2( mutex_image_callback );
    if ( sub_image_typed == 1 )
    {
        return; // Avoid subscribe the same image twice.
    }
    sub_image_typed = 2;
    g_received_compressed_img_msg.push_back( msg );
    if ( g_flag_if_first_rec_img )
    {
        g_flag_if_first_rec_img = 0;
        m_thread_pool_ptr->commit_task( &IMUFusion::serviceProcessImgBuffer, this );
    }
    return;
}

// ANCHOR - image_callback
void IMUFusion::imageCallback( const sensor_msgs::ImageConstPtr &msg )
{
    std::unique_lock< std::mutex > lock( mutex_image_callback );
    if ( sub_image_typed == 2 )
    {
        return; // Avoid subscribe the same image twice.
    }
    sub_image_typed = 1;

    if ( g_flag_if_first_rec_img )
    {
        g_flag_if_first_rec_img = 0;
        m_thread_pool_ptr->commit_task( &IMUFusion::serviceProcessImgBuffer, this );
    }

    cv::Mat temp_img = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
    processImage( temp_img, msg->header.stamp.toSec() );
}

double last_accept_time = 0;
int    buffer_max_frame = 0;
int    total_frame_count = 0;
void   IMUFusion::processImage( cv::Mat &temp_img, double msg_time )
{
    cv::Mat img_get;
    if ( temp_img.rows == 0 )
    {
        cout << "Process image error, image rows =0 " << endl;
        return;
    }
    // std::cout << "temp_img size " << temp_img.size() << std::endl;

    if ( msg_time < last_accept_time )
    {
        cout << "Error, image time revert!!" << endl;
        return;
    }

    if ( ( msg_time - last_accept_time ) < ( 1.0 / m_control_image_freq ) * 0.9 )
    {
        return;
    }
    last_accept_time = msg_time;

    if ( m_camera_start_ros_tim < 0 )
    {
        m_camera_start_ros_tim = msg_time;
        m_vio_scale_factor = m_vioImageWidth * m_image_downsample_ratio / temp_img.cols; // 320 * 24
        // load_vio_parameters();
        setInitialCameraParameter( g_lio_state, m_camera_intrinsic.data(), m_camera_dist_coeffs.data(), m_camera_ext_R.data(),
                                      m_camera_ext_t.data(), m_vio_scale_factor );
        cv::eigen2cv( g_cam_K, intrinsic );
        cv::eigen2cv( g_cam_dist, dist_coeffs );
        initUndistortRectifyMap( intrinsic, dist_coeffs, cv::Mat(), intrinsic, cv::Size( m_vioImageWidth / m_vio_scale_factor, m_vioImageHeight / m_vio_scale_factor ),
                                 CV_16SC2, m_ud_map1, m_ud_map2 );
        m_thread_pool_ptr->commit_task( &IMUFusion::servicePubColorMaps, this);
        m_thread_pool_ptr->commit_task( &IMUFusion::serviceVIOUpdate, this);
    }
    // std::cout << "temp image size cols: " << temp_img.cols << " rows :" << temp_img.rows << std::endl;

    if ( m_image_downsample_ratio != 1.0 )
    {
        cv::resize( temp_img, img_get, cv::Size( m_vioImageWidth / m_vio_scale_factor, m_vioImageHeight / m_vio_scale_factor ) );
    }
    else
    {
        img_get = temp_img; // clone ?
    }
    std::shared_ptr< ImageFrame > img_pose = std::make_shared< ImageFrame >( g_cam_K );
    if ( m_if_pub_raw_img )
    {
        img_pose->m_raw_img = img_get;
    }
    cv::remap( img_get, img_pose->m_img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR );
    // cv::imshow("sub Img", img_pose->m_img);
    img_pose->m_timestamp = msg_time;
    img_pose->init_cubic_interpolation();
    img_pose->image_equalize();
    m_camera_data_mutex.lock();
    m_queue_image_with_pose.push_back( img_pose );
    m_camera_data_mutex.unlock();
    total_frame_count++;

    if ( m_queue_image_with_pose.size() > buffer_max_frame )
    {
        buffer_max_frame = m_queue_image_with_pose.size();
    }

    // cout << "Image queue size = " << m_queue_image_with_pose.size() << endl;
}

void IMUFusion::loadVIOParameters()
{

    std::vector< double > camera_intrinsic_data, camera_dist_coeffs_data, camera_ext_R_data, camera_ext_t_data;
    m_ros_node_handle.getParam( "r3live_vio/image_width", m_vioImageWidth );
    m_ros_node_handle.getParam( "r3live_vio/image_height", m_vioImageHeight );
    m_ros_node_handle.getParam( "r3live_vio/camera_intrinsic", camera_intrinsic_data );
    m_ros_node_handle.getParam( "r3live_vio/camera_dist_coeffs", camera_dist_coeffs_data );
    m_ros_node_handle.getParam( "r3live_vio/camera_ext_R", camera_ext_R_data );
    m_ros_node_handle.getParam( "r3live_vio/camera_ext_t", camera_ext_t_data );

    CV_Assert( ( m_vioImageWidth != 0 && m_vioImageHeight != 0 ) );

    if ( ( camera_intrinsic_data.size() != 9 ) || ( camera_dist_coeffs_data.size() != 5 ) || ( camera_ext_R_data.size() != 9 ) ||
         ( camera_ext_t_data.size() != 3 ) )
    {

        cout << ANSI_COLOR_RED_BOLD << "Load VIO parameter fail!!!, please check!!!" << endl;
        printf( "Load camera data size = %d, %d, %d, %d\n", ( int ) camera_intrinsic_data.size(), camera_dist_coeffs_data.size(),
                camera_ext_R_data.size(), camera_ext_t_data.size() );
        cout << ANSI_COLOR_RESET << endl;
        std::this_thread::sleep_for( std::chrono::seconds( 3000000 ) );
    }

    m_camera_intrinsic = Eigen::Map< Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( camera_intrinsic_data.data() );
    m_camera_dist_coeffs = Eigen::Map< Eigen::Matrix< double, 5, 1 > >( camera_dist_coeffs_data.data() );
    m_camera_ext_R = Eigen::Map< Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( camera_ext_R_data.data() );
    m_camera_ext_t = Eigen::Map< Eigen::Matrix< double, 3, 1 > >( camera_ext_t_data.data() ); // pointcloud to camera
    auto R_point2cam = Eigen::Map< Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( camera_ext_R_data.data() );
    auto t_point2cam = Eigen::Map< Eigen::Matrix< double, 3, 1 > >( camera_ext_t_data.data() ); // pointcloud to camera
    //static const Eigen::Vector3d Lidar_offset_to_IMU(-38.86, 4.6, 33.46); // POP3

    m_camera_ext_t = -R_point2cam * Lidar_offset_to_IMU +  t_point2cam;
    m_camera_ext_R = m_camera_ext_R.inverse().eval();
    m_camera_ext_t = - m_camera_ext_R * m_camera_ext_t; // camera to imu

    // m_camera_ext_R << 0.98749966, 0.07059958, -0.07675654, 0.99629712,
    //     0.03873596, -0.13766935, -0.04906874, 0.98926205;
    // m_camera_ext_t << -29.22365, 10.77156, 38.70245;
    // m_camera_ext_t = -R_point2cam * Lidar_offset_to_IMU +  t_point2cam;

    // m_camera_ext_R << 0.99369908, 0.07287842, 0.08515205, 
    // -0.07301431,0.99732973, -0.00152152, 
    // -0.08503556, -0.00470538, 0.99636681;
    // m_camera_ext_t << -17.06121, 22.37007, 38.72366;
    cout << "[Ros_parameter]: r3live_vio/Camera Intrinsic: " << endl;
    cout << m_camera_intrinsic << endl;
    cout << "[Ros_parameter]: r3live_vio/Camera distcoeff: " << m_camera_dist_coeffs.transpose() << endl;
    cout << "[Ros_parameter]: r3live_vio/Camera extrinsic R: " << endl;
    cout << m_camera_ext_R << endl;
    cout << "[Ros_parameter]: r3live_vio/Camera extrinsic T: " << m_camera_ext_t.transpose() << endl;
    std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
}

void IMUFusion::setImagePose( std::shared_ptr< ImageFrame > &image_pose, const StatesGroup &state )
{
    mat_3_3 rot_mat = state.rot_end;
    vec_3   t_vec = state.pos_end;
    vec_3   pose_t = rot_mat * state.pos_ext_c2i + t_vec; // camera 2 world
    // mat_3_3 R_w2c = rot_mat * state.rot_ext_c2i; // camera 2 world

    // image_pose->set_pose( eigen_q( R_w2c ), pose_t );
    mat_3_3 R_c2w = rot_mat * state.rot_ext_c2i; // camera 2 world

    vec_3 t_w2c = -R_c2w.inverse() * pose_t; 
    mat_3_3 R_w2c = R_c2w.inverse();

    image_pose->set_pose( eigen_q( R_w2c ), t_w2c );

    image_pose->fx = state.camIntrinsic( 0 );
    image_pose->fy = state.camIntrinsic( 1 );
    image_pose->cx = state.camIntrinsic( 2 );
    image_pose->cy = state.camIntrinsic( 3 );

    image_pose->m_cam_K << image_pose->fx, 0, image_pose->cx, 0, image_pose->fy, image_pose->cy, 0, 0, 1;
    scope_color( ANSI_COLOR_CYAN_BOLD );
    cout << "Set Image Pose frm [" << image_pose->m_frame_idx << "], pose: " << eigen_q(rot_mat).coeffs().transpose()
    << " | " << pose_t.transpose()
    << " | " << eigen_q(rot_mat).angularDistance( eigen_q::Identity()) *57.3 << endl;
    // image_pose->inverse_pose();
}

void IMUFusion::publishCameraOdom( std::shared_ptr< ImageFrame > &image, double msg_time )
{
    eigen_q            odom_q = image->m_pose_w2c_q;
    vec_3              odom_t = image->m_pose_w2c_t;
    nav_msgs::Odometry camera_odom;
    camera_odom.header.frame_id = "world";
    camera_odom.child_frame_id = "/aft_mapped";
    camera_odom.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
    camera_odom.pose.pose.orientation.x = odom_q.x();
    camera_odom.pose.pose.orientation.y = odom_q.y();
    camera_odom.pose.pose.orientation.z = odom_q.z();
    camera_odom.pose.pose.orientation.w = odom_q.w();
    camera_odom.pose.pose.position.x = odom_t( 0 );
    camera_odom.pose.pose.position.y = odom_t( 1 );
    camera_odom.pose.pose.position.z = odom_t( 2 );
    pub_odom_cam.publish( camera_odom );

    geometry_msgs::PoseStamped msg_pose;
    msg_pose.header.stamp = ros::Time().fromSec( msg_time );
    msg_pose.header.frame_id = "world";
    msg_pose.pose.orientation.x = odom_q.x();
    msg_pose.pose.orientation.y = odom_q.y();
    msg_pose.pose.orientation.z = odom_q.z();
    msg_pose.pose.orientation.w = odom_q.w();
    msg_pose.pose.position.x = odom_t( 0 );
    msg_pose.pose.position.y = odom_t( 1 );
    msg_pose.pose.position.z = odom_t( 2 );
    camera_path.header.frame_id = "world";
    camera_path.poses.push_back( msg_pose );
    pub_path_cam.publish( camera_path );
}

void IMUFusion::publishTrackPoints(ColorMapTracker &tracker) {
  pcl::PointXYZRGB temp_point;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_for_pub;

  for (auto it : tracker.m_map_rgb_pts_in_current_frame_pos) {
    vec_3 pt = ((RGBPoints *)it.first)->get_pos();
    cv::Scalar color = ((RGBPoints *)it.first)->m_dbg_color;
    temp_point.x = pt(0);
    temp_point.y = pt(1);
    temp_point.z = pt(2);
    temp_point.r = color(2);
    temp_point.g = color(1);
    temp_point.b = color(0);
    pointcloud_for_pub.points.push_back(temp_point);
  }
  sensor_msgs::PointCloud2 ros_pc_msg;
  pcl::toROSMsg(pointcloud_for_pub, ros_pc_msg);
  ros_pc_msg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  ros_pc_msg.header.frame_id = "world";       // world; camera_init
  m_pub_visual_tracked_3d_pts.publish(ros_pc_msg);
}

bool IMUFusion::vioPreintegration( StatesGroup &stateIn, StatesGroup &stateOut, double curFrameTimestamp )
{
    stateOut = stateIn;
    if ( curFrameTimestamp <= stateIn.lastUpdateTime )
    {
        // cout << ANSI_COLOR_RED_BOLD << "Error current_frame_time <= state_in.last_update_time | " <<
        // current_frame_time - state_in.last_update_time << ANSI_COLOR_RESET << endl;
        return false;
    }
    mtxBuffer.lock();
    std::deque< sensor_msgs::Imu::ConstPtr > vioImuQueue;
    for (auto & it : imuBufferVio)
    {
        vioImuQueue.push_back( it );
        if ( it->header.stamp.toSec() > curFrameTimestamp )
        {
            break;
        }
    }

    while ( !imuBufferVio.empty() )
    {
        double imuTimestamp = imuBufferVio.front()->header.stamp.toSec();
        if ( imuTimestamp < curFrameTimestamp - 0.2 )
        {
            imuBufferVio.pop_front();
        }
        else
        {
            break;
        }
    }
    // cout << "Current VIO_imu buffer size = " << imu_buffer_vio.size() << endl;
    stateOut = m_imu_process->imuPreintegration( stateOut, vioImuQueue, curFrameTimestamp - vioImuQueue.back()->header.stamp.toSec() );
    eigen_q q_diff( stateOut.rot_end.transpose() * stateIn.rot_end );
    cout << "Pos diff = " << (stateOut.pos_end - stateIn.pos_end).transpose() << endl;
    cout << "Euler diff = " << q_diff.angularDistance(eigen_q::Identity()) * 57.3 << endl;
    mtxBuffer.unlock();
    stateOut.lastUpdateTime = curFrameTimestamp;
    return true;
}

// ANCHOR - huber_loss
double getHuberLossScale( double reprojection_error, double outlier_threshold = 2.0 )
{
    // http://ceres-solver.org/nnls_modeling.html#lossfunction
    double scale = 1.0;
    if ( reprojection_error / outlier_threshold < 1.0 )
    {
        scale = 1.0;
    }
    else
    {
        scale = ( 2 * sqrt( reprojection_error ) / sqrt( outlier_threshold ) - 1.0 ) / reprojection_error;
    }
    return scale;
}

// ANCHOR - VIO_esikf
const int minimum_iteration_pts = 10;
bool IMUFusion::vioEsikf(StatesGroup &stateIn, ColorMapTracker &opticalTrack) {
  Common_tools::Timer tim;
  tim.tic();
  scope_color(ANSI_COLOR_BLUE_BOLD);
  StatesGroup stateIter = stateIn;
  if (!m_ifEstimateIntrinsic) // When disable the online intrinsic calibration.
  {
    stateIter.camIntrinsic << g_cam_K(0, 0), g_cam_K(1, 1), g_cam_K(0, 2),
        g_cam_K(1, 2);
  }

  if (!m_ifEstimateI2CExtrinsic) {
    stateIter.pos_ext_c2i = m_inital_pos_ext_c2i;
    stateIter.rot_ext_c2i = m_inital_rot_ext_c2i;
  }

  Eigen::Matrix<double, -1, -1> H_mat;
  Eigen::Matrix<double, -1, 1> measVec;
  Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
  Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
  Eigen::Matrix<double, -1, -1> K, KH;
  Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> K_1;

  Eigen::SparseMatrix<double> H_mat_spa, H_T_H_spa, K_spa, KH_spa, vec_spa,
      I_STATE_spa;
  I_STATE.setIdentity();
  I_STATE_spa = I_STATE.sparseView();
  double fx, fy, cx, cy, time_td;

  int total_pt_size = opticalTrack.m_map_rgb_pts_in_current_frame_pos.size();
  std::vector<double> last_reprojection_error_vec(total_pt_size),
      current_reprojection_error_vec(total_pt_size);

  if (total_pt_size < minimum_iteration_pts) {
    stateIn = stateIter;
    return false;
  }
  H_mat.resize(total_pt_size * 2, DIM_OF_STATES);
  measVec.resize(total_pt_size * 2, 1);
  double last_repro_err = 3e8;
  int avail_pt_count = 0;
  double last_avr_repro_err = 0;

  double acc_reprojection_error = 0;
  double img_res_scale = 1.0;
  std::cout
      << "vio_esikf!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: "
      << std::endl;
  for (int iter_count = 0; iter_count < esikf_iter_times; iter_count++) {
    cout << "========== Iter " << iter_count << " =========" << endl;
    mat_3_3 R_imu = stateIter.rot_end;
    vec_3 t_imu = stateIter.pos_end;
    vec_3 t_c2w = R_imu * stateIter.pos_ext_c2i + t_imu;
    mat_3_3 R_c2w = R_imu * stateIter.rot_ext_c2i; // world to camera frame

    fx = stateIter.camIntrinsic(0);
    fy = stateIter.camIntrinsic(1);
    cx = stateIter.camIntrinsic(2);
    cy = stateIter.camIntrinsic(3);
    time_td = stateIter.td_ext_c2i_delta;

    vec_3 t_w2c = -R_c2w.transpose() * t_c2w;
    mat_3_3 R_w2c = R_c2w.transpose();
    int pt_idx = -1;
    acc_reprojection_error = 0;
    vec_3 pt_3d_w, pt_3d_cam;
    vec_2 pt_img_measure, pt_img_proj, pt_img_vel;
    eigen_mat_d<2, 3> mat_pre;
    eigen_mat_d<3, 3> mat_A, mat_B, mat_C, mat_D, pt_hat;
    H_mat.setZero();
    solution.setZero();
    measVec.setZero();
    avail_pt_count = 0;
    for (auto it = opticalTrack.m_map_rgb_pts_in_last_frame_pos.begin();
         it != opticalTrack.m_map_rgb_pts_in_last_frame_pos.end(); it++) {
      pt_3d_w = ((RGBPoints *)it->first)->get_pos();
      pt_img_vel = ((RGBPoints *)it->first)->m_img_vel;
      pt_img_measure = vec_2(it->second.x, it->second.y);
      pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
      pt_img_proj = vec_2(fx * pt_3d_cam(0) / pt_3d_cam(2) + cx,
                          fy * pt_3d_cam(1) / pt_3d_cam(2) + cy) +
                    time_td * pt_img_vel;
      double repro_err = (pt_img_proj - pt_img_measure).norm();
      double huber_loss_scale = getHuberLossScale(repro_err);
      pt_idx++;
      acc_reprojection_error += repro_err;
      // if (iter_count == 0 || ((repro_err -
      // last_reprojection_error_vec[pt_idx]) < 1.5))
      if (iter_count == 0 || ((repro_err - last_avr_repro_err * 5.0) < 0)) {
        last_reprojection_error_vec[pt_idx] = repro_err;
      } else {
        last_reprojection_error_vec[pt_idx] = repro_err;
      }
      avail_pt_count++;
      // Appendix E of r2live_Supplementary_material.
      // https://github.com/hku-mars/r2live/blob/master/supply/r2live_Supplementary_material.pdf
      mat_pre << fx / pt_3d_cam(2), 0, -fx * pt_3d_cam(0) / pt_3d_cam(2), 0,
          fy / pt_3d_cam(2), -fy * pt_3d_cam(1) / pt_3d_cam(2);

      pt_hat = Sophus::SO3d::hat(
          (R_imu.transpose() * (pt_3d_w - t_imu))); // convert pt -> imu body
      mat_A = stateIter.rot_ext_c2i.transpose() * pt_hat;
      mat_B = -stateIter.rot_ext_c2i.transpose() * (R_imu.transpose());
      mat_C = Sophus::SO3d::hat(pt_3d_cam);
      mat_D = -stateIter.rot_ext_c2i.transpose();
      measVec.block(pt_idx * 2, 0, 2, 1) =
          (pt_img_proj - pt_img_measure) * huber_loss_scale / img_res_scale;

      H_mat.block(pt_idx * 2, 0, 2, 3) = mat_pre * mat_A * huber_loss_scale;
      H_mat.block(pt_idx * 2, 3, 2, 3) = mat_pre * mat_B * huber_loss_scale;
      if (DIM_OF_STATES > 24) {
        // Estimate time td.
        H_mat.block(pt_idx * 2, 24, 2, 1) = pt_img_vel * huber_loss_scale;
        // H_mat(pt_idx * 2, 24) = pt_img_vel(0) * huber_loss_scale;
        // H_mat(pt_idx * 2 + 1, 24) = pt_img_vel(1) * huber_loss_scale;
      }
      if (m_ifEstimateI2CExtrinsic) {
        H_mat.block(pt_idx * 2, 18, 2, 3) = mat_pre * mat_C * huber_loss_scale;
        H_mat.block(pt_idx * 2, 21, 2, 3) = mat_pre * mat_D * huber_loss_scale;
      }

      if (m_ifEstimateIntrinsic) {
        H_mat(pt_idx * 2, 25) = pt_3d_cam(0) / pt_3d_cam(2) * huber_loss_scale;
        H_mat(pt_idx * 2 + 1, 26) =
            pt_3d_cam(1) / pt_3d_cam(2) * huber_loss_scale;
        H_mat(pt_idx * 2, 27) = 1 * huber_loss_scale;
        H_mat(pt_idx * 2 + 1, 28) = 1 * huber_loss_scale;
      }
    }
    H_mat = H_mat / img_res_scale;
    acc_reprojection_error /= total_pt_size;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: "
              << std::endl;
    std::cout << "project error: " << acc_reprojection_error << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: "
              << std::endl;

    last_avr_repro_err = acc_reprojection_error;
    if (avail_pt_count < minimum_iteration_pts) {
      break;
    }

    H_mat_spa = H_mat.sparseView();
    Eigen::SparseMatrix<double> Hsub_T_temp_mat = H_mat_spa.transpose();
    vec_spa = (stateIter - stateIn).sparseView();
    H_T_H_spa = Hsub_T_temp_mat * H_mat_spa;
    // Notice that we have combine some matrix using () in order to boost the
    // matrix multiplication.
    Eigen::SparseMatrix<double> temp_inv_mat =
        ((H_T_H_spa.toDense() +
          eigen_mat<-1, -1>(stateIn.cov * m_cam_measurement_weight).inverse())
             .inverse())
            .sparseView();
    KH_spa = temp_inv_mat * (Hsub_T_temp_mat * H_mat_spa);
    solution =
        (temp_inv_mat * (Hsub_T_temp_mat * ((-1 * measVec.sparseView()))) -
         (I_STATE_spa - KH_spa) * vec_spa)
            .toDense();

    stateIter = stateIter + solution;

    if (fabs(acc_reprojection_error - last_repro_err) < 0.01) {
      break;
    }
    last_repro_err = acc_reprojection_error;
  }

  if (avail_pt_count >= minimum_iteration_pts) {
    stateIter.cov =
        ((I_STATE_spa - KH_spa) * stateIter.cov.sparseView()).toDense();
  }

  stateIter.td_ext_c2i += stateIter.td_ext_c2i_delta;
  stateIter.td_ext_c2i_delta = 0;
  stateIn = stateIter;
  return true;
}

bool IMUFusion::vioPhotometric( StatesGroup &stateIn, ColorMapTracker &opticalTrack, std::shared_ptr< ImageFrame > &image )
{
    Common_tools::Timer tim;
    tim.tic();
    StatesGroup state_iter = stateIn;
    if ( !m_ifEstimateIntrinsic )     // When disable the online intrinsic calibration.
    {
        state_iter.camIntrinsic << g_cam_K( 0, 0 ), g_cam_K( 1, 1 ), g_cam_K( 0, 2 ), g_cam_K( 1, 2 );
    }
    if ( !m_ifEstimateI2CExtrinsic ) // When disable the online extrinsic calibration.
    {
        state_iter.pos_ext_c2i = m_inital_pos_ext_c2i;
        state_iter.rot_ext_c2i = m_inital_rot_ext_c2i;
    }
    Eigen::Matrix< double, -1, -1 >                       H_mat, R_mat_inv;
    Eigen::Matrix< double, -1, 1 >                        meas_vec;
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > G, H_T_H, I_STATE;
    Eigen::Matrix< double, DIM_OF_STATES, 1 >             solution;
    Eigen::Matrix< double, -1, -1 >                       K, KH;
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > K_1;
    Eigen::SparseMatrix< double >                         H_mat_spa, H_T_H_spa, R_mat_inv_spa, K_spa, KH_spa, vec_spa, I_STATE_spa;
    I_STATE.setIdentity();
    I_STATE_spa = I_STATE.sparseView();
    double fx, fy, cx, cy, time_td;

    int                   total_pt_size = opticalTrack.m_map_rgb_pts_in_current_frame_pos.size();
    std::vector< double > last_reprojection_error_vec( total_pt_size ), current_reprojection_error_vec( total_pt_size );
    if ( total_pt_size < minimum_iteration_pts )
    {
        stateIn = state_iter;
        return false;
    }

    int err_size = 3;
    H_mat.resize( total_pt_size * err_size, DIM_OF_STATES );
    meas_vec.resize( total_pt_size * err_size, 1 );
    R_mat_inv.resize( total_pt_size * err_size, total_pt_size * err_size );

    double last_repro_err = 3e8;
    int    avail_pt_count = 0;
    double last_avr_repro_err = 0;
    int    if_esikf = 1;

    double acc_photometric_error = 0;
#if DEBUG_PHOTOMETRIC
    printf("==== [Image frame %d] ====\r\n", g_camera_frame_idx);
#endif
    for ( int iter_count = 0; iter_count < 2; iter_count++ )
    {
        mat_3_3 R_imu = state_iter.rot_end;
        vec_3   t_imu = state_iter.pos_end;
        vec_3   t_c2w = R_imu * state_iter.pos_ext_c2i + t_imu;
        mat_3_3 R_c2w = R_imu * state_iter.rot_ext_c2i; // world to camera frame

        fx = state_iter.camIntrinsic( 0 );
        fy = state_iter.camIntrinsic( 1 );
        cx = state_iter.camIntrinsic( 2 );
        cy = state_iter.camIntrinsic( 3 );
        time_td = state_iter.td_ext_c2i_delta;

        vec_3   t_w2c = -R_c2w.transpose() * t_c2w;
        mat_3_3 R_w2c = R_c2w.transpose();
        int     pt_idx = -1;
        acc_photometric_error = 0;
        vec_3               pt_3d_w, pt_3d_cam;
        vec_2               pt_img_measure, pt_img_proj, pt_img_vel;
        eigen_mat_d< 2, 3 > mat_pre;
        eigen_mat_d< 3, 2 > mat_photometric;
        eigen_mat_d< 3, 3 > mat_d_pho_d_img;
        eigen_mat_d< 3, 3 > mat_A, mat_B, mat_C, mat_D, pt_hat;
        R_mat_inv.setZero();
        H_mat.setZero();
        solution.setZero();
        meas_vec.setZero();
        avail_pt_count = 0;
        int iter_layer = 0;
        tim.tic( "Build_cost" );
        for ( auto it = opticalTrack.m_map_rgb_pts_in_last_frame_pos.begin(); it != opticalTrack.m_map_rgb_pts_in_last_frame_pos.end(); it++ )
        {
            if ( ( ( RGBPoints * ) it->first )->m_N_rgb < 3 )
            {
                continue;
            }
            pt_idx++;
            pt_3d_w = ( ( RGBPoints * ) it->first )->get_pos();
            pt_img_vel = ( ( RGBPoints * ) it->first )->m_img_vel;
            pt_img_measure = vec_2( it->second.x, it->second.y );
            pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
            pt_img_proj = vec_2( fx * pt_3d_cam( 0 ) / pt_3d_cam( 2 ) + cx, fy * pt_3d_cam( 1 ) / pt_3d_cam( 2 ) + cy ) + time_td * pt_img_vel;

            vec_3   pt_rgb = ( ( RGBPoints * ) it->first )->get_rgb();
            mat_3_3 pt_rgb_info = mat_3_3::Zero();
            mat_3_3 pt_rgb_cov = ( ( RGBPoints * ) it->first )->get_rgb_cov();
            for ( int i = 0; i < 3; i++ )
            {
                pt_rgb_info( i, i ) = 1.0 / pt_rgb_cov( i, i ) ;
                R_mat_inv( pt_idx * err_size + i, pt_idx * err_size + i ) = pt_rgb_info( i, i );
                // R_mat_inv( pt_idx * err_size + i, pt_idx * err_size + i ) =  1.0;
            }
            vec_3  obs_rgb_dx, obs_rgb_dy;
            vec_3  obs_rgb = image->get_rgb( pt_img_proj( 0 ), pt_img_proj( 1 ), 0, &obs_rgb_dx, &obs_rgb_dy );
            vec_3  photometric_err_vec = ( obs_rgb - pt_rgb );
            double huber_loss_scale = getHuberLossScale( photometric_err_vec.norm() );
            photometric_err_vec *= huber_loss_scale;
            double photometric_err = photometric_err_vec.transpose() * pt_rgb_info * photometric_err_vec;

            acc_photometric_error += photometric_err;

            last_reprojection_error_vec[ pt_idx ] = photometric_err;

            mat_photometric.setZero();
            mat_photometric.col( 0 ) = obs_rgb_dx;
            mat_photometric.col( 1 ) = obs_rgb_dy;

            avail_pt_count++;
            mat_pre << fx / pt_3d_cam( 2 ), 0, -fx * pt_3d_cam( 0 ) / pt_3d_cam( 2 ), 0, fy / pt_3d_cam( 2 ), -fy * pt_3d_cam( 1 ) / pt_3d_cam( 2 );
            mat_d_pho_d_img = mat_photometric * mat_pre;

            pt_hat = Sophus::SO3d::hat( ( R_imu.transpose() * ( pt_3d_w - t_imu ) ) );
            mat_A = state_iter.rot_ext_c2i.transpose() * pt_hat;
            mat_B = -state_iter.rot_ext_c2i.transpose() * ( R_imu.transpose() );
            mat_C = Sophus::SO3d::hat( pt_3d_cam );
            mat_D = -state_iter.rot_ext_c2i.transpose();
            meas_vec.block( pt_idx * 3, 0, 3, 1 ) = photometric_err_vec ;

            H_mat.block( pt_idx * 3, 0, 3, 3 ) = mat_d_pho_d_img * mat_A * huber_loss_scale;
            H_mat.block( pt_idx * 3, 3, 3, 3 ) = mat_d_pho_d_img * mat_B * huber_loss_scale;
            if ( 1 )
            {
                if ( m_ifEstimateI2CExtrinsic )
                {
                    H_mat.block( pt_idx * 3, 18, 3, 3 ) = mat_d_pho_d_img * mat_C * huber_loss_scale;
                    H_mat.block( pt_idx * 3, 21, 3, 3 ) = mat_d_pho_d_img * mat_D * huber_loss_scale;
                }
            }
        }
        R_mat_inv_spa = R_mat_inv.sparseView();
       
        last_avr_repro_err = acc_photometric_error;
        if ( avail_pt_count < minimum_iteration_pts )
        {
            break;
        }
        // Esikf
        tim.tic( "Iter" );
        if ( if_esikf )
        {
            H_mat_spa = H_mat.sparseView();
            Eigen::SparseMatrix< double > Hsub_T_temp_mat = H_mat_spa.transpose();
            vec_spa = ( state_iter - stateIn ).sparseView();
            H_T_H_spa = Hsub_T_temp_mat * R_mat_inv_spa * H_mat_spa;
            Eigen::SparseMatrix< double > temp_inv_mat =
                ( H_T_H_spa.toDense() + ( stateIn.cov * m_cam_measurement_weight ).inverse() ).inverse().sparseView();
            // ( H_T_H_spa.toDense() + ( state_in.cov ).inverse() ).inverse().sparseView();
            Eigen::SparseMatrix< double > Ht_R_inv = ( Hsub_T_temp_mat * R_mat_inv_spa );
            KH_spa = temp_inv_mat * Ht_R_inv * H_mat_spa;
            solution = ( temp_inv_mat * ( Ht_R_inv * ( ( -1 * meas_vec.sparseView() ) ) ) - ( I_STATE_spa - KH_spa ) * vec_spa ).toDense();
        }
        state_iter = state_iter + solution;
#if DEBUG_PHOTOMETRIC
        cout << "Average photometric error: " <<  acc_photometric_error / total_pt_size << endl;
        cout << "Solved solution: "<< solution.transpose() << endl;
#else
        if ( ( acc_photometric_error / total_pt_size ) < 10 ) // By experience.
        {
            break;
        }
#endif
        if ( fabs( acc_photometric_error - last_repro_err ) < 0.01 )
        {
            break;
        }
        last_repro_err = acc_photometric_error;
    }
    if ( if_esikf && avail_pt_count >= minimum_iteration_pts )
    {
        state_iter.cov = ( ( I_STATE_spa - KH_spa ) * state_iter.cov.sparseView() ).toDense();
    }
    state_iter.td_ext_c2i += state_iter.td_ext_c2i_delta;
    state_iter.td_ext_c2i_delta = 0;
    stateIn = state_iter;
    return true;
}

void IMUFusion::servicePubColorMaps()
{
    int last_publish_map_idx = -3e8;
    int sleep_time_aft_pub = 10;
    int numberPointsPerTopic = 1000;
    if ( numberPointsPerTopic < 0 )
    {
        return;
    }
    while ( 1 )
    {
        ros::spinOnce();
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
        pcl::PointCloud< pcl::PointXYZRGB > colorPointcloud;
        sensor_msgs::PointCloud2            rosPointcloudMsg;
        int pts_size = m_mapColorPoints.m_ColorPointsVec.size();
        colorPointcloud.resize( numberPointsPerTopic );
        // for (int i = pts_size - 1; i > 0; i--)
        int publishId = 0;
        int cur_topic_idx = 0;
        if ( last_publish_map_idx == m_mapColorPoints.m_last_updated_frame_idx )
        {
            continue;
        }
        last_publish_map_idx = m_mapColorPoints.m_last_updated_frame_idx;
        for ( int i = 0; i < pts_size; i++ )
        {
            if ( m_mapColorPoints.m_ColorPointsVec[ i ]->m_N_rgb < 1 )
            {
                continue;
            }
            colorPointcloud.points[ publishId ].x = m_mapColorPoints.m_ColorPointsVec[ i ]->m_pos[ 0 ];
            colorPointcloud.points[ publishId ].y = m_mapColorPoints.m_ColorPointsVec[ i ]->m_pos[ 1 ];
            colorPointcloud.points[ publishId ].z = m_mapColorPoints.m_ColorPointsVec[ i ]->m_pos[ 2 ];
            colorPointcloud.points[ publishId ].r = m_mapColorPoints.m_ColorPointsVec[ i ]->m_rgb[ 2 ];
            colorPointcloud.points[ publishId ].g = m_mapColorPoints.m_ColorPointsVec[ i ]->m_rgb[ 1 ];
            colorPointcloud.points[ publishId ].b = m_mapColorPoints.m_ColorPointsVec[ i ]->m_rgb[ 0 ];
            // pc_rgb.points[i].intensity = m_map_rgb_pts.m_rgb_pts_vec[i]->m_obs_dis;
            publishId++;
            if ( publishId == numberPointsPerTopic )
            {
                publishId = 0;
                pcl::toROSMsg( colorPointcloud, rosPointcloudMsg );
                rosPointcloudMsg.header.frame_id = "world";       
                rosPointcloudMsg.header.stamp = ros::Time::now(); 
                if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )
                {
                    m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
                        std::make_shared< ros::Publisher >( m_ros_node_handle.advertise< sensor_msgs::PointCloud2 >(
                            std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
                }
                m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( rosPointcloudMsg );
                std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
                ros::spinOnce();
                cur_topic_idx++;
            }
        }

        colorPointcloud.resize( publishId );
        pcl::toROSMsg( colorPointcloud, rosPointcloudMsg );
        rosPointcloudMsg.header.frame_id = "world";       
        rosPointcloudMsg.header.stamp = ros::Time::now(); 
        if ( m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] == nullptr )
        {
            m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ] =
                std::make_shared< ros::Publisher >( m_ros_node_handle.advertise< sensor_msgs::PointCloud2 >(
                    std::string( "/RGB_map_" ).append( std::to_string( cur_topic_idx ) ), 100 ) );
        }
        std::this_thread::sleep_for( std::chrono::microseconds( sleep_time_aft_pub ) );
        ros::spinOnce();
        m_pub_rgb_render_pointcloud_ptr_vec[ cur_topic_idx ]->publish( rosPointcloudMsg );
        cur_topic_idx++;
        if ( cur_topic_idx >= 45 ) // Maximum pointcloud topics = 45.
        {
            numberPointsPerTopic *= 1.5;
            sleep_time_aft_pub *= 1.5;
        }
    }
}

void IMUFusion::publishRenderPoints( ros::Publisher &pts_pub, GlobalMap &m_map_rgb_pts )
{
    pcl::PointCloud< pcl::PointXYZRGB > colorPointcloud;
    sensor_msgs::PointCloud2            rosPointcloudMsg;
    colorPointcloud.reserve( 1e7 );
    m_map_rgb_pts.m_mutex_m_box_recent_hitted->lock();
    std::unordered_set< std::shared_ptr< RGBVoxel > > boxes_recent_hitted = m_map_rgb_pts.m_voxels_recent_visited;
    m_map_rgb_pts.m_mutex_m_box_recent_hitted->unlock();

    for ( VoxelSetIterator it = boxes_recent_hitted.begin(); it != boxes_recent_hitted.end(); it++ )
    {
        for ( int pt_idx = 0; pt_idx < ( *it )->m_pts_in_grid.size(); pt_idx++ )
        {
            pcl::PointXYZRGB           pt;
            std::shared_ptr< RGBPoints > rgb_pt = ( *it )->m_pts_in_grid[ pt_idx ];
            pt.x = rgb_pt->m_pos[ 0 ];
            pt.y = rgb_pt->m_pos[ 1 ];
            pt.z = rgb_pt->m_pos[ 2 ];
            pt.r = rgb_pt->m_rgb[ 2 ];
            pt.g = rgb_pt->m_rgb[ 1 ];
            pt.b = rgb_pt->m_rgb[ 0 ];
            if ( rgb_pt->m_N_rgb > m_pub_pt_minimum_views )
            {
                colorPointcloud.points.push_back( pt );
            }
        }
    }
    pcl::toROSMsg( colorPointcloud, rosPointcloudMsg );
    rosPointcloudMsg.header.frame_id = "world";       // world; camera_init
    rosPointcloudMsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
    pts_pub.publish( rosPointcloudMsg );
}

// ANCHOR -  service_VIO_update
void IMUFusion::serviceVIOUpdate()
{
    // Init cv windows for debug
    m_optialTracker.setIntrinsic( g_cam_K, g_cam_dist * 0, cv::Size( m_vioImageWidth / m_vio_scale_factor, m_vioImageHeight / m_vio_scale_factor ) );
    m_optialTracker.m_maximum_vio_tracked_pts = m_maximum_vio_tracked_pts;
    m_mapColorPoints.m_minimum_depth_for_projection = m_tracker_minimum_depth;
    m_mapColorPoints.m_maximum_depth_for_projection = m_tracker_maximum_depth;
    Common_tools::Timer tim;
    cv::Mat             img_get;
    while ( ros::ok() )
    {
        while ( g_camera_lidar_queue.m_if_have_lidar_data == 0 )
        {
            ros::spinOnce();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
            std::this_thread::yield();
            continue;
        }

        if ( m_queue_image_with_pose.size() == 0 )
        {
            ros::spinOnce();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
            std::this_thread::yield();
            continue;
        }
        m_camera_data_mutex.lock();
        while ( m_queue_image_with_pose.size() > m_maximumImageBuffer )
        {
            cout << ANSI_COLOR_BLUE_BOLD << "=== Pop image! current queue size = " << m_queue_image_with_pose.size() << " ===" << ANSI_COLOR_RESET
                 << endl;
            m_optialTracker.trackImg( m_queue_image_with_pose.front(), -20 );
            m_queue_image_with_pose.pop_front();
        }

        std::shared_ptr< ImageFrame > img_pose = m_queue_image_with_pose.front();
        double                             message_time = img_pose->m_timestamp;
        m_queue_image_with_pose.pop_front();
        m_camera_data_mutex.unlock();
        g_camera_lidar_queue.m_last_visual_time = img_pose->m_timestamp + g_lio_state.td_ext_c2i;

        img_pose->set_frame_idx( g_camera_frame_idx );
        tim.tic( "Frame" );

        if ( g_camera_frame_idx == 0 )
        {
            std::vector< cv::Point2f >                pts_2d_vec;
            std::vector< std::shared_ptr< RGBPoints > > rgb_pts_vec;
            // while ( ( m_map_rgb_pts.is_busy() ) || ( ( m_map_rgb_pts.m_rgb_pts_vec.size() <= 100 ) ) )
            while ( ( ( m_mapColorPoints.m_ColorPointsVec.size() <= 100 ) ) )
            {
                ros::spinOnce();
                std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
            }
            setImagePose( img_pose, g_lio_state ); // For first frame pose, we suppose that the motion is static.
            m_mapColorPoints.selection_points_for_projection( img_pose, &rgb_pts_vec, &pts_2d_vec, m_trackWindowsSize / m_vio_scale_factor );
            m_optialTracker.init( img_pose, rgb_pts_vec, pts_2d_vec );
            g_camera_frame_idx++;
            continue;
        }

        g_camera_frame_idx++;
        tim.tic( "Wait" );
        while ( g_camera_lidar_queue.if_camera_can_process() == false )
        {
            ros::spinOnce();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
            std::this_thread::yield();
        }
        g_cost_time_logger.record( tim, "Wait" );
        m_mutex_lio_process.lock();
        tim.tic( "Frame" );
        tim.tic( "Track_img" );
        StatesGroup state_out;
        m_cam_measurement_weight = std::max( 0.001, std::min( 5.0 / m_number_of_new_visited_voxel, 0.01 ) );
        if ( vioPreintegration( g_lio_state, state_out, img_pose->m_timestamp + g_lio_state.td_ext_c2i ) == false )
        {
            m_mutex_lio_process.unlock();
            continue;
        }
        // state_out = g_lio_state;
        setImagePose( img_pose, state_out );

        m_optialTracker.trackImg( img_pose, -20 );
        g_cost_time_logger.record( tim, "Track_img" );
        // cout << "Track_img cost " << tim.toc( "Track_img" ) << endl;
        tim.tic( "Ransac" );

        std::cout << "before vio preintegration !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        StatesGroup::display(g_lio_state);

        std::cout << "after vio preintegration !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        StatesGroup::display(state_out);

        // ANCHOR -  remove point using PnP.
        if ( m_optialTracker.removeOutlierRansac( img_pose ) == 0 )
        {
            cout << ANSI_COLOR_RED_BOLD << "****** Remove_outlier_using_ransac_pnp error*****" << ANSI_COLOR_RESET << endl;
        }
        g_cost_time_logger.record( tim, "Ransac" );
        tim.tic( "Vio_f2f" );
        bool res_esikf = true, res_photometric = true;
        wait_render_thread_finish();

        
        res_esikf = vioEsikf( state_out, m_optialTracker );
        std::cout << "after esikf !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        StatesGroup::display(state_out);

        g_cost_time_logger.record( tim, "Vio_f2f" );
        tim.tic( "Vio_f2m" );
        res_photometric = vioPhotometric(state_out, m_optialTracker, img_pose );
        g_cost_time_logger.record( tim, "Vio_f2m" );
        g_lio_state = state_out;
        print_dash_board();
        setImagePose( img_pose, state_out );

        if ( 1 )
        {
            tim.tic( "Render" );
            // m_map_rgb_pts.render_pts_in_voxels(img_pose, m_last_added_rgb_pts_vec);
            if ( 1 ) // Using multiple threads for rendering
            {
                m_mapColorPoints.m_if_get_all_pts_in_boxes_using_mp = 0;
                // m_map_rgb_pts.render_pts_in_voxels_mp(img_pose, &m_map_rgb_pts.m_rgb_pts_in_recent_visited_voxels,
                // img_pose->m_timestamp);
                m_render_thread = std::make_shared< std::shared_future< void > >( m_thread_pool_ptr->commit_task(
                    render_pts_in_voxels_mp, img_pose, &m_mapColorPoints.m_voxels_recent_visited, img_pose->m_timestamp ) );
            }
            else
            {
                m_mapColorPoints.m_if_get_all_pts_in_boxes_using_mp = 0;
                // m_map_rgb_pts.render_pts_in_voxels( img_pose, m_map_rgb_pts.m_rgb_pts_in_recent_visited_voxels,
                // img_pose->m_timestamp );
            }
            m_mapColorPoints.m_last_updated_frame_idx = img_pose->m_frame_idx;
            g_cost_time_logger.record( tim, "Render" );

            tim.tic( "Mvs_record" );
            if ( m_if_record_mvs )
            {
                // m_mvs_recorder.insert_image_and_pts( img_pose, m_map_rgb_pts.m_voxels_recent_visited );
            }
            g_cost_time_logger.record( tim, "Mvs_record" );
        }
        // ANCHOR - render point cloud
        dump_lio_state_to_log( m_lio_state_fp );
        m_mutex_lio_process.unlock();
        // cout << "Solve image pose cost " << tim.toc("Solve_pose") << endl;
        m_mapColorPoints.update_pose_for_projection( img_pose, -0.4 );
        m_optialTracker.update_and_append_track_pts( img_pose, m_mapColorPoints, m_trackWindowsSize / m_vio_scale_factor, 1000000 );
        g_cost_time_logger.record( tim, "Frame" );
        double frame_cost = tim.toc( "Frame" );
        g_image_vec.push_back( img_pose );
        frame_cost_time_vec.push_back( frame_cost );
        if ( g_image_vec.size() > 10 )
        {
            g_image_vec.pop_front();
            frame_cost_time_vec.pop_front();
        }
        tim.tic( "Pub" );
        double display_cost_time = std::accumulate( frame_cost_time_vec.begin(), frame_cost_time_vec.end(), 0.0 ) / frame_cost_time_vec.size();
        g_vio_frame_cost_time = display_cost_time;
        publishRenderPoints( m_pub_render_rgb_pts, m_mapColorPoints );
        publishCameraOdom( img_pose, message_time );
        // publish_track_img( op_track.m_debug_track_img, display_cost_time );
        publishTrackImg( img_pose->m_raw_img, display_cost_time );
        publishTrackPoints(m_optialTracker);

        if ( m_if_pub_raw_img )
        {
            publishRawImg( img_pose->m_raw_img );
        }

        if ( g_camera_lidar_queue.m_if_dump_log )
        {
            g_cost_time_logger.flush();
        }
        // cout << "Publish cost time " << tim.toc("Pub") << endl;
    }
}
