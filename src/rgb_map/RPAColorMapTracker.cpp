#include "RPAColorMapTracker.hpp"
#include "lib_sophus/so3.hpp"

ColorMapTracker::ColorMapTracker()
{
    cv::TermCriteria criteria = cv::TermCriteria( ( cv::TermCriteria::COUNT ) + ( cv::TermCriteria::EPS ), 10, 0.05 );
    if ( m_lk_optical_flow_kernel == nullptr )
    {
        m_lk_optical_flow_kernel = std::make_shared< LK_optical_flow_kernel >( cv::Size( 21, 21 ), 3, criteria,
                                                                               cv_OPTFLOW_LK_GET_MIN_EIGENVALS );
    }
}

void ColorMapTracker::update_and_append_track_pts( std::shared_ptr< ImageFrame > &img_pose, GlobalMap &map_rgb,
                                                  double mini_dis, int minimum_frame_diff )
{
    Common_tools::Timer tim;
    tim.tic();
    double                    u_d, v_d;
    int                       u_i, v_i;
    double                    max_allow_repro_err = 2.0 * img_pose->m_img_cols / 320.0;
    Hash_map_2d< int, float > map_2d_pts_occupied;

    for ( auto it = m_map_rgb_pts_in_last_frame_pos.begin(); it != m_map_rgb_pts_in_last_frame_pos.end(); )
    {
        RGBPoints *rgb_pt = ( ( RGBPoints * ) it->first );
        vec_3    pt_3d = ( ( RGBPoints * ) it->first )->get_pos();
        int      res = img_pose->project_3d_point_in_this_img( pt_3d, u_d, v_d, nullptr, 1.0 );
        u_i = std::round( u_d / mini_dis ) * mini_dis;
        v_i = std::round( v_d / mini_dis ) * mini_dis;

        double error = vec_2( u_d - it->second.x, v_d - it->second.y ).norm();

        if ( error > max_allow_repro_err )
        {
            // cout << "Remove: " << vec_2(it->second.x, it->second.y).transpose() << " | " << vec_2(u, v).transpose()
            // << endl;
            rgb_pt->m_is_out_lier_count++;
            if ( rgb_pt->m_is_out_lier_count > 1 || ( error > max_allow_repro_err * 2 ) )
            // if (rgb_pt->m_is_out_lier_count > 3)
            {
                rgb_pt->m_is_out_lier_count = 0; // Reset
                it = m_map_rgb_pts_in_last_frame_pos.erase( it );
                continue;
            }
        }
        else
        {
            rgb_pt->m_is_out_lier_count = 0;
        }

        if ( res )
        {
            double depth = ( pt_3d - img_pose->m_pose_w2c_t ).norm();
            if ( map_2d_pts_occupied.IfExist( u_i, v_i ) == false )
            {
                map_2d_pts_occupied.insert( u_i, v_i, depth );
                // it->second = cv::Point2f(u, v);
            }
        }
        else
        {
            // m_map_rgb_pts_in_last_frame_pos.erase(it);
        }
        it++;
    }

    int new_added_pts = 0;

    tim.tic( "Add" );
    while ( map_rgb.m_updated_frame_index < img_pose->m_frame_idx - minimum_frame_diff )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        continue;
    }
    map_rgb.m_mutex_pts_vec->lock();
    int new_add_pt = 0;

    if ( map_rgb.m_pts_rgb_vec_for_projection != nullptr )
    {
        int pt_size = map_rgb.m_pts_rgb_vec_for_projection->size();
        for ( int i = 0; i < pt_size; i++ )
        {
            if ( m_map_rgb_pts_in_last_frame_pos.find( ( *map_rgb.m_pts_rgb_vec_for_projection )[ i ].get() ) !=
                 m_map_rgb_pts_in_last_frame_pos.end() )
            {
                continue;
            }
            vec_3 pt_3d = ( *map_rgb.m_pts_rgb_vec_for_projection )[ i ]->get_pos();
            int   res = img_pose->project_3d_point_in_this_img( pt_3d, u_d, v_d, nullptr, 1.0 );
            u_i = std::round( u_d / mini_dis ) * mini_dis;
            v_i = std::round( v_d / mini_dis ) * mini_dis;
            // vec_3 rgb_color = img_pose->get_rgb(u, v);
            // double grey = img_pose->get_grey_color(u, v);
            // (*map_rgb.m_pts_rgb_vec_for_projection)[i]->update_gray(grey);
            // (*map_rgb.m_pts_rgb_vec_for_projection)[i]->update_rgb(rgb_color);
            if ( res )
            {
                double depth = ( pt_3d - img_pose->m_pose_w2c_t ).norm();
                if ( map_2d_pts_occupied.IfExist( u_i, v_i ) == false )
                {
                    map_2d_pts_occupied.insert( u_i, v_i, depth );
                    m_map_rgb_pts_in_last_frame_pos[ ( *map_rgb.m_pts_rgb_vec_for_projection )[ i ].get() ] =
                        cv::Point2f( u_d, v_d );
                    new_added_pts++;
                }
            }
            new_add_pt++;
            if ( m_map_rgb_pts_in_last_frame_pos.size() >= m_maximum_vio_tracked_pts )
            {
                break;
            }
        }
        // cout << "Tracker new added pts = " << new_added_pts << " |  " << map_rgb.m_pts_rgb_vec_for_projection->size()
        //      << " | " << img_pose->m_frame_idx << " | " << map_rgb.m_updated_frame_index
        //      << ", cost = " << tim.toc("Add") << endl;
    }

    map_rgb.m_mutex_pts_vec->unlock();
    updateLastTrackingVectorAndIds();
    // cout << "Update points cost time = " << tim.toc() << endl;
}

void ColorMapTracker::reject_error_tracking_pts( std::shared_ptr< ImageFrame > &img_pose, double dis )
{
    double u, v;
    int    remove_count = 0;
    int    total_count = m_map_rgb_pts_in_current_frame_pos.size();
    // cout << "Cam mat: " <<img_pose->m_cam_K << endl;
    // cout << "Image pose: ";
    // img_pose->display_pose();
    scope_color( ANSI_COLOR_BLUE_BOLD );
    for ( auto it = m_map_rgb_pts_in_current_frame_pos.begin(); it != m_map_rgb_pts_in_current_frame_pos.end(); it++ )
    {
        cv::Point2f predicted_pt = it->second;
        vec_3       pt_3d = ( ( RGBPoints * ) it->first )->get_pos();
        int         res = img_pose->project_3d_point_in_this_img( pt_3d, u, v, nullptr, 1.0 );
        if ( res )
        {
            if ( ( fabs( u - predicted_pt.x ) > dis ) || ( fabs( v - predicted_pt.y ) > dis ) )
            {
                // Remove tracking pts
                m_map_rgb_pts_in_current_frame_pos.erase( it );
                remove_count++;
            }
        }
        else
        {
            // cout << pt_3d.transpose() << " | ";
            // cout << "Predicted: " << vec_2(predicted_pt.x, predicted_pt.y).transpose() << ", measure: " << vec_2(u,
            // v).transpose() << endl;
            m_map_rgb_pts_in_current_frame_pos.erase( it );
            remove_count++;
        }
    }
    cout << "Total pts = " << total_count << ", rejected pts = " << remove_count << endl;
}

// inline void image_equalize(cv::Mat &img, int amp)
// {
//     cv::Mat img_temp;
//     cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
//     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
//     // Equalize gray image.
//     clahe->apply(img, img_temp);
//     img = img_temp;
// }

// inline cv::Mat equalize_color_image_ycrcb(cv::Mat &image)
// {
//     cv::Mat hist_equalized_image;
//     cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);

//     //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
//     std::vector<cv::Mat> vec_channels;
//     cv::split(hist_equalized_image, vec_channels);

//     //Equalize the histogram of only the Y channel
//     // cv::equalizeHist(vec_channels[0], vec_channels[0]);
//     image_equalize( vec_channels[0], 2 );
//     cv::merge(vec_channels, hist_equalized_image);
//     cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
//     return hist_equalized_image;
// }

void ColorMapTracker::trackImg( std::shared_ptr< ImageFrame > &img_pose, double dis, int if_use_opencv )
{
    Common_tools::Timer tim;
    m_current_frame = img_pose->m_img;
    m_current_frame_time = img_pose->m_timestamp;
    m_map_rgb_pts_in_current_frame_pos.clear();
    if ( m_current_frame.empty() )
    {
        std::cout << "current frame is empty" << std::endl; 
        return;
    }
    cv::Mat frame_gray = img_pose->m_img_gray;
    tim.tic( "HE" );
    tim.tic( "opTrack" );
    std::vector< uchar > status;
    std::vector< float > err;
    m_current_tracked_pts = m_lastTrackedPoints;
    int before_track = m_lastTrackedPoints.size();
    if ( m_lastTrackedPoints.size() < 30 )
    {
        m_last_frame_time = m_current_frame_time;
        std::cout << "m last tracked pts is"<<m_lastTrackedPoints.size() <<  "less than 30" << std::endl; 
        return;
    }

    m_lk_optical_flow_kernel->track_image( frame_gray, m_lastTrackedPoints, m_current_tracked_pts, status, 2 );
    reduce_vector( m_lastTrackedPoints, status );
    reduce_vector( m_oldIds, status );
    reduce_vector( m_current_tracked_pts, status );

    int     after_track = m_lastTrackedPoints.size();
    cv::Mat mat_F;

    tim.tic( "Reject_F" );
    unsigned int pts_before_F = m_lastTrackedPoints.size();
    mat_F = cv::findFundamentalMat( m_lastTrackedPoints, m_current_tracked_pts, cv::FM_RANSAC, 1.0, 0.997, status );
    unsigned int size_a = m_current_tracked_pts.size();
    reduce_vector( m_lastTrackedPoints, status );
    reduce_vector( m_oldIds, status );
    reduce_vector( m_current_tracked_pts, status );

    m_map_rgb_pts_in_current_frame_pos.clear();
    double frame_time_diff = ( m_current_frame_time - m_last_frame_time );
    std::cout << "m_last_tracked_pts size " << m_lastTrackedPoints.size() << std::endl;
    for ( uint i = 0; i < m_lastTrackedPoints.size(); i++ )
    {
        if ( img_pose->if_2d_points_available( m_current_tracked_pts[ i ].x, m_current_tracked_pts[ i ].y, 1.0, 0.05 ) )
        {
            RGBPoints *rgb_pts_ptr = ( ( RGBPoints * ) m_colorPointsPtrVecInLastFrame[ m_oldIds[ i ] ] );
            m_map_rgb_pts_in_current_frame_pos[ rgb_pts_ptr ] = m_current_tracked_pts[ i ];
            cv::Point2f pt_img_vel = ( m_current_tracked_pts[ i ] - m_lastTrackedPoints[ i ] ) / frame_time_diff;
            rgb_pts_ptr->m_img_pt_in_last_frame = vec_2( m_lastTrackedPoints[ i ].x, m_lastTrackedPoints[ i ].y );
            rgb_pts_ptr->m_img_pt_in_current_frame =
                vec_2( m_current_tracked_pts[ i ].x, m_current_tracked_pts[ i ].y );
            rgb_pts_ptr->m_img_vel = vec_2( pt_img_vel.x, pt_img_vel.y );
        }
    }

    if ( dis > 0 )
    {
        reject_error_tracking_pts( img_pose, dis );
    }

    m_old_gray = frame_gray.clone();
    m_old_frame = m_current_frame;
    m_map_rgb_pts_in_last_frame_pos = m_map_rgb_pts_in_current_frame_pos;
    updateLastTrackingVectorAndIds();

    m_frame_idx++;
    m_last_frame_time = m_current_frame_time;
}

int ColorMapTracker::getAllTrackedPoints( std::vector< std::vector< cv::Point2f > > *img_pt_vec )
{
    int hit_count = 0;
    for ( auto it = m_map_id_pts_vec.begin(); it != m_map_id_pts_vec.end(); it++ )
    {
        if ( it->second.size() == m_frame_idx )
        {
            hit_count++;
            if ( img_pt_vec )
            {
                img_pt_vec->push_back( it->second );
            }
        }
    }
    cout << "Total frame " << m_frame_idx;
    cout << ", success tracked points = " << hit_count << endl;
    return hit_count;
}

int ColorMapTracker::removeOutlierRansac( std::shared_ptr< ImageFrame > &img_pose, int if_remove_ourlier )
{
    Common_tools::Timer tim;
    tim.tic();

    cv::Mat                    r_vec, t_vec;
    cv::Mat                    R_mat;
    vec_3                      eigen_r_vec, eigen_t_vec;
    std::vector< cv::Point3f > pt_3d_vec, pt_3d_vec_selected;
    std::vector< cv::Point2f > pt_2d_vec, pt_2d_vec_selected;
    std::vector< void * >      map_ptr_vec;

    std::cout << "m_map_rgb_pts_in_current_frame_pos " << m_map_rgb_pts_in_current_frame_pos.size() << std::endl;
    for ( auto it = m_map_rgb_pts_in_current_frame_pos.begin(); it != m_map_rgb_pts_in_current_frame_pos.end(); it++ )
    {
        map_ptr_vec.push_back( it->first );
        vec_3 pt_3d = ( ( RGBPoints * ) it->first )->get_pos();
        pt_3d_vec.push_back( cv::Point3f( pt_3d( 0 ), pt_3d( 1 ), pt_3d( 2 ) ) );
        pt_2d_vec.push_back( it->second );
    }
    if ( pt_3d_vec.size() < 10 )
    {
        std::cout << "pt 3d vec size " << pt_3d_vec.size() << std::endl;
        return 0;
    }
    if ( 1 )
    {
        std::vector< int > status;
        try
        {
            // cv::solvePnPRansac( pt_3d_vec, pt_2d_vec, m_intrinsic, cv::Mat(), r_vec, t_vec, false, 200, 1.5, 0.99,
            //                     status ); // SOLVEPNP_ITERATIVE
            cv::solvePnPRansac( pt_3d_vec, pt_2d_vec, m_intrinsic, cv::Mat(), r_vec, t_vec, false, 200, 5, 0.99,
                                status ); // SOLVEPNP_ITERATIVE
        }
        catch ( cv::Exception &e )
        {
            scope_color( ANSI_COLOR_RED_BOLD );
            cout << "Catching a cv exception: " << e.msg << endl;
            return 0;
        }
        if ( if_remove_ourlier )
        {
            // Remove outlier
            m_map_rgb_pts_in_last_frame_pos.clear();
            m_map_rgb_pts_in_current_frame_pos.clear();
            for ( unsigned int i = 0; i < status.size(); i++ )
            {
                int inlier_idx = status[ i ];
                {
                    m_map_rgb_pts_in_last_frame_pos[ map_ptr_vec[ inlier_idx ] ] = pt_2d_vec[ inlier_idx ];
                    m_map_rgb_pts_in_current_frame_pos[ map_ptr_vec[ inlier_idx ] ] = pt_2d_vec[ inlier_idx ];
                }
            }
        }
        updateLastTrackingVectorAndIds();
    }

    // cv::solvePnP(pt_3d_vec, pt_2d_vec, m_intrinsic, m_dist_coeffs * 0, r_vec, t_vec);

    cv::cv2eigen( r_vec, eigen_r_vec );
    cv::cv2eigen( t_vec, eigen_t_vec );
    // eigen_q solver_q = Sophus::SO3d::exp(eigen_r_vec).unit_quaternion().inverse();
    // eigen_q solver_q = Sophus::SO3d::exp( eigen_r_vec ).unit_quaternion().inverse();
    // vec_3   solver_t = ( solver_q * eigen_t_vec ) * -1.0;
    eigen_q solver_q = Sophus::SO3d::exp( eigen_r_vec ).unit_quaternion();
    vec_3   solver_t = eigen_t_vec;
    std::cout << "point size : " << pt_3d_vec.size() << std::endl;
    // for (auto point : pt_3d_vec)
    // {
    //     std::cout << " ========================================3d================================================ "<< std::endl;
    //     std::cout << "  " <<point << std::endl;
    // }
    // for (auto point : pt_2d_vec)
    // {
    //     std::cout << " ========================================2d================================================ "<< std::endl;
    //     std::cout << "  " <<point << std::endl;
    // }
    cout << "Solve pose: " << solver_q.coeffs().transpose() << " | " << solver_t.transpose() << endl;
    int    if_update = 1;
    double t_diff = ( solver_t - img_pose->m_pose_w2c_t ).norm();
    double r_diff = ( solver_q ).angularDistance( img_pose->m_pose_w2c_q ) * 57.3;
    
    if_update = 1;
    t_last_estimated = solver_t;
    if ( t_diff < 3 && r_diff < 1.0 )
    {
        img_pose->m_pnp_pose_w2c_q = solver_q;
        img_pose->m_pnp_pose_w2c_t = solver_t;

        img_pose->m_pose_w2c_q = solver_q;
        img_pose->m_pose_w2c_t = solver_t;
    }
    else
    {
        img_pose->m_pnp_pose_w2c_q = img_pose->m_pose_w2c_q;
        img_pose->m_pnp_pose_w2c_t = img_pose->m_pose_w2c_t;
    }
    img_pose->refresh_pose_for_projection();
    img_pose->m_have_solved_pnp = 1;
    // cout << "Estimate pose cost time = " << tim.toc() << endl;
    return if_update;
}
