#pragma once
#include <unordered_set>
#include "tools_logger.hpp"
#include "opencv2/opencv.hpp"
#include "RPAImageFrame.hpp"
#include "tools_kd_hash.hpp"
#include "tools_thread_pool.hpp"
#include "tools_serialization.hpp"
#include "common_tools.h"
// #include "assert.h"
#define R3LIVE_MAP_MAJOR_VERSION 1
#define R3LIVE_MAP_MINOR_VERSION 0
extern cv::RNG g_rng;

// extern std::atomic< long > g_pts_index;
class RGBPoints
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#if 0
    std::atomic<double> m_pos[3];
    std::atomic<double> m_rgb[3];
    std::atomic<double> m_cov_rgb[3];
    std::atomic<double> m_gray;
    std::atomic<double> m_cov_gray;
    std::atomic<int> m_N_gray;
    std::atomic<int> m_N_rgb;
#else
    double m_pos[ 3 ] = { 0 };
    double m_rgb[ 3 ] = { 0 };
    double m_cov_rgb[ 3 ] = { 0 };
    double m_gray = 0;
    double m_cov_gray = 0;
    int    m_N_gray = 0;
    int    m_N_rgb = 0;
    int    m_pt_index = 0;
#endif
    vec_2      m_img_vel;
    vec_2      m_img_pt_in_last_frame;
    vec_2      m_img_pt_in_current_frame;
    int        m_is_out_lier_count = 0;
    cv::Scalar m_dbg_color;
    double     m_obs_dis = 0;
    double     m_last_obs_time = 0;
    void       clear()
    {
        m_rgb[ 0 ] = 0;
        m_rgb[ 1 ] = 0;
        m_rgb[ 2 ] = 0;
        m_gray = 0;
        m_cov_gray = 0;
        m_N_gray = 0;
        m_N_rgb = 0;
        m_obs_dis = 0;
        m_last_obs_time = 0;
        int r = g_rng.uniform( 0, 256 );
        int g = g_rng.uniform( 0, 256 );
        int b = g_rng.uniform( 0, 256 );
        m_dbg_color = cv::Scalar( r, g, b );
        // m_rgb = vec_3(255, 255, 255);
    };

    RGBPoints()
    {
        // m_pt_index = g_pts_index++;
        clear();
    };
    ~RGBPoints(){};

    void set_pos( const vec_3 &pos );
    vec_3          get_pos();
    vec_3          get_rgb();
    mat_3_3        get_rgb_cov();
    pcl::PointXYZI get_pt();
    void update_gray( const double gray, double obs_dis = 1.0 );
    int update_rgb( const vec_3 &rgb, const double obs_dis, const vec_3 obs_sigma, const double obs_time );

  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        ar &m_pos;
        ar &m_rgb;
        ar &m_pt_index;
        ar &m_cov_rgb;
        ar &m_gray;
        ar &m_N_rgb;
        ar &m_N_gray;
    }
};
using RGBPointsPtr = std::shared_ptr< RGBPoints >;

class RGBVoxel
{
  public:
    std::vector< RGBPointsPtr > m_pts_in_grid;
    double                    m_last_visited_time = 0;
    RGBVoxel() = default;
    ~RGBVoxel() = default;
    void add_pt( RGBPointsPtr &rgb_pts ) { m_pts_in_grid.push_back( rgb_pts ); }
};

using RGBVoxelPtr = std::shared_ptr< RGBVoxel >;
using VoxelSetIterator = std::unordered_set< std::shared_ptr< RGBVoxel > >::iterator;

struct GlobalMap
{
    int                                                          m_map_major_version = R3LIVE_MAP_MAJOR_VERSION;
    int                                                          m_map_minor_version = R3LIVE_MAP_MINOR_VERSION;
    int                                                          m_if_get_all_pts_in_boxes_using_mp = 1;
    std::vector< RGBPointsPtr >                    m_rgb_pts_vec;
    // std::vector< RGB_pt_ptr >                    m_rgb_pts_in_recent_visited_voxels;
    std::shared_ptr< std::vector< RGBPointsPtr> >                  m_pts_rgb_vec_for_projection = nullptr;
    std::shared_ptr< std::mutex >                                m_mutex_pts_vec;
    std::shared_ptr< std::mutex >                                m_mutex_recent_added_list;
    std::shared_ptr< std::mutex >                                m_mutex_img_pose_for_projection;
    std::shared_ptr< std::mutex >                                m_mutex_rgb_pts_in_recent_hitted_boxes;
    std::shared_ptr< std::mutex >                                m_mutex_m_box_recent_hitted;
    std::shared_ptr< std::mutex >                                m_mutex_pts_last_visited;
    ImageFrame                                              m_img_for_projection;
    double                                                       m_recent_visited_voxel_activated_time = 0.0;
    bool                                                         m_in_appending_pts = 0;
    int                                                          m_updated_frame_index = 0;
    std::shared_ptr< std::thread >                               m_thread_service;
    int                                                          m_if_reload_init_voxel_and_hashed_pts = true;

    HashMap3D< long, RGBPointsPtr >   m_hashmap_3d_pts;
    HashMap3D< long, std::shared_ptr< RGBVoxel > > m_hashmap_voxels;
    std::unordered_set< std::shared_ptr< RGBVoxel > > m_voxels_recent_visited;
    std::vector< std::shared_ptr< RGBPoints > >          m_pts_last_hitted;
    double                                   m_minimum_pts_size = 0.05; // 5cm minimum distance.
    double                                   m_voxel_resolution = 0.1;
    double                                   m_maximum_depth_for_projection = 200;
    double                                   m_minimum_depth_for_projection = 3;
    int                                      m_last_updated_frame_idx = -1;
    void                                     clear();
    void set_minmum_dis( double minimum_dis );

    GlobalMap( int if_start_service = 1 );
    ~GlobalMap();

    void service_refresh_pts_for_projection();
    void render_points_for_projection( std::shared_ptr< ImageFrame > &img_ptr );
    void update_pose_for_projection( std::shared_ptr< ImageFrame > &img, double fov_margin = 0.0001 );
    bool is_busy();
    template < typename T >
    int append_points_to_global_map( pcl::PointCloud< T > &pc_in, double  added_time,  std::vector< RGBPointsPtr > *pts_added_vec = nullptr, int step = 1 );
    void render_with_a_image( std::shared_ptr< ImageFrame > &img_ptr, int if_select = 1 );
    void selection_points_for_projection( std::shared_ptr< ImageFrame > &image_pose, std::vector< std::shared_ptr< RGBPoints > > *pc_out_vec = nullptr,
                                          std::vector< cv::Point2f > *pc_2d_out_vec = nullptr, double minimum_dis = 5, int skip_step = 1,int use_all_pts = 0 );
    void save_to_pcd( std::string dir_name, std::string file_name = std::string( "/rgb_pt" ) , int save_pts_with_views = 3);
    void save_and_display_pointcloud( std::string dir_name = std::string( "/home/ziv/temp/" ), std::string file_name = std::string( "/rgb_pt" ) ,  int save_pts_with_views = 3);
    void render_pts_in_voxels( std::shared_ptr< ImageFrame > &img_ptr, std::vector< std::shared_ptr< RGBPoints > > &voxels_for_render, double obs_time = 0 );

  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        boost::serialization::split_free( ar, *this, version );
    }
};




void render_pts_in_voxels_mp( std::shared_ptr< ImageFrame > &img_ptr, std::unordered_set< RGBVoxelPtr > *voxels_for_render, const double &obs_time = 0 );

template < typename Archive >
inline void save( Archive &ar, const GlobalMap &global_map, const unsigned int /*version*/ )
{
    int vector_size;
    vector_size = global_map.m_rgb_pts_vec.size();
    ar << global_map.m_map_major_version;
    ar << global_map.m_map_minor_version;
    ar << global_map.m_minimum_pts_size;
    ar << global_map.m_voxel_resolution;
    ar << vector_size;
    cout << ANSI_COLOR_YELLOW_BOLD;
    for ( int i = 0; i < vector_size; i++ )
    {
        ar << ( *global_map.m_rgb_pts_vec[ i ] );
        CV_Assert( global_map.m_rgb_pts_vec[ i ]->m_pt_index == i );
        if ( ( i % 10000 == 0 ) || ( i == vector_size - 1 ) )
        {
            cout << ANSI_DELETE_CURRENT_LINE << "Saving global map: " << ( i * 100.0 / (vector_size-1) ) << " %";
            ANSI_SCREEN_FLUSH;
        }
    }
    cout << endl;
}

template < typename Archive >
inline void load( Archive &ar, GlobalMap &global_map, const unsigned int /*version*/ )
{
    Common_tools::Timer tim;
    tim.tic();
    int vector_size;
    vector_size = global_map.m_rgb_pts_vec.size();
    ar >> global_map.m_map_major_version;
    ar >> global_map.m_map_minor_version;
    ar >> global_map.m_minimum_pts_size;
    ar >> global_map.m_voxel_resolution;
    ar >> vector_size;
    int grid_x, grid_y, grid_z, box_x, box_y, box_z;
    scope_color( ANSI_COLOR_YELLOW_BOLD );
    for ( int i = 0; i < vector_size; i++ )
    {
        // printf_line;
        std::shared_ptr< RGBPoints > rgb_pt = std::make_shared< RGBPoints >();
        ar >> *rgb_pt;
        CV_Assert( rgb_pt->m_pt_index == global_map.m_rgb_pts_vec.size() );
        global_map.m_rgb_pts_vec.push_back( rgb_pt );
        grid_x = std::round( rgb_pt->m_pos[ 0 ] / global_map.m_minimum_pts_size );
        grid_y = std::round( rgb_pt->m_pos[ 1 ] / global_map.m_minimum_pts_size );
        grid_z = std::round( rgb_pt->m_pos[ 2 ] / global_map.m_minimum_pts_size );
        box_x = std::round( rgb_pt->m_pos[ 0 ] / global_map.m_voxel_resolution );
        box_y = std::round( rgb_pt->m_pos[ 1 ] / global_map.m_voxel_resolution );
        box_z = std::round( rgb_pt->m_pos[ 2 ] / global_map.m_voxel_resolution );

        if ( global_map.m_if_reload_init_voxel_and_hashed_pts )
        {
            // if init voxel and hashmap_3d_pts, comment this to save loading time if necessary.
            global_map.m_hashmap_3d_pts.Insert( grid_x, grid_y, grid_z, rgb_pt );
            if ( !global_map.m_hashmap_voxels.IfExist( box_x, box_y, box_z ) )
            {
                std::shared_ptr< RGBVoxel > box_rgb = std::make_shared< RGBVoxel >();
                global_map.m_hashmap_voxels.Insert( box_x, box_y, box_z, box_rgb );
            }
            global_map.m_hashmap_voxels.m_map_3d_hash_map[ box_x ][ box_y ][ box_z ]->add_pt( rgb_pt );
        }
        if ( ( i % 10000 == 0 ) || ( i == vector_size - 1 ) )
        {
            cout << ANSI_DELETE_CURRENT_LINE << "Loading global map: " << ( i * 100.0 / (vector_size-1) ) << " %";
            ANSI_SCREEN_FLUSH;
        }
    }
    cout << endl;
    cout << "Load offine global map cost: " << tim.toc() << " ms" << ANSI_COLOR_RESET << endl;
}
