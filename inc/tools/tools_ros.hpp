#ifndef __TOOLS_ROS_HPP__
#define __TOOLS_ROS_HPP__
#include <ros/ros.h>
#include <vector>
#include <iostream>

using std::cout;
using std::endl;

namespace Common_tools
{
template < typename T >
inline T get_ros_parameter( ros::NodeHandle &nh, const std::string parameter_name, T &parameter, T default_val )
{
    nh.param< T >( parameter_name.c_str(), parameter, default_val );
    // ENABLE_SCREEN_PRINTF;
    cout << "[Ros_parameter]: " << parameter_name << " ==> " << parameter << std::endl;
    return parameter;
}

template < typename T >
inline std::vector< T > get_ros_parameter_array( ros::NodeHandle &nh, const std::string parameter_name )
{
    std::vector< T > config_vector;
    nh.getParam( parameter_name, config_vector );
    cout << "[Ros_configurations]: " << parameter_name << ":{  ";
    for(int i =0 ; i < config_vector.size(); i++)
    {
        cout << config_vector[i] << ", ";
    }
    cout << "\b\b" << "  }" << endl;
    return config_vector;
}
}

using Common_tools::get_ros_parameter;

#endif