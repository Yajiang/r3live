import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
    
    # Subscribed Topics
    lidar_topic = LaunchConfiguration("LiDAR_pointcloud_topic",default="/laser_cloud_flat")
    imu_topic = LaunchConfiguration("IMU_topic",default="/livox/imu")
    image_topic = LaunchConfiguration("Image_topic",default="/camera/image_color")

    # Directory for Map Output
    map_dir = LaunchConfiguration("map_output_dir",default="$(env HOME)/r3live_output")

    # Path to Configuration File
    config_file_path = os.path.join(
        get_package_share_directory('r3live'),
        'config',
        'r3live_config_ros2.yaml'
    )

    with open(config_file_path, 'r') as f:
        params_dict = yaml.safe_load(f)

    # LiDAR Front End Node
    liadr_front_end_node = Node(
        package="r3live",
        executable="r3live_LiDAR_front_end",
        name="r3live_LiDAR_front_end",
        output="screen",
        parameters=[config_file_path,
            {"LiDAR_pointcloud_topic": lidar_topic},
            {"IMU_topic": imu_topic},
            {"Image_topic": image_topic}])

    # Mapping Node
    mapping_node = Node(
        package="r3live",
        executable="r3live_mapping",
        name="r3live_mapping",
        output="screen",
        parameters=[config_file_path,
            {"r3live_common/map_output_dir": map_dir}])
    

    # RViz Node
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="1",
        description="Launch RViz?"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory('r3live'), 'config', 'rviz', 'r3live_rviz_config_ros2.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Create launch description object
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(rviz_arg)

    # Add nodes to launch description
    ld.add_action(mapping_node)
    ld.add_action(liadr_front_end_node)
    ld.add_action(rviz_node)

    return ld

if __name__ == "__main__":
    generate_launch_description()
