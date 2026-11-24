import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('campus_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_share, 'worlds', 'umn_campus.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    models_path = os.path.join(pkg_share, 'models')
    rviz_config = os.path.join(pkg_share, 'rviz', 'campus.rviz')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo server-only without GUI'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with preconfigured campus displays'
    )

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')

    set_gz_resource = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
        condition=UnlessCondition(headless)
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-s -r {world_path}'}.items(),
        condition=IfCondition(headless)
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    odom_tf = Node(
        package='campus_sim',
        executable='odom_tf_broadcaster.py',
        output='screen'
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1.2', '0', '0', '0', 'wearer', 'wearer/body/chest_lidar']
    )

    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1.2', '0', '0', '0', 'wearer', 'wearer/body/imu']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    planner = Node(
        package='route_planner',
        executable='route_planner',
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        rviz_arg,
        set_gz_resource,
        gz_sim_gui,
        gz_sim_headless,
        bridge,
        odom_tf,
        lidar_static_tf,
        imu_static_tf,
        rviz_node,
        set_gz_resource,
        gz_sim,
        bridge,
        planner,
    ])
