from launch import LaunchDescription
from launch_ros.actions import Node
import launch 
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration
import os 
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='lidar_scanv2').find('lidar_scanv2')
    default_model_path = os.path.join(pkg_share, 'wamv.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'tahseen.rviz')
    
    lidar_pub = Node(
            package='lidar_scanv2',
            executable='lidar_pub'
    )
    lidar0_frame=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'lidar_0']
    )
    test_frame=Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '3', '--y', '0', '--z', '5', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'zed_2i_base_link', '--child-frame-id', 'test2']
    )
    zed_left_frame=Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '5', '--y', '0', '--z', '5', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'zed2i_base_link', '--child-frame-id', 'zed2i_right_camera_optical_frame']
    )
    base_footprint=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0.1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'lidar_0', '--child-frame-id', 'base_footprint']
    )
    pcl2_ls=Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pcl_to_laserscan',
            output='screen',
            parameters=[
                {
                    'target_frame': 'lidar_0',  # Replace 'base_link' with your target frame
                    'transform_tolerance': 0.01,   # Adjust the transform tolerance as needed
                }
            ],
            remappings=[
                #('/cloud_in', '/rgb_pointcloud'),
                ('/cloud_in', '/lidar_0/m1600/pcl2'),  # Replace with your PointCloud2 topic
                ('/scan', '/scan'),        # Replace with the desired LaserScan topic
            ],
        )
 
    
    colorMappingNode = Node(
            package='lidar_scanv2',
            executable='colorMappingNode'
    )
    fixed_math_node = Node(
        package='lidar_scanv2',
        executable='fixed_math'
    )
    base_link=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '2', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'base_link']
    )
    odom_node=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'ekf.yaml')],
       
      
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    
    odom_node = Node(
            package='lidar_scan',
            executable='pose_transform'
    )
    return LaunchDescription([
        #launch_ros.actions.SetParameter(name='use_sim_time', value=True), # attempt to solve TF_OLD_DATA error

        launch.actions.DeclareLaunchArgument('map_topic', default_value='/map',
                                            description='Occupancy grid map topic'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        #launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),

        lidar0_frame,
        joint_state_publisher_node,
        robot_state_publisher_node,
        odom_node,
        #base_link,
        spawn_entity,
        robot_localization_node,
        pcl2_ls,
        #map_node,
        #colorMappingNode
        #fixed_math_node
        #rviz_node
        #rviz_node
        #map_ned,
        #test_frame,
        #zed_left_frame,
    ])