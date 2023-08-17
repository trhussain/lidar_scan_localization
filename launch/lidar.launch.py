from launch import LaunchDescription
from launch_ros.actions import Node
import launch 
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration
import os 
from ament_index_python.packages import get_package_share_directory
'''
Lidar & RVIZ Launch Configuration File 

Within the generate_launch_description method there multiple nodes declared. Most of these nodes should be irrelevant, but 
exercise caution when deleting. 

If launched without the URDF model, these 3 frames are required 
lidar0
base_footprint
base_link 

If launched with, these 3 frames should be able to be excluded

Nav2 needs the base frames for localization & mapping purposes. The lidar0 frame is necessary for tf to actually have one to reference off of. 

If launched with the URDF model, I believe these should be able to be excluded, but exercise caution and test it out 


The next node is necessary as it processes the lasercloud point cloud messages into laserscan messages and publishes it to the /scan topic 

NAV2 by default takes in information from the /scan topic and processes that with the SLAM toolbox and publishes the resultant data to the 
/map topic which hosts the occupancy grid information. 


The three following nodes are used for localization 

robot_localization_node
robot_state_publisher_node 
joint_state_publisher_node 

The localization node launches the EKF node that takes in IMU and Odom data. This needs work.
The robot & joint state publisher nodes are for URDF information I believe. The nav2 getting started guide 
will explain these two much better than I can. https://navigation.ros.org/setup_guides/urdf/setup_urdf.html
I didn't fully understand how those two nodes worked. 

The spawn entity node was used for gazebo spawning, but I stopped spawning the gazebo app, so should be deleted. 

The rviz node launches rviz. 





NOTE: I believe that ALL the files in lidar_scan are now irrelevant. The one I worked on was lidar_pub and pose_transform 
I tried to manually convert the pcl2 messages to laserscan, but found the package and stopped working on that.

The pose_transform script was to try and set a fixed transform of the lidar frame to keep it from going haywire 
'''
def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='lidar_scan').find('lidar_scan')
    default_model_path = os.path.join(pkg_share, 'wamv.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'robot.rviz')
    
    
    lidar0_frame=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'lidar_0']
    )
    base_footprint=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0.1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'lidar_0', '--child-frame-id', 'base_footprint']
    )
    base_link=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '2', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'base_link']
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

    lidar_pub = Node(
            package='lidar_scan',
            executable='lidar_pub'
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

        launch.actions.DeclareLaunchArgument('map_topic', default_value='/map',
                                            description='Occupancy grid map topic'), # Making sure the map topic is set to /map
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, # Ensures rviz config path is set correctly 
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', # trying to set sim_time_true but not sure if it did anything
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, # used to launch URDF model 
                                            description='Absolute path to robot urdf file'),

        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        # The above line was used to launch gazebo, kept in case you want to use it 
        
        
        lidar0_frame,
        joint_state_publisher_node,
        robot_state_publisher_node,
        odom_node,
        spawn_entity,
        robot_localization_node,
        pcl2_ls,
        rviz_node

    ])