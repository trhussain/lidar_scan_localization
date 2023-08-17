# lidar_scan_localization

## Running the package
- Clone the repo
- colcon build --package-select lidar_scan
- source install/setup.bash
- ros2 launch lidar_scan lidar.launch.py

It should automatically open up RVIZ. If not, you may open the provided RVIZ config file within the github with

- ros2 run rviz2 rviz2 -d <path/to/config.rviz>

In another terminal launch the bag file. Do not play rosbag in loop mode, timing issue between rosbag time and sim time will occur

- ros2 bag play <rosbag.db3>

In another terminal run
- ros2 launch slam_toolbox online_async_launch.py

This runs NAV2's SLAM toolbox in ASYNC mode. There is another mode, SYNC, I believe. This may be more applicable to recorded data, unsure.


If frames start to dissapear, stop the rosbag, hit reset in the bottom left corner of RVIZ, and replay ROSBAG

What it should look like:

![Screenshot from 2023-08-15 20-47-47](https://github.com/trhussain/lidar_scan_localization/assets/89410479/c0b76cf8-9bf0-4696-8e14-22e85af7adad)

## Code explanation:

### Package Structure

A launch file lidar.launch.py is utilized to launch required nodes, topics, and other ROS tools.
Within lidar.launch.py we have one method.



![image](https://github.com/trhussain/lidar_scan_localization/assets/89410479/61ae040e-adc3-47b1-b0b3-8ad74bc4151e)


At the top the path variables I declare are for different models, applications, and packages I want accesible.
**ISSUE:** I should be able to declare a relative path of say wamv.urdf or robot.rviz, but for some reason it errored when they were within a folder. So I left
them in the main directory and it worked fine.

Below I have node declarations and access to whichever package I wanted that node to execute. Funny thing, these aren't all ROS 'nodes' though. When you run
ROS2 node list, you don't see the ones I actually return to launch in the list. Only a few of them actually apppear there. So I don't know why they are labeled as nodes.

### TF-Frames
The are a few frames that were missing which NAV2 required. The _lidar0, base_footprint, and base_link_, frames. Lidar0 was required because the tf tree of ROS didn't have an
actual frame for lidar0, so it was having an issue with correct position generation of the lidar point clouds. Base footprint and base link were required for NAV2 to do SLAM I believe.

**NOTE:** I used the static publisher package to declare these frames as there were simply static frames. This is not the correct way to do this. This was just for testing purposes.
I believe the standard way to do this is to do so in a URDF file. This was done later on by Parissa


### PCL2->LaserScan
The second thing was creating the PCL2 to LS converter


![image](https://github.com/trhussain/lidar_scan_localization/assets/89410479/ab8a3ec7-1e6a-4a5f-9590-9e2926d7dda2)


**IMPORTANT:**In order to generate a occupancy grid, the SLAM toolbox takes in messages from the /scan topic and outputs data to the /map topic. The message type of
that data has to be a laserscan message. I initially tried to manually convert it (bad) and then found a package that does that transform for you. HOWEVER, there are a lot
of parameters that you would want to change. I left it as default for all, but these parameters may be desired to change. ([Package](http://wiki.ros.org/pointcloud_to_laserscan))

![image](https://github.com/trhussain/lidar_scan_localization/assets/89410479/d9243f79-c4c1-4804-afa0-87fe4e90352e)



### Localization
**INCOMPLETE:**

![image](https://github.com/trhussain/lidar_scan_localization/assets/89410479/000d6509-d6bf-4c8c-8230-53573bc0ffa5)

Robot state publisher and joint status publisher were for the URDF Model.

The Robot localization node calls the EKF Filter and launches that as a _node_.


### Return launchdescription
![image](https://github.com/trhussain/lidar_scan_localization/assets/89410479/8eb7f2d2-2b9a-4696-a9d2-f6f1a1ca67fc)

This launches any listed nodes within its parameters. The launch arguments spawn entities or applications once the ROS pacakge is generated.

I don't believe the map_topic launch argument was working correctly.


### setup.py

![image](https://github.com/trhussain/lidar_scan_localization/assets/89410479/1c088763-fb8a-4824-a4a4-7b8b9767f655)

The 'share/' allows the datafiles to include particular files that I wanted, like the rviz config file, the urdf model, and my EKF node.
This is a **necessary** step to allow the launch file to access necessary files.

Also inputting any packages you made into the console_scripts parameters is important.

### EKF

This is a fusion of IMU and Odom data. I mainly referenced off of the source YAML file for this and the example on NAV2s page.


![image](https://github.com/trhussain/lidar_scan_localization/assets/89410479/9ef23a9b-d5a8-46de-8891-179c9d4ecdef)
This is what each point in the matrix represents.

The ekf_exp.yaml file shows the example code of all the params you can set for a ekf.



### General Overview
When you run the current state of the code there are a few main things that happen
TF Frames
1. Lidar 0, Base_Link, Base_footprint
   
The base frames are necessary for the SLAM toolbox to figure out localization and mapping. The lidar frame to let
tf actually position the lidar

2. PCL2 -> Lasercloud message type conversion and publishing to /scan topic

The SLAM toolbox takes in information, whatever is on the /scan toolbox, anded. publishes to the /map topic. This is done
within the launch file with the pcl2->laserscan package. 
In RVIZ you can see the occupancy grid with the occupancy grid topic display

3. URDF Model - **incomplete**

It has no physical mesh properties but just visual for now. Parissa focused on this so she'll
go more in depth on it in hers. It also places the TF frames _overriding_ the frames declared within the launch file from the static broadcaster I believe, so you should be able to delete those static publishers from the launch file but I would specifically test this

4. EKF Node - **incomplete**

This is the current step necessary for proper localization. Previously, we were running the lidar data bag files
without the stereo camera. However, when stereo camera is introduced there is new odom & IMU data. (meaning velocity, position, roll, pitch, yaw, etc, acceleration) Now, the information _exists_ but I don't quite know how RVIZ is using it. When I ran my code _without_ the ekf node, it went haywire. So it was somehow accessing the odom & IMU data in some manner. I launched the EKF node from the launch file and it launched correctly but the way it was processing data was incorrect. I believe this was due to an issue with sim-time vs real-time data as when we ran it briefly on live position didn't go haywire, but further testing needed

By haywire I mean the lidar0 frame spawned _very_ far away from the origin of RVIZ. 

The main thing necessary to fully operate autonomously (to my understanding and according to NAV-2) is to complete the first time robot setup 
according NAV-2s first time robot setup guide (https://navigation.ros.org/setup_guides/transformation/setup_transforms.html). 





