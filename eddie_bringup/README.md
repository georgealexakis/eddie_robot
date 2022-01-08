# Eddie bringup

This package launches eddie's robot motors, robot monitoring, robot localization and the attached sensors. Eddie robot is equipped with multiple sensors for research and is used for multiple experiments.

## Required packages

The required packages depend on the robot setup.
List of packages:
* joint_state_publisher
* robot_state_publisher
* robot_pose_publisher
* tf2_web_republisher
* robot_localization
* web_video_server
* rosbridge_server
* rviz
* rosserial_python
* imu_bno055
* velodyne_pointcloud
* piksi_multi_rtk_ros

## Cases

There are 4 different launch file that are developed for different cases.

* eddie_bringup.launch: Bring up only the model and the basic operations (monitoring, urdf, rosbridge).
* eddie_bringup_lidar_imu.launch: Bring up with the lidar, IMU and the fusion with the encoders to improve localization.
* eddie_bringup_lidar_t265.launch: Bring up with the lidar, t265 camera for odometry. 
* eddie_bringup_multiple_sensors.launch: Bring up with the lidar, gps, and imu, encoders for odometry.

## Future Implementation

GPS navigation. 

## Links

* https://navigation.ros.org/setup_guides/odom/setup_odom.html: Odometry
* http://docs.ros.org/en/melodic/api/robot_localization/html/index.html: Odometry
* https://roscon.ros.org/2015/presentations/robot_localization.pdf: Odometry
