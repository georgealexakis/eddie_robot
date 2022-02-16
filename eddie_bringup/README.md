# Eddie bringup

This package launches eddie's robot motors, robot monitoring, robot localization and the attached sensors. Eddie robot is equipped with multiple sensors for research and is used for multiple experiments. This is a package for research and experiments.

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
* ros_imu_bno055
* velodyne_pointcloud
* piksi_multi_rtk_ros

## Installation of packages

```
$ sudo apt-get install libgeographic-dev
$ sudo apt-get install geographiclib-tools

$ sudo apt-get install ros-melodic-robot-localization
$ sudo apt-get install ros-melodic-move-base
$ sudo apt-get install ros-melodic-web-video-server
$ sudo apt-get install ros-melodic-rosbridge-server
$ sudo apt-get install ros-melodic-tf2-web-republisher
$ sudo apt-get install ros-melodic-teleop-twist-joy
$ sudo apt-get install ros-melodic-joint-state-publisher
$ sudo apt-get install ros-melodic-robot-state-publisher
$ sudo apt-get install ros-melodic-velodyne
$ sudo apt-get install ros-melodic-rviz-imu-plugin
```

```
$ cd catkin_ws/src
$ git clone https://github.com/GT-RAIL/robot_pose_publisher.git
$ cd ../
$ catkin_make
```
## Realsense installation

Follow the instructions: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

## IMU BNO055 setup with serial communication installation

`$ sudo apt-get install python3-pip`

Follow the instructions: https://automaticaddison.com/tag/bno055/

## Piksi RTK GPS installation process in Jetson TX2/Jetson Nano

The installation process for Jetson boards is a lit bit tricky so the instructions are helpful for the future installations. Follow the instructions: https://support.swiftnav.com/support/solutions/articles/44001907919-using-ros-with-swift-navigation-gnss-devices

For the git clone process follow the commands below:

Issue retrieved from: https://github.com/ethz-asl/ethz_piksi_ros/issues/122
```
$ git clone https://github.com/ethz-asl/ethz_piksi_ros.git
$ cd ethz_piksi_ros
$ git checkout v1.11.0
```

Install script ./install_piksi_multi.sh will fail to install some packages so do it manually following the commands below:

```
$ pip install sbp==2.6.5
$ sudo pip install numpy-quaternion==2021.8.30.10.33.11
$ sudo apt-get install python-pyproj
$ sudo apt-get install python-llvmlite
$ sudo apt-get install python-numba
$ sudo apt-get install python-llvmlite
```

## Cases

There are 4 different launch file that are developed for different cases.

* eddie_bringup.launch: Bring up only the model and the basic operations (monitoring, URDF, rosbridge).
* eddie_bringup_lidar_imu.launch: Bring up with the lidar, IMU and the fusion with the encoders to improve localization.
* eddie_bringup_lidar_t265.launch: Bring up with the lidar, t265 camera for odometry. 
* eddie_bringup_multiple_sensors.launch: Bring up with the lidar, GPS, and IMU, encoders for odometry.

## Links

* Odometry: https://navigation.ros.org/setup_guides/odom/setup_odom.html
* Odometry: http://docs.ros.org/en/melodic/api/robot_localization/html/index.html
* Odometry: https://roscon.ros.org/2015/presentations/robot_localization.pdf