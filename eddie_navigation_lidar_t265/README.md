# Eddie navigation

This package provides autonomous navigation for eddie robot by using a laser scanner VLP16 and odometry from realsense t265 camera.

## Configuration

For the configuration, it has been used the rqt_reconfigure tool and some official repositories as examples.

Use ``$ rosrun rqt_reconfigure rqt_reconfigure`` to configure dynamically the parameters.

## Navigation and Mapping

### Navigation without map

``$ roslaunch eddie_navigation_lidar_t265 odom_navigation_demo.launch``

### Mapping

``$ roslaunch eddie_navigation_lidar_t265 gmapping.launch``

### NAvigation with map

``$ roslaunch eddie_navigation_lidar_t265 amcl_navigation.launch map_file:=/path/to/my/map.yaml``

## Links

https://www.micro-semiconductor.it/datasheet/37-28992.pdf
https://kaiyuzheng.me/documents/navguide.pdf
https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_navigation
https://github.com/jackal/jackal/tree/melodic-devel/jackal_navigation