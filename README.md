# Kamayuc Localization

Localization stack for the navigation task of the Kamayuc/Leo rover.

## Launching

Publish landmarks position into ROS tf tree.
```
roslaunch kamayuc_localization pub_tf_artag.launch
```

Publish rover position extracted from gazebo, it's the real position.
```
roslaunch kamayuc_localization pub_tf_rover.launch
```

Visual Odom.
```
roslaunch kamayuc_perception visual_odom.launch
```

Run EKF local filter (`/wheel_odom`, `/visual_odom` and `/zed2/imu`).
```
roslaunch kamayuc_localization ekf_local.launch
```

Run EKF global filter (landmarks detection).
```
roslaunch kamayuc_localization ekf_global.launch
```

Once all the nodes are runing, let's validate that your tf tree is completely chained:
- map -> odom -> base_link  
- map -> landmark_<`x`>
```
rosrun rqt_tf_tree rqt_tf_tree
```

Artag 3D
```
roslaunch kamayuc_perception artag_track_3D.launch
```

Transform landmark estimation (`perception stack`) to the `/map -> /base_link` frame.
```
roslaunch kamayuc_localization map_artag_ekf.launch
```

## ROS API

### 1. Local-EKF

### Subscribed topics

* **`visual_odom`** ([nav_msgs/Odometry])
    
    Current linear and angular velocities of the robot estimated from zed2 camera consecutive frames.

* **`wheel_odom`** ([geometry_msgs/TwistStamped])
    
    Current linear and angular velocities of the robot estimated from wheel velocities.

* **`imu/data`** ([sensor_msgs/Imu])
    
    Accelerometer, gyroscope, and orientation data from ZED2 IMU.

### Published topics

* **`ekf_local/odometry`** ([nav_msgs/Odometry])

    Set frame relation between the `/odom` and `/base_link` tf's.

### 2. Global-EKF

### Subscribed topics

* **`ekf_local/odometry/map`** ([nav_msgs/Odometry])
    
    Current linear and angular velocities of the robot estimated from local ekf.

* **`ar_tag/marker_<x>`** ([geometry_msgs/PoseStamped])

    Current linear and angular velocities of the robot estimated from wheel velocities.

### Published topics  gaa

* **`ekf_global/odometry`** ([nav_msgs/Odometry])

    Set frame relation between the `/map` and `/base_link` tf's.


[nav_msgs/Odometry]: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[geometry_msgs/PoseStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
