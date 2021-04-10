# Kamayuc Localization

Localization stack for the navigation task of the Kamayuc/Leo rover.

## Launching

Publish landmarks position into ROS tf tree.
```
roslaunch kamayuc_localization landmark_tf_pub.launch
```

Run EKF local filter (continous measurements).
```
roslaunch kamayuc_localization ekf_local.launch
```

Run EKF global filter (landmarks detection).
```
roslaunch kamayuc_localization ekf_global.launch
```

Let's validate that your tf scheme is completely chained (map -> odom -> base_link) and (map -> landmark_<`x`>)
```
rosrun rqt_tf_tree rqt_tf_tree
```

Estimate landmarks position and compare with the provided real position.
```
roslaunch kamayuc_localization artag_track_2D.launch
```

Transform landmark estimation to the `/map -> /base_link` frames.
```
roslaunch kamayuc_localization artag_pub.launch
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
    
    Current linear and angular velocities of the robot estimated from zed2 camera consecutive frames.

* **`ar_tag/marker_<x>`** ([geometry_msgs/PoseStamped])

    Current linear and angular velocities of the robot estimated from wheel velocities.

### Published topics

* **`ekf_global/odometry`** ([nav_msgs/Odometry])

    Set frame relation between the `/map` and `/base_link` tf's.

## TODO

- [ ] Add rover real position.
- [ ] Setup /map -> /odom tf transformation (with initial zero-values).
- [ ] Add covariance matrix to /wheel_odom (TwistStamped) and publish in topic /leo_odom (TwistWithCovarianceStamped).
- [ ] Global EKF localization about landmarks. [Global EKF](https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/02%20-%20Global%20Pose%20Estimate%20Fusion%20(Example%20Implementation).md)

[nav_msgs/Odometry]: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[geometry_msgs/PoseStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
