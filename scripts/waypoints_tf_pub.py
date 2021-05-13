#!/usr/bin/env python
import rospy
import rospkg
import tf
import tf2_ros
import pandas as pd
import geometry_msgs

def read_waypoints(path_csv):
    df = pd.read_csv(path_csv, index_col='number')
    return df

def waypoint_tf(df_waypoints, idx):
    waypoint_tfstamped = geometry_msgs.msg.TransformStamped()
    waypoint_frame = "waypoint_"+str(idx)

    if idx==5:
        waypoint_frame = "waypoint_5a"
    elif idx==6:
        waypoint_frame = "waypoint_5b"

    waypoint_tfstamped.header.stamp = rospy.Time.now()
    waypoint_tfstamped.header.frame_id = "map"
    waypoint_tfstamped.child_frame_id = waypoint_frame

    offset = [-15, 3, 0]

    waypoint_tfstamped.transform.translation.x = float(df_waypoints["x"][idx] + offset[0])
    waypoint_tfstamped.transform.translation.y = float(df_waypoints["y"][idx] + offset[1])
    waypoint_tfstamped.transform.translation.z = float(df_waypoints["z"][idx] + offset[2])
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    waypoint_tfstamped.transform.rotation.x = quat[0]
    waypoint_tfstamped.transform.rotation.y = quat[1]
    waypoint_tfstamped.transform.rotation.z = quat[2]
    waypoint_tfstamped.transform.rotation.w = quat[3]
    return waypoint_tfstamped


if __name__ == '__main__':
    rospy.init_node('wp_static_tf2_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # Number of transforms
    path_csv = rospy.get_param('~csv', '')

    # Initialize tf /map -> /marker<x>
    df_waypoints = read_waypoints(path_csv)

    waypoints_tf = []
    for idx in df_waypoints.index.to_list():
        waypoints_tf.append(waypoint_tf(df_waypoints, idx))

    broadcaster.sendTransform(waypoints_tf)
    rospy.spin()
