#!/usr/bin/env python
import rospy
import rospkg
import tf
import tf2_ros
import pandas as pd
import geometry_msgs

def read_landmarks(path_csv):
    df = pd.read_csv(path_csv, index_col='number')
    return df

def landmark_tf(df_landmarks, idx):
    landmark_tfstamped = geometry_msgs.msg.TransformStamped()
    landmark_frame = "landmark"+str(idx)

    landmark_tfstamped.header.stamp = rospy.Time.now()
    landmark_tfstamped.header.frame_id = "map"
    landmark_tfstamped.child_frame_id = landmark_frame

    offset = [-15, 3, 0]

    landmark_tfstamped.transform.translation.x = float(df_landmarks["x"][idx] + offset[0])
    landmark_tfstamped.transform.translation.y = float(df_landmarks["y"][idx] + offset[1])
    landmark_tfstamped.transform.translation.z = float(df_landmarks["z"][idx] + offset[2])
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    landmark_tfstamped.transform.rotation.x = quat[0]
    landmark_tfstamped.transform.rotation.y = quat[1]
    landmark_tfstamped.transform.rotation.z = quat[2]
    landmark_tfstamped.transform.rotation.w = quat[3]
    return landmark_tfstamped


if __name__ == '__main__':
    rospy.init_node('ar_static_tf2_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # Number of transforms
    path_csv = rospy.get_param('~csv', '')

    # Initialize tf /map -> /marker<x>
    df_landmarks = read_landmarks(path_csv)

    landmarks_tf = []
    for idx in df_landmarks.index.to_list():
        landmarks_tf.append(landmark_tf(df_landmarks, idx))

    broadcaster.sendTransform(landmarks_tf)
    rospy.spin()
