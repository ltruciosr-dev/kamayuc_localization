//https://github.com/osrf/subt_hello_world/blob/master/subt_solution_launch/src/base_link_costmap_projector.cpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher ekf_pub;
ros::Subscriber ekf_sub;
tf2_ros::Buffer tfBuffer;

const std::string map_frame = "map";
const std::string robot_frame = "base_link";
const std::string odom_frame = "odom";

void ekf_tf(const nav_msgs::Odometry &odom);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flattening_node");
    ros::NodeHandle nh;

    std::string input_topic = "ekf_local/odometry";
    std::string output_topic = "ekf_local/odometry/map";

    tf2_ros::TransformListener tfListener(tfBuffer);

    // Subscribers
    ekf_pub = nh.advertise<nav_msgs::Odometry>(output_topic, 1);
    ekf_sub = nh.subscribe(input_topic, 1, &ekf_tf);

    ros::spin();
}

void ekf_tf(const nav_msgs::Odometry &odom)
{
    geometry_msgs::TransformStamped inTf, mapTf;
    geometry_msgs::Pose inPose, outPose;
    nav_msgs::Odometry outOdom;

    // Get inPose from inTf
    inTf = tfBuffer.lookupTransform(odom_frame, robot_frame, ros::Time(0), ros::Duration(0.5));
    inPose.position.x = inTf.transform.translation.x;
    inPose.position.y = inTf.transform.translation.y;
    inPose.position.z = inTf.transform.translation.z;
    inPose.orientation = inTf.transform.rotation;

    try
    {
        mapTf = tfBuffer.lookupTransform(map_frame, odom_frame, ros::Time(0));
        tf2::doTransform(inPose, outPose, mapTf);
        outOdom.pose.pose = outPose;
    }

    catch (tf2::TransformException &ex)
    {
        outOdom.pose.pose = inPose;
    }

    outOdom.pose.covariance = odom.pose.covariance;
    // header setup
    outOdom.header.stamp = odom.header.stamp;
    outOdom.header.frame_id = map_frame;
    outOdom.child_frame_id = robot_frame;
    ekf_pub.publish(outOdom);
}