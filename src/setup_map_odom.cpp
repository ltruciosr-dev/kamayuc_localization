#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <vector>

class TFSetup
{
private:
    ros::Publisher ekf_pub_;
    ros::Subscriber ekf_sub_;
    ros::NodeHandle nh_;

    tf2_ros::Buffer tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";
    std::string robot_frame_ = "base_link";

    nav_msgs::Odometry map_odom_;

public:
    TFSetup(ros::NodeHandle &nh) : nh_{nh}
    {
        // Initialize buffer
        tfListener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);

        // Subscribers/Publishers
        ekf_sub_ = nh.subscribe<nav_msgs::Odometry>("ekf_local/odometry", 1, 
        &TFSetup::CoreCallback, this);
        ekf_pub_ = nh.advertise<nav_msgs::Odometry>("ekf_local/odometry/map", 1);
    }

    void CoreCallback(const nav_msgs::Odometry odom)
    {
        geometry_msgs::TransformStamped mapTf;
        geometry_msgs::Pose odom_pose, map_pose;
    
        // header setup
        map_odom_.header.stamp = odom.header.stamp;
        map_odom_.header.frame_id = map_frame_;
        map_odom_.child_frame_id = robot_frame_;

        map_odom_.pose.covariance = odom.pose.covariance;
        
        // odom to pose
        odom_pose = odom.pose.pose;

        try
        {
            mapTf = tfBuffer_.lookupTransform(map_frame_, odom_frame_, ros::Time(0));
            tf2::doTransform(odom_pose, map_pose, mapTf);
            map_odom_.pose.pose = map_pose;
        }

        catch (tf2::TransformException &ex)
        {
            map_odom_.pose.pose = odom_pose;
        }
    
        ekf_pub_.publish(map_odom_);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "setup_tf_map");
    ros::NodeHandle nh;
    TFSetup core(nh);
    ros::spin();

    return 0;
}