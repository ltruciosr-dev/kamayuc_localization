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
        ekf_pub_ = nh.advertise<nav_msgs::Odometry>("ekf_global/init_odom", 1);
    }

    void publish_tf()
    {
        geometry_msgs::TransformStamped mapTf;
        // header setup
        map_odom_.header.stamp = ros::Time(0);
        map_odom_.header.frame_id = map_frame_;
        map_odom_.child_frame_id = odom_frame_;

        try
        {
            mapTf = tfBuffer_.lookupTransform(map_frame_, odom_frame_, ros::Time(0));
            map_odom_.pose.pose.position.x = mapTf.transform.translation.x;
            map_odom_.pose.pose.position.y = mapTf.transform.translation.y;
            map_odom_.pose.pose.position.z = mapTf.transform.translation.z;
        }

        catch (tf2::TransformException &ex)
        {
            map_odom_.pose.pose.position.x = 0.0;
            map_odom_.pose.pose.position.y = 0.0;
            map_odom_.pose.pose.position.z = 0.0;
        }
        map_odom_.pose.pose.orientation.x = 0.0;
        map_odom_.pose.pose.orientation.y = 0.0;
        map_odom_.pose.pose.orientation.z = 0.0;
        map_odom_.pose.pose.orientation.w = 1.0;

        ekf_pub_.publish(map_odom_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_tf_map");
    ros::NodeHandle nh;
    TFSetup core(nh);
    ros::Rate rate(30);

    while (ros::ok())
    {
        core.publish_tf();
        rate.sleep();
    }
    
    return 0;
}