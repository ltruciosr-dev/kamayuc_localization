#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <memory>
#include <vector>

class ARMapping
{
private:
    ros::Publisher pub_marker1_;
    ros::Publisher pub_marker2_;
    ros::Publisher pub_marker3_;
    ros::Subscriber sub_ar_;
    ros::NodeHandle nh_;

    std::string robot_frame_;
    std::string odom_frame_;
    std::string world_frame_;
    int num_markers;

    tf2_ros::Buffer tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    geometry_msgs::TransformStamped RobotToMap_;
    std::vector<geometry_msgs::TransformStamped> markers_pose_;


public:
    ARMapping(ros::NodeHandle &nh) : nh_{nh}
    {
        // Initialize buffer
        // tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
        tfListener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);
        // Initialize subscriber
        sub_ar_ = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1,
                                                                  &ARMapping::CoreCallback, this);
        // Initialize publishers
        pub_marker1_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ar_tag/marker_1", 1);
        pub_marker2_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ar_tag/marker_2", 1);
        pub_marker3_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ar_tag/marker_3", 1);
        // Initialize parameters
        nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("world_frame", world_frame_, "map");
        nh_.param<int> ("num_landmarks", num_markers, 15);
        
        // Get markers static pose
        markers_pose_.resize(num_markers);
        std::cout << "markers size: " << markers_pose_.size() << std::endl;
        for (int idx = 0; idx < markers_pose_.size(); ++idx)
        {
            std::string marker_frame = "marker" + std::to_string(idx + 1);
            markers_pose_[idx] = tfBuffer_.lookupTransform(world_frame_, marker_frame, ros::Time(0), ros::Duration(1));
        }
    }

    geometry_msgs::PoseWithCovarianceStamped PublishPose(const ar_track_alvar_msgs::AlvarMarker &marker, const geometry_msgs::TransformStamped &marker_static)
    {
        RobotToMap_.transform.translation.x = 0;
        RobotToMap_.transform.translation.y = 0;
        RobotToMap_.transform.translation.z = 0;

        // Reading landmark position from robot in robot frame
        geometry_msgs::Pose lmk_pose_robot, lmk_pose_map;
        lmk_pose_robot.position = marker.pose.pose.position;
        tf2::doTransform(lmk_pose_robot, lmk_pose_map, RobotToMap_);

        // Compute estimated robot pose from map
        geometry_msgs::PoseWithCovarianceStamped robot_pose;
        robot_pose.header.stamp = marker.header.stamp;
        robot_pose.header.frame_id = world_frame_;
        robot_pose.pose.pose.position.x = marker_static.transform.translation.x - lmk_pose_map.position.x;
        robot_pose.pose.pose.position.y = marker_static.transform.translation.y - lmk_pose_map.position.y;
        robot_pose.pose.pose.position.z = marker_static.transform.translation.z - lmk_pose_map.position.z;
        robot_pose.pose.pose.orientation = RobotToMap_.transform.rotation;

        float pos_confidence = 0.2;
        float rot_confidence = 0.1;
        robot_pose.pose.covariance[6 * 0 + 0] = pos_confidence * pos_confidence;
        robot_pose.pose.covariance[6 * 1 + 1] = pos_confidence * pos_confidence;
        robot_pose.pose.covariance[6 * 2 + 2] = pos_confidence * pos_confidence;
        robot_pose.pose.covariance[6 * 3 + 3] = rot_confidence * rot_confidence;
        robot_pose.pose.covariance[6 * 4 + 4] = rot_confidence * rot_confidence;
        robot_pose.pose.covariance[6 * 5 + 5] = rot_confidence * rot_confidence;
        return robot_pose;
    }

    void CoreCallback(const ar_track_alvar_msgs::AlvarMarkers data)
    {
        if (!data.markers.empty())
        {
            // First read the map -> base_link transform
            RobotToMap_ = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0), ros::Duration(1.5));
            int counter = 1;
            for (const auto &marker : data.markers)
            {
                int idx = (int)marker.id;
                geometry_msgs::PoseWithCovarianceStamped pose = PublishPose(marker, markers_pose_[idx]);
                if (counter == 1)
                {
                    pub_marker1_.publish(pose);
                }
                else if (counter == 2)
                {
                    pub_marker2_.publish(pose);
                }
                else if (counter == 3)
                {
                    pub_marker3_.publish(pose);
                    break;
                }
                ++ counter;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_relay");
    ros::NodeHandle nh;
    ARMapping core(nh);
    ros::spin();
    return 0;
}
