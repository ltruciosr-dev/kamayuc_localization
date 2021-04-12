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
    std::vector<ros::Publisher> pub_markers_;
    ros::Subscriber sub_markers_;
    ros::NodeHandle nh_;

    std::string rover_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    int n_markers;

    tf2_ros::Buffer tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    geometry_msgs::TransformStamped RoverToMap_;
    std::vector<geometry_msgs::TransformStamped> markers_pose_;

public:
    ARMapping(ros::NodeHandle &nh) : nh_{nh}
    {
        // Parameters
        nh_.param<std::string>("robot_frame", rover_frame_, "base_link");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<int>("num_landmarks", n_markers, 15);

        // Buffer
        tfListener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);

        // Publisers/Subscribers
        sub_markers_ = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1, &ARMapping::CoreCallback, this);

        pub_markers_.reserve(n_markers);
        markers_pose_.reserve(n_markers);
        for (int idx = 0; idx < n_markers; ++idx)
        {
            std::string marker_frame = "landmark_" + std::to_string(idx + 1);
            std::string marker_topic = "/landmark_" + std::to_string(idx + 1) + "/pose";
            pub_markers_.push_back(nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(marker_topic, 1));
            markers_pose_.push_back(tfBuffer_.lookupTransform(map_frame_, marker_frame, ros::Time(0), ros::Duration(0.5)));
        }
    }

    geometry_msgs::PoseWithCovarianceStamped GetMarkerPose(const ar_track_alvar_msgs::AlvarMarker &marker, const geometry_msgs::TransformStamped &marker_static_tf)
    {
        RoverToMap_.transform.translation.x = 0;
        RoverToMap_.transform.translation.y = 0;
        RoverToMap_.transform.translation.z = 0;

        // Reading landmark position from robot in robot frame
        geometry_msgs::Pose marker_pose_robot, marker_pose_map;
        marker_pose_robot.position = marker.pose.pose.position;
        tf2::doTransform(marker_pose_robot, marker_pose_map, RoverToMap_);

        // Compute estimated robot pose from map
        geometry_msgs::PoseWithCovarianceStamped robot_pose;
        robot_pose.header.stamp = marker.header.stamp;
        robot_pose.header.frame_id = map_frame_;
        robot_pose.pose.pose.position.x = marker_static_tf.transform.translation.x - marker_pose_map.position.x;
        robot_pose.pose.pose.position.y = marker_static_tf.transform.translation.y - marker_pose_map.position.y;
        robot_pose.pose.pose.position.z = marker_static_tf.transform.translation.z - marker_pose_map.position.z;
        robot_pose.pose.pose.orientation = RoverToMap_.transform.rotation;

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
            RoverToMap_ = tfBuffer_.lookupTransform(map_frame_, rover_frame_, ros::Time(0), ros::Duration(0.5));

            for (const auto &marker : data.markers)
            {
                try
                {
                    if (marker.id < 1 and marker.id > n_markers)
                        continue;
                    auto marker_index = marker.id - 1;
                    geometry_msgs::PoseWithCovarianceStamped pose = GetMarkerPose(marker, markers_pose_[marker_index]);
                    pub_markers_[marker_index].publish(pose);

                    // DEBUG
                    auto distance = std::sqrt(marker.pose.pose.position.x * marker.pose.pose.position.x +
                                              marker.pose.pose.position.y * marker.pose.pose.position.y);
                    std::cout << "marker_id: " << marker.id << std::endl;
                    std::cout << "confidence: " << marker.confidence << std::endl;
                    std::cout << std::setprecision(3) << "distance: " << distance << std::endl;
                    std::cout << "---------------------------" << std::endl;
                }
                catch (const std::exception &e)
                {
                    std::cout << "ERROR" << '\n';
                    std::cout << e.what() << '\n';
                }
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
