#! /usr/bin/env python
import rospy
import time
import tf
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState

class TF_Broadcaster(object):
    def __init__(self):
        rospy.init_node("tf_transform")
        self.pub = rospy.Publisher("/gazebo_odom", Odometry, queue_size = 1)
        
        # Initialize rover odom:
        self.rover_init_odom = Odometry() 
        self.rover_init_odom = self.init_odom()

    def init_odom(self):
        model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rover_gazebo = model("leo", "")

        rover_odom = Odometry()
        
        rover_odom.header =  rover_gazebo.header
        rover_odom.pose.pose = rover_gazebo.pose

        return rover_odom

    def get_odom(self, frame_id = "map"):
        model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rover_gazebo = model("leo", "")

        rover_odom = Odometry()
        
        rover_odom.header =  rover_gazebo.header
        rover_odom.header.frame_id = frame_id
        rover_odom.pose.pose.position = rover_gazebo.pose.position
        rover_odom.pose.pose.orientation = rover_gazebo.pose.orientation
        rover_odom.twist.twist = rover_gazebo.twist

        return rover_odom
        

    def odom_pose(self):
        # Get odom
        rover_odom = self.get_odom()
        self.pub.publish(rover_odom)

        # Broadcast
        br = tf.TransformBroadcaster()
        br.sendTransform((rover_odom.pose.pose.position.x, rover_odom.pose.pose.position.y, rover_odom.pose.pose.position.z),
                         (rover_odom.pose.pose.orientation.x, rover_odom.pose.pose.orientation.y, rover_odom.pose.pose.orientation.z, rover_odom.pose.pose.orientation.w),
                          rospy.Time.now(),
                          "gazebo_odom",
                          "map")        
                        
if __name__=="__main__":
    Tf = TF_Broadcaster()
    rate = rospy.Rate(30) # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        Tf.odom_pose()
        rate.sleep()