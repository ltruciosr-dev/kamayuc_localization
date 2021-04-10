#! /usr/bin/env python
import rospy
import time
import tf
from gazebo_msgs.srv import GetModelState

class TF_Broadcaster(object):

    def __init__(self):
        rospy.init_node("tf_transform")

    def Get_pose_in_gazebo(self):                                                           #Funcion para llamar al service para obtener posicion y angulo
        model_coordinates=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)       #Service para obtener coordenadas
        object_coordinates= model_coordinates("robot_mov","")                               #Llamar posicion del robot_mov           
        return object_coordinates                                   #Guardar variables en una lista Pose                                  

    def odom_pose(self):
        data=self.Get_pose_in_gazebo()
        br=tf.TransformBroadcaster()
        br.sendTransform((data.pose.position.x,data.pose.position.y,data.pose.position.z),
                        (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w),
                        rospy.Time.now(),
                        "/odom",
                        "/world")        
                        
if __name__=="__main__":
    Tf=TF_Broadcaster()
    while not rospy.is_shutdown():
        Tf.odom_pose()