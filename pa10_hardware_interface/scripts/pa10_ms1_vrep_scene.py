#!/usr/bin/env python

import numpy as np
from add_collision_object import *

import rospy
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    
    try:
        rospy.init_node('add_collion_objects')
        
        scene = add_collision_object()
        # Define Desired Object Position
        obj_pose = Pose()
        obj_pose.position.x = -0.6
        obj_pose.position.y = -1.2
        obj_pose.position.z = 0.25
        
        obj_pose.orientation.x = 0.0
        obj_pose.orientation.y = 0.0
        obj_pose.orientation.z = 0.0
        obj_pose.orientation.w = 1.0
        
        # Define object shape, wrt frame_id, dimensions
        scene.add_object("box", "world",obj_pose, [0.41,0.41,0.5])
        rospy.sleep(0.7)
        
        # Define Desired Object Position
        obj_pose = Pose()
        obj_pose.position.x = -0.15
        obj_pose.position.y = -2.125
        obj_pose.position.z = 2.05
        
        obj_pose.orientation.x = 0.0
        obj_pose.orientation.y = 0.0
        obj_pose.orientation.z = 0.0
        obj_pose.orientation.w = 1.0
        
        # Define object shape, wrt frame_id, dimensions
        scene.add_object("box", "world",obj_pose, [0.15,2.41,4.1]);
        rospy.sleep(0.7)
        
    except rospy.ROSInterruptException: pass
