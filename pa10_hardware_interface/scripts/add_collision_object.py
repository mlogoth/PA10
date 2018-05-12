#!/usr/bin/env python

"""
Add a collision Object In the Planning Scene
For Visualization and collision check with environment

Description:
This class adds a collision object in the planning scene in order to 
avoid it while planning using MoveIt and visualize it in RVIZ.
"""

import numpy as np
# Ros Libs
import rospy
import tf

# MoveIt Labraries
import moveit_commander

# ROS Messages
from geometry_msgs.msg import Pose, PoseStamped

class add_collision_object:
	
	def __init__(self):
		""" Initialize Collision Object Class """
		self.scene = moveit_commander.PlanningSceneInterface()
		self.robot = moveit_commander.RobotCommander()
		rospy.sleep(2)


	def add_object(self, shape = "box", frame = "base_footprint", pose = Pose(), dimensions = [0.1,0.1,0.1]):
		""" Add Collision Object to the World """
		p = PoseStamped()
		p.header.frame_id =  self.robot.get_planning_frame()
		p.pose = pose
		if (shape == "box"):
			self.scene.add_box(frame, p , dimensions)

	def remove(self):
		""" Remove All Object from Collision World """
		self.scene.remove_world_object()

