#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def moveJoint (jointcmds,nbJoints):
  topic_name = 'arm_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)

  jointCmd.joint_names.append('E1')
  jointCmd.joint_names.append('E2')
  jointCmd.joint_names.append('S1')
  jointCmd.joint_names.append('S2')
  jointCmd.joint_names.append('W1')
  jointCmd.joint_names.append('W2')
  jointCmd.joint_names.append('W3')
  
  for i in range(0, nbJoints):
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(50)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

def moveFingers (jointcmds,nbJoints):
  topic_name = 'gripper_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append('finger_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

if __name__ == '__main__':
  try:    
    rospy.init_node('Move_PA10_home_position')		
    rospy.sleep(10)

    #home robots
    moveJoint ([0.0,0.0,0.0,0.0,0.0,0.0,0.0],7)
    moveFingers ([0.0,0.0],2)
  except rospy.ROSInterruptException:
    print "program interrupted before completion"
