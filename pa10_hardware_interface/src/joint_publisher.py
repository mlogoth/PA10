#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import sin,cos,pi

def talker():
    pub = rospy.Publisher('arm_controller/command', Float64MultiArray, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        msg = Float64MultiArray()
        
#        msg.joint_names = ['finger_joint_1',finger_joint_2]
#        
#        msg.points = [JointTrajectoryPoint()]
#        msg.data = [0.4*sin(2*pi*now.nsecs),0.4*cos(2*pi*now.nsecs),0.4*sin(pi*now.nsecs),0.4*cos(2*pi*now.nsecs),0.4*sin(2*pi*now.nsecs),0.4*cos(2*pi*now.nsecs),0.4*sin(pi*now.nsecs)]
        msg.data = [-0.1,-0.1,0.1,-0.1,0.1,-0.1,0.1]
#        msg.points[0].velocities=[0.0 for i in range(0,7)]
#        msg.points[0].accelerations = [0.0 for i in range(0,7)]
#        msg.points[0].effort = [0.0 for i in range(0,7)]
#        msg.points[0].time_from_start = now
        
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

