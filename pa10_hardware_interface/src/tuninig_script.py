#!/usr/bin/env python
# license removed for brevity
import rospy
import argparse
from std_msgs.msg import String
from std_msgs.msg import Float64
from math import cos, sin, pi
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
if __name__ == '__main__':
    try:
        
        # Description of parser
        parser = argparse.ArgumentParser(description='Ros controllers tuning script.')
        
        parser.add_argument("-j", "--joint", type=int, default=1, 
                              help='the num of the joint to be tuned')

        parser.add_argument("-a", "--amplitude" , type=float, default=0.5, 
                              help='the amplitude of the equation')
        args = parser.parse_args()
        
        print "Tuning of joint {} control mode.".format(args.joint)
        
        # topic
        topic = "arm_controller/command"
        print "Topic Name: /{}".format(topic)
        
        pub = rospy.Publisher(topic, JointTrajectory, queue_size=100)
        
        
        
        
        rospy.init_node('talker', anonymous=True)
        
        jointcmds = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        
        rate = rospy.Rate(10)
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            
            
            jointCmd = JointTrajectory()  
            point = JointTrajectoryPoint()
            jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
            point.time_from_start = rospy.Duration.from_sec(0.5)

            jointCmd.joint_names.append('S1')
            jointCmd.joint_names.append('S2')
            jointCmd.joint_names.append('E1')
            jointCmd.joint_names.append('E2')
            jointCmd.joint_names.append('W1')
            jointCmd.joint_names.append('W2')
            jointCmd.joint_names.append('W3')
            
            jointcmds[args.joint-1] = args.amplitude#*sin(0.2*pi*t)
            
            for i in range(0, 7):
                point.positions.append(jointcmds[i])
                point.velocities.append(0)
                point.accelerations.append(0)
                point.effort.append(0) 
            jointCmd.points.append(point)
            #rospy.loginfo(hello_str)
            pub.publish(jointCmd)
            rate.sleep()
    except rospy.ROSInterruptException:
        msg = Float64()
        msg.data = 0.0
        pub.publish(msg)
        rate.sleep()
        pass
