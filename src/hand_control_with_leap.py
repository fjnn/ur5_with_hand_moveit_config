#! /usr/bin/env python

"""
It looks like I will use move_groups for EE mapping and action server for joint space mapping
"""

import sys
import time

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState

from Classes.IMU_class_full_arm import IMUsubscriber
from Classes.leap_class import LeapSubscriber



IMU = IMUsubscriber()
Leap = LeapSubscriber()

sys.exit("done")


def movegroup_init():
    global hand_group, arm_group
    moveit_commander.roscpp_initialize(sys.argv)
#    rospy.init_node("hand_control_with_leap_node", anonymous=False)
    robot = moveit_commander.RobotCommander()
    
    hand_group = moveit_commander.MoveGroupCommander("hand")
    arm_group = moveit_commander.MoveGroupCommander("arm")
    hand_group.set_named_target("handClosed")
    plan_hand = hand_group.go()  
    arm_group.set_named_target("bend1")
    plan_arm = arm_group.go()  
    


def main():
    global hand_group, arm_group
    try:
        movegroup_init()
        rospy.sleep(5)
        IMU.init_subscribers_and_publishers()
        
        hand_group.set_named_target("handOpen")
        plan_hand = hand_group.go()  
        arm_group.set_named_target("home")
        plan_arm = arm_group.go()  
        
#        while not rospy.is_shutdown():
#            now = time.time()
#            prev = 0
#            # print "++++", index
#    #        index += 1  This index is to get 5 sensor readings and compute 1 KF estimation. Check pose_node.py in my_human_pkg for more
#            # print "total: ", (now - start)
#            # print "interval:", (now - prev), "calibration_flag:", IMU.calibration_flag
#            IMU.update()
#            IMU.r.sleep()
#            prev = now

    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




