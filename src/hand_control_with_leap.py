#! /usr/bin/env python

"""
It looks like I will use move_groups for EE mapping and action server for joint space mapping
TODO: This whole thing can be written in classes. My_Move_Groups(). Don't overcode yet :D
"""

import sys
import time

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState

from Classes.IMU_class_full_arm import IMUsubscriber
from Classes.leap_class import LeapSubscriber



#IMU = IMUsubscriber()
#Leap = LeapSubscriber()


def movegroup_init():
    moveit_commander.roscpp_initialize(sys.argv)
#    rospy.init_node("hand_control_with_leap_node", anonymous=False)
    robot = moveit_commander.RobotCommander()
    
    hand_group = moveit_commander.MoveGroupCommander("hand")
    arm_group = moveit_commander.MoveGroupCommander("arm")
    hand_group.set_named_target("handOpen")
    plan_hand = hand_group.go()  
    arm_group.set_named_target("home")
    plan_arm = arm_group.go()  
    return hand_group, arm_group
    

def task_space_map():
    """
    Here I will use predefined poses for robot to move.
    Move group will be used like that:
        
#        hand_group.set_named_target("handOpen")
#        plan_hand = hand_group.go()  
#        arm_group.set_named_target("home")
#        plan_arm = arm_group.go()  
 
    """
    pass


def rt_ee_mapping():
    """
    Not predefined poses in move group
    Real time hand position w.r.t torso will be mapped ee position w.r.t /base_link
    """
    pass


def rt_joints_mapping(hand_group, arm_group):
    """
    IMU readings will be mapped real time.
    Not move groups but action-server clients will be used
    """
    arm_group.clear_pose_targets()
    hand_group.clear_pose_targets()
    
    arm_group_variable_values = arm_group.get_current_joint_values()
    print "============ Arm joint values: ", arm_group_variable_values
    hand_group_variable_values = hand_group.get_current_joint_values()
    print "============ Hand joint values: ", hand_group_variable_values
    print "click Enter to continue"
    dummy_input = raw_input()
    arm_group_variable_values[4] = 1.0
    arm_group.set_joint_value_target(arm_group_variable_values)
    
    plan = arm_group.plan()
    plan_arm = arm_group.go()  
    
    dummy_input = raw_input()
    


def main():
    try:
        hand_group, arm_group = movegroup_init()
#        rospy.sleep(5)
        rt_joints_mapping(hand_group, arm_group)
        sys.exit("done")
#        IMU.init_subscribers_and_publishers()
#        
#        hand_group.set_named_target("handOpen")
#        plan_hand = hand_group.go()  
#        arm_group.set_named_target("home")
#        plan_arm = arm_group.go()  
#        Leap.init_subscribers_and_publishers()
        
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
#            Leap.update()
#            Leap.r.sleep()

    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




