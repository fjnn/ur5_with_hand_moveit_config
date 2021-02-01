#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg








def main():
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("hand_control_with_leap_node", anonymous=False)
        robot = moveit_commander.RobotCommander()
        
        hand_group = moveit_commander.MoveGroupCommander("hand")
        hand_group.set_named_target("handClosed")
        plan = hand_group.go()        
#        rospy.sleep(5)
        hand_group.set_named_target("handOpen")
        plan = hand_group.go()        
        rospy.sleep(5)

    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




