#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("hand_control_with_leap_node", anonymous=False)
robot = moveit_commander.RobotCommander()

hand_group = moveit_commander.MoveGroupCommander("hand")
hand_group.set_named_target("handOpen")
plan1 = hand_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()


#import rospy
#from std_msgs.msg import String
#
#def talker():
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()
#
#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass






