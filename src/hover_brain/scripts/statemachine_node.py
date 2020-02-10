#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from hover_brain.hover_statemachine import HoverStatemachine

if __name__ == '__main__':
    rospy.init_node('hover_statemachine')
    try:
        HoverStatemachine()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass