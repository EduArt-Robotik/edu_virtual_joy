#!/usr/bin/env python
# license removed for brevity
import rospy
from eduVirtualJoy import EduVirtualJoy

if __name__ == '__main__':
    rospy.init_node('edu_virtual_joy_node', anonymous=True)
    EduVirtualJoy().run()
