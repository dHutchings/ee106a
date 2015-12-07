#!/usr/bin/env python
import sys
import rospy
import baxter_interface

rospy.init_node("head_panner")
head = baxter_interface.Head()
head.set_pan(float(sys.argv[1]))