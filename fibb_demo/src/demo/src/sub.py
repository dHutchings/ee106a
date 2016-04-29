#!/usr/bin/env python

import rospy
import time
from demo.msg import FibMsg


def callback(msg):
    time.sleep(3) #to make sure that it doesn't run away...  has to be before the publish
    to_send = FibMsg()
    to_send.n_minus_1 = msg.n
    to_send.n = msg.n + msg.n_minus_1
    pub.publish(to_send)



if __name__ == '__main__':
    
    rospy.init_node('fib_calculator')
    sub = rospy.Subscriber('/Fib_topic',FibMsg,callback,queue_size = 1)
    pub = rospy.Publisher('/Fib_topic',FibMsg,queue_size = 1)

    rospy.spin()
