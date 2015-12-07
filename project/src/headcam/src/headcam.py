#!/usr/bin/env python
import sys
import rospy
# import roslib; roslib.load_manifest('kinematics')

from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import numpy as np

from tf2_msgs.msg import TFMessage
import tf
import time

import traceback
rbt = None
shift = False

usage_str = \
"""
Inverse kinematics script

Requires baxter.sh and abxter_moveit_noexec.launch to be run beforehand.
Interactive mode allows control of the end-effector position via 
text input or arrow keys.

Keyboard-interactive controls:
[UP, DOWN]      move along y-axis   Up is y+, Down is y-
[LEFT, RIGHT]   move along x-axis.  Right is x+, Left is x-
[Z, X]          move along z-axis
[G]             toggle gripper

Flags:
--help, -h      print usage instructions
-k              run interactive keyboard interface
-t              run interactive text interface
-l      tf_topic    ar_marker_number        Get transforms to a set tag using the tf service.
"""

rbt = None
last_seen = time.time()



def listener(tf_topic):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback

    print("Initializing node... ")
    rospy.init_node("head_cam")
    print("topic is " + str(tf_topic))

    rospy.Subscriber(tf_topic,TFMessage,callback)
    headpub = rospy.Publisher(tf_topic,TFMessage)
  

    global tf_listener

    tf_listener = tf.TransformListener()


    print("Construction of listener completed")

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

def callback(data):
    #run every time i see a TFMessage on tf_topic.
    global tf_listener
    global last_seen
    global trans
    global rot
    
    tim = data.transforms[0]
    frame_id = tim.header.frame_id
    child_frame = tim.child_frame_id
    translations = tim.transform.translation
    tim = tim.header.stamp



    #print("callback run \n\r")

    #right now, hardcoded to look out for AR_Marker_63.  This may not be the world's greatest Idea...
    if  tf_listener.frameExists('ar_marker_8') & (child_frame=='ar_marker_8'):
        now = rospy.Time.now()

        #tf_listener.waitForTransform('ar_marker_63','left_gripper',now,rospy.Dure)
        try:
            ndata = data
            ndata.transforms[0].transform.translation.x = -translations.x
            ndata.transforms[0].transform.translation.y = -translations.y
            ndata.transforms[0].child_frame_id = "YourMother"
            headpub.publish(ndata)
            #bring this down to 5hz, the camera rate.  I don't wanna kill the processor on this...
            #time.sleep(0.2)
        #time0 asks for the most recent one
        except:
            #print("error with the update \n\r")
            pass



def get_pose(limb):
    pose = {}

    while len(pose) is 0:
        #sometimes, i'm getting empty poses.  don't know why, but this should take care of it.
        pose = limb.endpoint_pose()
        #print(pose)
        time.sleep(.01)

    pose = {'rot': list(pose['orientation']), 
            'trans' : list(pose['position'])}

    return pose

        
def talker(tf_topic):
  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  print("Initializing node... ")
  rospy.init_node("headpub")
  print("topic is " + str(tf_topic))


  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type std_msgs/String to the topic /chatter_talk
  pub = rospy.Publisher(tf_topic,TFMessage)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  print("Construction opf talker completed")
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
        if shift:
    
            # Publish our data to the 'chatter_talk' topic
            pub.publish(ndata)
            shift=False
    
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
        
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell


# Initializes listener for the TF and then closed_loop_pick
def init():
    listener('/tf')


if __name__ == '__main__':
  try:
    talker('/tf')
    init()
  except rospy.ROSInterruptException: pass