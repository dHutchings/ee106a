#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import exp_quat_func as eqf

def callback(msg, ar_tags):
    tags = {}
    for i in range(0, len(msg.transforms)):

        # YOUR CODE HERE
        # The code should look at the transforms for each AR tag
        # Then compute the rigid body transform between AR0 and AR1, 
        # AR0 and ARZ, AR1 and ARZ
        #  hint: use the functions you wrote in exp_quat_func

        #extract tag ID
        tag = msg.transforms[i].child_frame_id

        #extract rotation quaternion as an nd array
        rotation_msg = msg.transforms[i].transform.rotation        
        rotation_quaternion = np.array([rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w])

        #compute omega, theta, of the quaternion.
        omega, theta = eqf.quaternion_to_exp(rotation_quaternion)

        #extract translation as nd array
        translation_msg = msg.transforms[i].transform.translation
        translation = np.array([translation_msg.x, translation_msg.y, translation_msg.z])

        #compute transform from camera to AR tag.
        tag_transform = eqf.create_rbt(omega,theta,translation)
        

        print("Read tag " + str(tag))
        print(dir(msg.transforms[i]))
        print msg.transforms[i]
        print('foo1')
        print(msg.transforms[i].child_frame_id)
        print('foo2')
  
if __name__=='__main__':
    rospy.init_node('ar_tags_subs_manual')
    if len(sys.argv) < 4:
        print('Use: ar_tags_subs_manual.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_tags = {}
    ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    rospy.Subscriber('/tf', TFMessage, callback, ar_tags)
    rospy.spin()
