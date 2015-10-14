#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf

#new stuff
import ar_tag_subs as ats

listener = None




def follow_ar_tag(zumy, ar_tags):
    """
    This function should obtain the rigid body transform between the Zumy and the AR ar_tag
    Then compute and send the twist required to drive the Zumy to the AR ar_tag
    """
    print(zumy)
    print(ar_tags)
    # YOUR CODE HERE

    #copy paste from lab2, change the proper nouns.  Pumbing to make a publisher

    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    r = rospy.Rate(10) # 10hz


    #plumbing to get transforms:
    listener = tf.TransformListener()

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        #ok, need to compute the transform between tag[0] (stationary) and tag[1] (zumy)
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar1'], ar_tags['arZ'], rospy.Time(0))

            rbt = ats.return_rbt(trans=trans, rot=rot)
            #rbt = return_rbt(np.array(trans),np.array(rot))
            print('gab between ' + ar_tags['ar1'] + ' and ' + ar_tags['arZ'])
            print rbt
            twist = ats.compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar1'] + ' and ' + ar_tags['arZ'])
            print twist
        except:
            print("AR TAGS EXISTANCE IS AS FOLLOWS: ")
            #print(listener.frameExists(ar_tags['ar0']))
            print(listener.frameExists(ar_tags['ar1']))
            print(listener.frameExists(ar_tags['arZ']))
            print ''


        #S IS STUFF TO PUBLISH
        #pub.publish(s,rospy.get_time())

        # Use our rate object to sleep until it is time to publish again
        r.sleep()
  
if __name__=='__main__':
    rospy.init_node('follow_ar_tag')
    pub = rospy.Publisher('/zumy1c/cmd_vel', Twist, queue_size=10)

    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    rospy.spin()
