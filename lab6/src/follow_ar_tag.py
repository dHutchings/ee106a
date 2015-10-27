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
import atexit
import time
import math
import traceback
import random

listener = None

last_seen_zumy = 0

def exit_handler():

    #tell zumy to stop on exit!!
    for i in range(0,10):
        out = Twist()
        pub.publish(out)
        rospy.Rate(10).sleep()
        print("STOP MEOW")
    print 'Zumy should stop'

atexit.register(exit_handler)


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
        print("loop " + str(random.random()))
        #ok, need to compute the transform between tag[0] (stationary) and tag[1] (zumy)
        try_success = False
        try:
            now = rospy.Time.now()
            
            listener.waitForTransform(ar_tags['ar1'],ar_tags['arZ'],now,rospy.Duration(1))
            (trans, rot) = listener.lookupTransform(ar_tags['ar1'], ar_tags['arZ'],now)# rospy.Time(0))
            
            rbt = ats.return_rbt(np.array(trans),np.array(rot))


            #print('gab between ' + ar_tags['ar1'] + ' and ' + ar_tags['arZ'])
            #print rbt
            
            twist = ats.compute_twist(rbt=rbt)
            #print('twist between ' + ar_tags['ar1'] + ' and ' + ar_tags['arZ'])
            #print twist

            #now, need to create a geomeryy Msg Twist.



            theta = math.degrees(math.atan2(rbt[1][3],rbt[0][3]))
            print(theta)
            #extract theta out of RBT, because we want zumy to move directly towards the target, not move to park ontop of (and with the orientation of) the target
            
            last_seen_zumy = time.time()
            try_success = True      
        except:
            traceback.print_exc()
            #print("in Except")
            print("AR TAGS EXISTANCE IS AS FOLLOWS: ")
            print(listener.frameExists(ar_tags['ar1']))
            print(listener.frameExists(ar_tags['arZ']))
            print ''

        global last_seen_zumy
        out = Twist()

        if (time.time() - last_seen_zumy > 2):
            #I haven't seen zumy for 2 seconds.  Issue E-stop
            #do publish here so i can be more selective on when to NOT publish stops.
            pub.publish(out)
            print("Stop!")

        elif try_success:

            out = Twist()
            
            #if abs(theta) < 5:
            #go forward!
            #print("FORWARD!")

            #ok, two parts: going forward, and turning.

            print(trans)

            dist = trans[0]
            print("dist is " + str(dist))
            print("Theta is " + str(theta))
            
            if abs(dist)>1:
                linear_gain = .13
            elif abs(dist) > .5:
                linear_gain = .12
            else:
                linear_gain = .10

            if dist > .12:
                out.linear.x =  linear_gain# * twist[0][0]
            else:
                
                #mission accomplished... i'm really close
                pass

            #else:
            #just rotate towards objective.
            #let's come up with a new RBT that achieve those goals... just rotation, no translation.
            omega = np.array([0,0,1]) #rotate about z axis?
            theta_rad = math.radians(theta)
            trans = np.array([0,0,0]) #no translation!
            rbt = eqf.create_rbt(omega,theta_rad,trans)
            twist = ats.compute_twist(rbt=rbt)

            #ok, so now I have a rotational axis.  I now need to scale it by something... ok, what about linear function?

            if abs(theta) > 20:
                rotational_gain = .18
            else:
                rotational_gain = .18*abs(theta/20)

            out.angular.z = rotational_gain*twist[1][2]
            

            print(out)

            pub.publish(out)
        #S IS STUFF TO PUBLISH
        #pub.publish(s,rospy.get_time())

        # Use our rate object to sleep until it is time to publish again
        r.sleep()
            
    exit_handler()









  
if __name__=='__main__':
    rospy.init_node('follow_ar_tag')


    ar_tags = {}
    zumy_name = sys.argv[1]

        
    pub = rospy.Publisher("/zumy" + zumy_name +"/cmd_vel", Twist, queue_size=10)
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    rospy.spin()



