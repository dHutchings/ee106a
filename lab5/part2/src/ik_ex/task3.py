#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import argparse

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

step = 0

step0 = [0.704,-.134,-.130,0.933,0.355,0.000,0.052]
step1 = [0.704,-.134,-.150,0.933,0.355,0.000,0.052]
step2 = [0.704,-.134,-.1,0.933,0.355,0.000,0.052]
step3 = [.807,.072,-.130,0.933,0.355,0.000,0.052]
step4 = [.5,.5,.3,0,1,0,0]

step5 = [.807,.072,-.130,0.933,0.355,0.000,0.052]
step6 = [.807,.072,-.150,0.933,0.355,0.000,0.052]
step7 = [.807,.072,-.100,0.933,0.355,0.000,0.052]

step8 = step0


global CHECK_VERSION



positions = [step0,step1,step2,step3,step4,step5,step6,step7,step8]


def move_arm():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    global step

    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    
    while not rospy.is_shutdown():
        print("I am on step " + str(step))
        raw_input('Press [ Enter ]: ')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_hand"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        cmd = positions[step]
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = cmd[0]
        request.ik_request.pose_stamped.pose.position.y = cmd[1]
        request.ik_request.pose_stamped.pose.position.z = cmd[2]     
        request.ik_request.pose_stamped.pose.orientation.x = cmd[3]
        request.ik_request.pose_stamped.pose.orientation.y = cmd[4]
        request.ik_request.pose_stamped.pose.orientation.z = cmd[5]
        request.ik_request.pose_stamped.pose.orientation.w = cmd[6]
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("left_arm")


            #this command tries to achieve a pose: which is position, orientation
            group.set_pose_target(request.ik_request.pose_stamped)

            #then, this command tries to achieve a position only.  orientation is airbitrary.
            group.go()


            if (step == 1) or (step == 6):
                print("closing")
                grip_left.close()
            elif(step == 3) or (step == 8):
                print("OPENING")
                grip_left.open()
            
        




        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        step = (step+1) % len(positions)




if __name__ == '__main__':
    move_arm()
