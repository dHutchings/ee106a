#!/usr/bin/env python

#get this to be run as python so the imports work..

import rospy
import argparse
import sys


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import time

latch = False


#the next three variables are tuning parameters.
overlap = 100 #overlap the non-censored areas by N pixels.  This is so an AR tag on the border doesn't get cut.  IN pratice, set to the largest size an AR tag will be in the FOV
x_splits = 3 #of splits of the image in x,y
y_splits = 2


def censor_image(image,xmin,xmax,ymin,ymax,publisher,frame_id):
    #censors the image to just the area between x/y min/max.
    #also publishes the thing, using the publisher that is given to it.  (There's many... muhahaha)
    
    #make a new, blank image.  Keep the data type as unsigned 8 bit ints, so we can get this back to ROS
    new_image = np.zeros(image.shape,dtype=np.uint8)

    #strip height,width data
    height = image.shape[0]
    width = image.shape[1]

    #use slice notation to efficiently directly set what I want.  Everything else is zero
    new_image[ymin:ymax,xmin:xmax,:] = image[ymin:ymax,xmin:xmax,:]

    #convert back to ros image
    image_out = bridge.cv2_to_imgmsg(new_image, encoding="rgb8")

    print(dir(image_out))
    image_out.header.frame_id = frame_id

    #and publish it.
    publisher.publish(image_out)


def callback(data):

    #print("I've censored another image!")
    old_time = time.time()

    frame_id = data.header.frame_id #need this frame ID so TF can work properly

    #convert ros image to open cv image.
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    #convert open cv image into numpy array.
    cv_image = np.asarray(cv_image)

    #strip height,width data
    height = cv_image.shape[0]
    width = cv_image.shape[1]

    #lots of logic to set up the offset windowing & split.
    #note: does not cleanly handle the case where the offset is on the order of the size of the window
    ypts = np.linspace(0,height,num=(y_splits+1))
    xpts = np.linspace(0,width,num=(x_splits+1))
    for y in range(0,y_splits):
        for x in range(0,x_splits):

            #print("Y_top")    
            if y == 0:
                y_top = 0
            else:
                y_top = ypts[y] - overlap
            #print(y_top)

            #print("Y_bot")
            if y == y_splits-1:
                y_bot = height
            else:
                y_bot = ypts[y+1] + overlap
            #print(y_bot)

            #print("X_left")    
            if x == 0:
                x_left = 0
            else:
                x_left = xpts[x] - overlap
            #print(x_left)

            #print("X_right")
            if x == x_splits-1:
                x_right = width
            else:
                x_right = xpts[x+1] + overlap
            #print(x_right)

            #print(str(x) + " " + str(y))
            #print( y*x_splits + x)

            #publisher number.  Indexed to zero.  Goes from #0 at 0,0, to last one at the end.  Goes along x, then along y
            pub_number = y*x_splits + x

            #get the right publisher
            publisher = pubs[pub_number]

            censor_image(cv_image,x_left,x_right,y_top,y_bot,publisher,frame_id)
    
    print('Ive sensored another image! ' + str(time.time() - old_time))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global pubs

    rospy.init_node('image_censor', anonymous=True)

    #initliaze the array of publishers... this is gonna get insane.
    pubs = []

    #make lots of publishers.  It's a global, so the callback can get to it.
    for i in range(0,x_splits*y_splits):
        pubs.append(rospy.Publisher('/censored/image_raw'+str(i), Image, queue_size=10))

    #and here's my subscriber, which will listen on the line below.  This should really be made a command line argument.
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)


    # spin() simply keeps python from exiting until this node is stopped.  Callbacks will automagically continue
    rospy.spin()


if __name__=='__main__':
    global bridge
    print sys.argv
    bridge = CvBridge()

    listener()
