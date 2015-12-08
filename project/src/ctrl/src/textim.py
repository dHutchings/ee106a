import Image
import ImageDraw
import ImageFont
import cStringIO
import os
import sys
import argparse
 
import rospy
 
import cv2
import cv_bridge
 
# img = cv2.imread(path)
# bridge = cv_bridge.CvBridge()
# msg = bridge.cv2_to_imgmsg(img,encoding="bgr8")
# pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
# pub.publish(msg)
# # Sleep to allow for image to be published.
# rospy.sleep(1)


if __name__ == '__main__':

    img = Image.new('RGB', (200, 100))
    d = ImageDraw.Draw(img)
    d.text((20, 20), 'Hello', fill=(255, 0, 0))
    text_width, text_height = d.textsize('Hello')

    s = cStringIO.StringIO()
    img.save("/home/arak/Desktop/img.png")
    in_memory_file = s.getvalue()

    raw_img_data = img.tostring()

    #!/usr/bin/env python

# usage_str = \

# def listen_bstate():
#     print("Initializing node... ")
#     rospy.init_node('headimg')
#     s = rospy.Subsriber('pente_ctrl/boardstate', String, sendimg)
#     print "Ready to Adjust Frames"

# def sendimg(strdata):
#     img = Image.new('RGB', (200, 100))
#     d = ImageDraw.Draw(strdata)
#     d.text((20, 20), , fill=(255, 0, 0))
#     text_width, text_height = d.textsize(strdata)

#     s = cStringIO.StringIO()
#     img.save("image.png")
#     in_memory_file = s.getvalue()
#     raw_img_data = img.tostring()

#     return("success")

# def init():
#     movement_server()
#     #then, down here, do a while_true
#     r = rospy.Rate(10) # 10hz
#     bridge = cv_bridge.CvBridge()

#     while not rospy.is_shutdown():
#         now = rospy.Time.now()
#         path = "image.png"

#         img = cv2.imread(path)
#         msg = bridge.cv2_to_imgmsg(img,encoding="bgr8")
#         pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
#         pub.publish(msg)
#         # Sleep to allow for image to be published.
#         rospy.sleep(1)

# if __name__ == '__main__':
#     if(len(sys.argv) > 1):
#         print usage_str
#     else:
#         init()