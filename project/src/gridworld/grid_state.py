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
import os


from pieces import all_pieces



#assume grid_origin (EG, ar_tag0, is located at 0cm x/y)
grid_origin = [0,0]

grid_size = [11,7] #number of squares.  Remember, start from 0.  (This assumes that the origin is on the grid, in the bottom left hand corner, pacman style)
grid_meas = [27.3,21.5]
square_size = 0.027 #square size in m.



def list_all_ARtags(listener):
    lis = listener.allFramesAsString();

    if lis.find('ar') is not -1:
        #ok, that means I have at least one ar tag somewhere.
        #split into individual sentences
        lis = lis.split('\n')
        lis = lis[:-1] #strip out last, empty, string, which is an artifact of splitting on\n when \n is last character

        for i in range(0,len(lis)):
            sent = lis[i]
            sent_split = sent.split(' ')
            lis[i] = sent_split[1]

    #lis is now a list of strings, where each string is the ar_marker_N.
    return lis

def locate_on_grid(x,y):
    #return integer location of grid square, given the x y position, and the constants above in line 15-18.
    col = int(round(x / float( square_size)))#round(x / square_size)
    row = int(round(y / float( square_size)))#round(y / square_size)

    return (col,row)

def project_onto_plane(point):
    #point = (x,y,z)

    #http://stackoverflow.com/questions/8942950/how-do-i-find-the-orthogonal-projection-of-a-point-onto-a-plane

    q = np.array([point[0],point[1],point[2]])
    p = np.array([0,0,0]) #assume that the tag origin is at 0,0,0, with normal straight up, eg, [0,0,1]
    n = np.array([0,0,1])

    q_proj = q - np.dot(q - p, n) * n

    return q_proj

def print_board(game_state):
    #http://stackoverflow.com/questions/22104920/how-do-i-print-a-grid-from-a-list-of-lists-with-numbered-rows-and-columns
    print(game_state)
    for i in range(-1,grid_size[0]):
        if i == -1:
            # column for row numbers
            print("   "),
        else:
            # column headers
            print("{0:2d} ".format(i)),
    print
    for i in range(grid_size[1],-1,-1):
        # row number
        print("{0:2d} ".format(i)),

        for j in range(grid_size[0]):
            #print (j,i)
            if (j,i) in game_state:
                print(" " + game_state[(j,i)] + " "),
            else:
                print(" . "),
        print

if __name__=='__main__':
    rospy.init_node('ar_tags_subs')
    if len(sys.argv) < 2:
        print('Use: ar_tag_subs.py [Origin number]')
        sys.exit()
    origin = 'ar_marker_' + sys.argv[1]

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        tags = list_all_ARtags(listener)



        if origin in tags:

            tags.remove(origin)        
            print(repr(tags))
            print(tags)
            tags.sort()

            game_state = {}

            for tag in tags:
                if tag in all_pieces:
                    #filter out AR_alvar thinking that it sees at tag #70 or something like that
                    (trans, rot) = listener.lookupTransform(origin, tag, rospy.Time(0))
                    
                    xyz = project_onto_plane(trans)  #project it onto plane of origin.

                    xy = xyz[0:2] #strip out last element, to collapse it to xy.

                    #print "XY between origin and " + tag + " is %1.3f %1.3f" %(xy[0],xy[1])

                    xy = locate_on_grid(xy[0],xy[1])
                    print(tag + " XY Grid" + str(xy))

                    game_state[xy] = all_pieces[tag]

            print_board(game_state)
        else:
            print("Cannot see origin " + origin)


        #print(listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'],rospy.Time(0)))

        rate.sleep()
