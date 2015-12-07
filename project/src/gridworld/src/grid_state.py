#!/usr/bin/env python
import tf
import rospy
import sys

from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import Transform, Vector3
import os


from pieces import all_tags
import time
from board_geometry import *
from gridworld.srv import *
from headcam.srv import *



#assume grid_origin (EG, ar_tag0, is located at 0cm x/y)
grid_origin = [0,0]

grid_size = [8,8] #number of squares.  Remember, start from 0.  (This assumes that the origin is on the grid, in the bottom left hand corner, pacman style)
grid_identifiers = [['A','B','C','D','E','F','G','H'],[1,2,3,4,5,6,7,8]]

grid_meas = [27.3,21.5]
square_size = 0.027 #square size in m.

state = None #initial board state.

timeout = 1 #time in seconds before grid_state decides that the AR tag is gone

def list_all_visible_ARtags(listener):
    lis = listener.allFramesAsString();



    #great.  So all those frams are EVER seen.  I need to filter out so I only look for tags seen... the last 5 secnods?

    now = rospy.Time.now()


    if lis.find('ar') is not -1:
        #ok, that means I have at least one ar tag somewhere.
        #split into individual sentences
        lis = lis.split('\n')
        lis = lis[:-1] #strip out last, empty, string, which is an artifact of splitting on\n when \n is last character

        for i in range(0,len(lis)):
            sent = lis[i]
            sent_split = sent.split(' ')
            
            frame = sent_split[1]

            if frame in list_of_board_markers:
                #that means that this is an AR tag that's from my board, and not some other ar tag
                #print(dir(tf))
                last_seen = listener.getLatestCommonTime(frame,'usb_cam')
                #currently, hardcoded for the webcam.  Seems reasonable, but I may need to pass in the right arguments to it...

                #print("Last seen at " + str(last_seen) + " now is " + str(now))

                t_diff = ( now.to_sec() - last_seen.to_sec())

                #print(t_diff)

                if abs(t_diff) < timeout:
                    lis[i] = frame

                #lis is now a list of strings, where each string is the ar_marker_N.  I've seen all of those within 2 seconds.
    return lis

#def get_all_kind_ARtags(kind):


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

def print_board_state(game_state):
    #print list of positions that have othello pieces, and their assosiated tuple
    #then, print_board


    for grid in game_state.keys():
        if game_state[grid][1] is not None:
            print(str(grid) + " " + str(game_state[grid])),

    print("")
    print_board(game_state)

    print("--------------------------------------------------------")



def print_board(game_state):
    #http://stackoverflow.com/questions/22104920/how-do-i-print-a-grid-from-a-list-of-lists-with-numbered-rows-and-columns
    #print(game_state)

    print("    "),
    for i in grid_identifiers[0]:
        # column headers
        print(i + '  '),
    print
    grid_identifiers[1].reverse()
    for i in grid_identifiers[1]:
        # row number
        print(str(i) + "  "),

        for j in grid_identifiers[0]:
            #print("J is " +str(j))
            #print("I is " + str(i))


            if (j,i) in game_state:
                #print("Found it in game state"),
                print(" " + game_state[(j,i)][0] + " "),
            else:
                print(" . "),
        print
    grid_identifiers[1].reverse()
    #reverse it back

def get_board_tags():

    board_tags = {}

    for tag in all_tags.keys():
        if all_tags[tag] == '.':
            # '.' is the symbol for a board tag,
            board_tags[tag] = all_tags[tag]

    return board_tags

def blank_board_state():
    #a blank board state.  THis is assuming that there are no othello pieces on there...
    board_state = {}
    for x in grid_identifiers[0]:
        for y in grid_identifiers[1]:
            board_state[x,y] = ('.',None,None)
    return board_state


def get_nearby_locs(pos,itr):
    #itr represents the number of steps that I have to go.  it goes 0,1,2,3, etc.  I'll handle the sizing into square_size
    x = grid_identifiers[0].index(pos[0])  #get x in numerical format.
    y = pos[1]

    #adjust for get_adjacent_cells' funky behaviour with square_size == None
    if itr == 0:
        nearby_locs = get_adjacent_cells(x,y,grid_size[0],grid_size[1],None)
    else:
        nearby_locs = get_adjacent_cells(x,y,grid_size[0],grid_size[1],2*itr - 1)

    nearby_locs = list(nearby_locs)

    for i in range(len(nearby_locs)):
        loc = nearby_locs[i]
        nearby_locs[i] = (grid_identifiers[0][loc[0]],loc[1])
        #convert loc[0] back to A thru H
        #do this loc thing so that we can keep everything in tuples.

    return nearby_locs



def get_adjacent_cells(x_coord, y_coord,width,height,square_size = None):
    #import from CS 188 Project 5.

    #http://stackoverflow.com/questions/2373306/pythonic-and-efficient-way-of-finding-adjacent-cells-in-grid
    #plus some mods i did myself
    #square size had better be odd....

    #one small mod: changed >= to > on the x's so that our grid goes from 1 to 8
    if square_size == None:
        #I want the 4 nearest neighbors, nothing else...
        result = []
        #individually iterate along x and y axis, just one in each direction
        for x,y in [(x_coord,y_coord+j) for j in (-1,0,1) if j != 0]:
            if (x > 0) and (x < width) and (y > 0) and (y <= height):
                result.append((x,y))
        for x,y in [(x_coord+j,y_coord) for j in (-1,0,1) if j != 0]:
            if (x > 0) and (x < width) and (y > 0) and (y <= height):
                result.append((x,y))
        return result
    elif square_size == 1: #size 1 square, IE, 8 nearest neighbors
        a = 1
        val_range = range(-a,(a+1))
    else:
        a = square_size/2  #foored down b/c ints
        val_range = range(-a,(a+1))
    result = []
    #iterate in all ways...
    for x,y in [(x_coord+i,y_coord+j) for i in val_range for j in val_range if i != 0 or j != 0]:
        if (x > 0) and (x < width) and (y > 0) and (y < height):
            result.append((x,y))
    return result


def determine_board_state():
    visible_tags = list_all_visible_ARtags(listener)
    #print("Visible tags is " + str(visible_tags))

    non_visible_tags = []

    for board_tag in get_board_tags().keys():
        if board_tag not in visible_tags:
            non_visible_tags.append(board_tag)

    #print("I can't see the following board tags:")
    #print(non_visible_tags)
    
    board_state = blank_board_state()



    #print(board_state)

    #print("Non visible_tags is")
    #print(non_visible_tags)



    for tag in non_visible_tags:
        pos = marker_to_loc[tag]
        
        #print(tag)
        #print(pos)

        success = False
        count = 0
        #successfully found a neighbor that's free
        homing_name = None

        while not success:
            #need to find nearest place that is open
            #print("POS IS " + str(pos))
            #print("Count is " + str(count))
            nearby_locs = get_nearby_locs(pos,count)

            #print(nearby_locs)

            #print("")
            #print("")

            for loc in nearby_locs:
                if not (loc_to_marker[loc] in non_visible_tags):
                    homing_loc = loc
                    homing_name = loc_to_marker[homing_loc]
                    success = True
                    break



            count = count+1


        #mark an othello piece there
        board_state[pos] = ('X','piece_' + str(pos[0]) + '_' + str(pos[1]), homing_name )

        #ok, if there's a piece there, it needs both target_name, and a homing_name




    #print board_state

    return board_state


def update_tf_frames(bstate):
    #iterate through all the grid points.  If it's found that there's an othello piece there...
    for grid in bstate.keys():
        if bstate[grid][1] is not None:

            child_frame_id = bstate[grid][1]
            homing_frame_id = bstate[grid][2]

            transform = geometry_msgs.msg.Transform()

            x = grid_identifiers[0].index(grid[0])
            parent_x = grid_identifiers[0].index(marker_to_loc[homing_frame_id][0])
            dx = - parent_x + x
            dy = - marker_to_loc[homing_frame_id][1] + grid[1] 

            #print('Child is ' + str(child_frame_id))
            #print('Parent is' + str(homing_frame_id))
            #print("DX " + str(dx) + " DY " + str(dy))

            transform.translation.x = dx * square_size
            transform.translation.y = dy * square_size
            transform.translation.z = 0.2/2.54/100 #b/c an othello piece has height.  distance determined by eye.
            transform.rotation.x = 0
            transform.rotation.y = 0
            transform.rotation.z = 0
            transform.rotation.w = -1

            #think that q = [1,0,0,0] is a no rotation quaternion
            response = tf_updater(child=child_frame_id,parent= homing_frame_id,transform = transform)
            #print response
        else:
            #remove it from view, since I know that there's now no piece there.  remove it --> empty homing frame ID
            pos = grid

            #construct what the child_frame would be called, and then tell TransServ.py to kick it out.
            child_frame_id = 'piece_' + str(pos[0]) + '_' + str(pos[1])
            homing_frame_id = ''

            transform = geometry_msgs.msg.Transform()
            transform.translation.x = 0
            transform.translation.y = 0
            transform.translation.z = 0
            transform.rotation.x = 1
            transform.rotation.y = 0
            transform.rotation.z = 0
            transform.rotation.w = -1

            #have to populate something in the quaternion, so it's not all nans

            response = tf_updater(child=child_frame_id,parent= homing_frame_id,transform = transform)

            #print(transform)
            #print(str(grid) + " " + str(bstate[grid])),


def run_board_state():
    global state
    while not rospy.is_shutdown():
        #print("foo1")
        state = determine_board_state()
        #print("foo2")
        
        #print(state)

        print_board_state(state)

        #print("foo3")
        update_tf_frames(state)

        #print("foo4")
        #ok, now, i need to go through my board state for 

        time.sleep(0.50)

        print("")

def service_handle(data):
    #I don't care about data for now.

    return str(state)
    #return the dictionary board state as a string

def init():
    global listener
    global rate
    global tf_updater

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    s = rospy.Service('board_state', tag_persistance, service_handle)

    rospy.wait_for_service('new_tf_frames')
    tf_updater = rospy.ServiceProxy('new_tf_frames',tag_persistance)

    print("Ready to give out board state")

    #have to give it a bit of time to see the AR tags.
    time.sleep(3)
    run_board_state()
    #todo: Setup service!.


if __name__=='__main__':
    print("HELLO WORLD!")
    rospy.init_node('ar_tags_subs')
    if len(sys.argv) > 1:
        print('Use: grid_world.py')
        sys.exit()
    else:
        init()

        #remove later.




        


