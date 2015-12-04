#!/usr/bin/env python
import sys
import rospy
# import roslib; roslib.load_manifest('kinematics')
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import numpy as np
import exp_quat_func as eqf
import kin_func_skeleton as kfs
from tf2_msgs.msg import TFMessage
import tf
import curses
import time

import traceback
rbt = None

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

def move_to_coord(trans, rot, arm, which_arm='left', keep_oreint=False,base="base"):
    #coordinates are in baxter's torso!
    goal = PoseStamped()
    goal.header.frame_id = base

    # x, y, and z position
    goal.pose.position.x = trans[0]
    goal.pose.position.y = trans[1]
    goal.pose.position.z = trans[2]
    
    # Orientation as a quaternion
    goal.pose.orientation.x = rot[0]
    goal.pose.orientation.y = rot[1]
    goal.pose.orientation.z = rot[2]
    goal.pose.orientation.w = rot[3]

    # Set the goal state to the pose you just defined
    arm.set_pose_target(goal)

    # Set the start state for the arm
    arm.set_start_state_to_current_state()

    if keep_oreint:
        # Create a path constraint for the arm
        orien_const = OrientationConstraint()
        orien_const.link_name = which_arm+"_gripper";
        orien_const.header.frame_id = "base";

        #constrain it to be the same as my goal state.  Seems reasonable.

        orien_const.orientation.x = rot[0];
        orien_const.orientation.y = rot[1];
        orien_const.orientation.z = rot[2];
        orien_const.orientation.w = rot[3];
        orien_const.absolute_x_axis_tolerance = 0.1;
        orien_const.absolute_y_axis_tolerance = 0.1;
        orien_const.absolute_z_axis_tolerance = 0.1;
        orien_const.weight = 1.0;
        consts = Constraints()
        consts.orientation_constraints = [orien_const]
        print(consts)
        arm.set_path_constraints(consts)

    # Plan a path
    arm_plan = arm.plan()

    # Execute the plan
    arm.execute(arm_plan)

def incrimental_movement(dx,dy,dz,arm,which_arm,rbt=None,last_pos = None,changeHeight=True,keep_oreint=False):
    #move the arm an incrimental distance dx, dy, dz.\
    #relative to a frame which has the RBT to the BASE FRAME
    if rbt == None:
        new_rot = np.eye(3)
    else:
        new_rot = np.array([[rbt[0][0], rbt[0][1], rbt[0][2]], 
            [rbt[1][0], rbt[1][1], rbt[1][2]], [rbt[2][0], rbt[2][1], rbt[2][2]]])
        new_rot = np.linalg.inv(new_rot)

    inc_dist = np.array([[dx],[dy],[dz]])
    inc_trans = np.dot(new_rot, inc_dist)

    limb = baxter_interface.Limb(which_arm)
    
    pose = get_pose(limb)


    if last_pos is None:
        #ok, so i don't know where my code thinks I should be.  Let me just fetch where I am

        trans = pose['trans']
        last_pos = np.array([[trans[0]], [trans[1]], [trans[2]] ])
    else:
        pass
        #print("I have a last pos \n\r")
        #print(last_pos)
    #if i have a last pos fed to me by someone, use that instead.  That'll allow some wrapper software to keep track of where i was supposed to be at
    #this will be useful in case of small intended movements given consecutively.

    #print("I think that i'm at \n\r")
    #print(last_pos)

    #print("And I want to move by, in the base frame \n\r")
    #print(inc_trans)
    dest = last_pos + inc_trans
    #pack np array into normal list

    if changeHeight:
        dest_ar = [dest[0][0], dest[1][0] ,dest[2][0]]
    else:
        #get my last height, and shove that into the dest array
        dest_ar = [dest[0][0], dest[1][0] ,last_pos[2][0]]

    #print("My destination is \n\r")
    #print(str(dest_ar) + "\n\r")
    print(pose['rot'])
    move_to_coord(dest_ar, pose['rot'], arm, which_arm,keep_oreint)
    #but return it as a col array
    return dest


    

def text_ctrl():
    return

# This command dictates which key decides which Baxter movement.
def keyboard_ctrl(which_arm, arm, gripper, new_rot=None):

    #HACK HACK HACk
    global rbt

    gripper_closed = False
    limb = baxter_interface.Limb(which_arm)

    pose = get_pose(limb)

    screen = curses.initscr()

    if new_rot == None:
        new_rot = np.eye(3)
    # Here we may need to use inverse instead of the array we have. 
    # I am not sure if we will be receiving the rbt from the tag to the camera or vice versa.
    # Right now this assumes the rbt is from the tag to the torso. 
    new_rot = np.array([[new_rot[0][0], new_rot[0][1], new_rot[0][2]], 
        [new_rot[1][0], new_rot[1][1], new_rot[1][2]], [new_rot[2][0], new_rot[2][1], new_rot[2][2]]])
    trans = pose['trans']
    trans_mat = np.array([[trans[0]], [trans[1]], [trans[2]]])

    inc_var = 0.050


    try:

        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        screen.keypad(1)
        screen.nodelay(1)

        while True:

            #moved inside while so dynamic updates to inc_var can propigate

            if rbt == None:
                new_rot = np.eye(3)
            else:
                #print("NEW RBT\n\r")
                new_rot = rbt
                #print(new_rot)
                #print("\n\r")

            new_rot = np.array([[new_rot[0][0], new_rot[0][1], new_rot[0][2]], 
                [new_rot[1][0], new_rot[1][1], new_rot[1][2]], [new_rot[2][0], new_rot[2][1], new_rot[2][2]]])

            new_rot = np.linalg.inv(new_rot)
            #need to invert hte rotation matrix.  While I have the RBT from the tag to the base, I need the rotation matrix that will rotate 
            #cardinal directions the other way 'round'


            event = screen.getch()

            
            if event == curses.KEY_RIGHT:
                trans_mat  = incrimental_movement(inc_var,0,0,arm,which_arm,rbt,trans_mat)
            elif event == curses.KEY_LEFT:
                trans_mat  = incrimental_movement(-inc_var,0,0,arm,which_arm,rbt,trans_mat)
            elif event == curses.KEY_UP:
                trans_mat  = incrimental_movement(0,inc_var,0,arm,which_arm,rbt,trans_mat)
            elif event == curses.KEY_DOWN:
                trans_mat  = incrimental_movement(0,-inc_var,0,arm,which_arm,rbt,trans_mat)
            elif event == 122:  # Z
                trans_mat  = incrimental_movement(0,0,inc_var,arm,which_arm,rbt,trans_mat)
            elif event == 120:  # X
                trans_mat  = incrimental_movement(0,0,-inc_var,arm,which_arm,rbt,trans_mat)
            elif event == 103:  # G
                if gripper_closed:
                    gripper.open(block=False)
                    gripper_closed = False
                else:
                    gripper.close(block=False)
                    gripper_closed = True
            elif event == 100: #D
                screen.nodelay(0) #temporarily turn off non-blocking getch
                #set inc_var to a new value
                print("Enter new value for inc-var, in the form\n\r")
                print("XX.XXd.  You must terminate with a d since we are bad at using curses\n\r")
                event2 = screen.getch()
                string = chr(event2)

                while event2 is not 'd':
                    event2 = chr(screen.getch())
                    string = string + event2
                string = string[:-1]
                inc_var = float(string)

                print("New Inc_var is " + str(inc_var) + '\n\r')
                screen.nodelay(1) #and then turn back on non-blocking getch
            elif event == 101: #E
            

                #resets position and orientation values. 
                pose = limb.endpoint_pose()
                pose = {'rot': list(pose['orientation']), 
                        'trans' : list(pose['position'])}
                trans = pose['trans']
                trans_mat = np.array([[trans[0]], [trans[1]], [trans[2]]])
                print("Orientation and position reset \n\r")
            elif event == 102: #F
                pose = limb.endpoint_pose()
                pose = {'rot': list(pose['orientation']), 
                        'trans' : list(pose['position'])}
                trans = pose['trans']
                print( str(trans) + "\n\r")
            elif event == 113: #q:
                exit_soft = True
                break


            

            elif event == 27:   # Esc
                exit_soft = False
                break
            elif event == -1:  #no press on getch, it'll just keep running
                pass
            else:
                print("invalid key: %r \n\r" % event)
            #so we can only spam keyboard commands so fast.  But in general, don't hold down the key.
            time.sleep(0.2)
            
    finally:
        screen.keypad(0)
        curses.curs_set(1)
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        if not exit_soft:
            rospy.signal_shutdown("Shutdown signal recieved")

def listener(tf_topic):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback
    rospy.Subscriber(tf_topic,TFMessage,callback)

    global tf_listener

    tf_listener = tf.TransformListener()

def callback(data):
    #run every time i see a TFMessage on tf_topic.
    global tf_listener
    global last_seen
    global rbt

    #header = data.header
    #print(data.transforms[0])
    #print(dir(data.transforms[0]))

    tim = data.transforms[0]
    tim = tim.header.stamp
    #print(tim)
    #print('fo')
    if tf_listener.frameExists('ar_marker_63'):

        now = rospy.Time.now()
        #tf_listener.waitForTransform('ar_marker_63','left_gripper',now,rospy.Dure)
        try:
            (trans,rot) = tf_listener.lookupTransform('ar_marker_63','base',tim)
            
            (omega,theta) = eqf.quaternion_to_exp(rot)
            rbt = eqf.return_rbt(trans,rot)

            #print(rbt)
            #print("")
            #print("")
            #print("Period was " + str(time.time() - last_seen))
            if last_seen - time.time() > 10:
                #print("updated rbt, i hadn't seen it in a while \n\r")
                #i haven't seen the tag in a while
                pass
            last_seen = time.time()

            #bring this down to 5hz, the camera rate.  I don't wanna kill the processor on this...
            time.sleep(0.2)
        #time0 asks for the most recent one
        except:
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

def closed_loop_pick(which_arm,arm,destination='othello_piece',):
    #assume that i'm already close.
    global tf_listener
    global last_seen
    limb = baxter_interface.Limb(which_arm)

    while True:
        pose = get_pose(limb)

        execute = True
        exit = raw_input("E to quit. K to start keyboard.  D for dist to target. Otherwise, i'll move to 0 \n\r")
        if (exit == 'e') or (exit == 'E'):
            rospy.signal_shutdown("Shutdown signal recieved")
            return
        elif(exit == 'd') or (exit == 'D'):
            execute = False
        elif(exit == 'k') or (exit == 'K'):
            keyboard_ctrl(which_arm,arm,left_gripper)

        try:
            #now = rospy.Time.now()
            #tf_listener.waitForTransform('suction_cup','othello_piece',now,rospy.Duration(2.5))
            #now = rospy.Time.now()
            last_time = tf_listener.getLatestCommonTime('suction_cup','othello_piece')
            (trans,rot) = tf_listener.lookupTransform('suction_cup','othello_piece',last_time)

            print("Saw the othello piece " + str(last_time.to_sec() - rospy.Time.now().to_sec()) + " secs ago \n\r")

            #get the RBT from the suction_cup to the base so incrimental movement can deal with it.
            last_time = tf_listener.getLatestCommonTime('suction_cup','base')
            (trans_base,rot_base) = tf_listener.lookupTransform('suction_cup','base',last_time)
            rbt_to_base = eqf.return_rbt(trans_base,rot_base)
            
            #print("time diff")


            #if time.time() - last_seen > 3:
            #    print("Warning.  lag is over 3 seconds")
            #last_seen = time.time()
        except KeyboardInterrupt:
            sys.exit()

        except:
            print("Could not find trans / rot \n\r")
            traceback.print_exc()
        else:
            print("Error between suction cup and the othello piece is \n\r")
            print(trans)
            print("")


            err_dist = np.linalg.norm(trans[:2]) #error in meters from 

            print("XY ERROR IS " +str(err_dist))

            if execute:

                if trans[2] > 0.05:
                    #i'm very high away, just get closer...
                    trans_goal = (.85*trans[0],.85*trans[1],trans[2]/2)
                    change_height = True
                    keep_oreint = False
                elif err_dist > 0.01: #1cm, 1 inch.
                    #final lateral alignment
                    trans_goal = (trans[0],trans[1],0) #i want to move the suction cupt to 0,0,whatever height i'm at right now of hte othello piece.
                    change_height = False
                    keep_oreint = True
                else:
                    #just go down % hit it
                    trans_goal = (.5*trans[0],.5*trans[1],trans[2])
                    change_height = True
                    keep_oreint = True

                print("I want to move by, in the suction cup frame, \n\r")
                print(str(trans_goal) + "\n\r")

                incrimental_movement(trans_goal[0],trans_goal[1],trans_goal[2], arm, which_arm,rbt = rbt_to_base,changeHeight = change_height,keep_oreint=keep_oreint)
                #wait for movement to execute ... wonder if there's a better way to do this?
                #time.sleep(2.5)
        
# Initializes all limbs/parts of limbs for later usage in the code. 
def init(mode='ROS'):
    print("Initializing node... ")
    rospy.init_node("ik_control")

    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled

    # Initialize moveit commander
    moveit_commander.roscpp_initialize([sys.argv[0]])

    # Initialize arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(5)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(5)

    # Initialize grippers
    right_gripper = baxter_interface.gripper.Gripper('right')

    global left_gripper
    left_gripper = baxter_interface.gripper.Gripper('left')

    left_arm_str = 'left_arm'
    right_arm_str = 'right_arm'

    print('Calibrating...')
    right_gripper.calibrate()
    left_gripper.calibrate()
    rospy.sleep(2.0)

    # Shutdown callback
    def clean_shutdown():
        if not init_state:
            print("Disabling robot...")
            rs.disable()

    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    # Start keyboard control
    if mode == 'keyboard':

        keyboard_ctrl('left', left_arm, left_gripper, rbt)        
    elif mode == 'text':
        text_ctrl('left', left_arm, left_gripper)
    elif mode == 'ROS':
        # Set up listener node
        listener('/tf')
        rospy.spin()
    elif mode =='ROS&keyboard':
        print("DOING BOTH AT SAME TIME")
        listener('/tf')
        keyboard_ctrl('left', left_arm, left_gripper, rbt)
    elif mode == 'CloseLoop':
        #print(moveit_commander)
        #print(dir(moveit_commander))
        #print(moveit_commander.move_group)
        #print(dir(moveit_commander.move_group))
        #left_arm.set_end_effector_link('suction_cup')
        #left_arm.set_goal_position_tolerance(0.001)
        listener('/tf')
        closed_loop_pick('left',left_arm)

if __name__ == '__main__':
    if len(sys.argv) == 1:
        init()
    elif ('-l' in sys.argv) and ('-k' in sys.argv):
        init('ROS&keyboard') 
    elif sys.argv[1] == '-h':
        print(usage_str)
    elif sys.argv[1] == '--help':
        print(usage_str)
    elif sys.argv[1] == '-k':
        init('keyboard')
    elif sys.argv[1] == '-t':
        init('text')
    elif sys.argv[1] == '-l':
        init('ROS')
    elif sys.argv[1] == '-c':
        init('CloseLoop')
    else:
        print("invalid arguments")
        print(usage_str)