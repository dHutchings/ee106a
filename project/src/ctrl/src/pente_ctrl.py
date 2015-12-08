#!/usr/bin/env python
import sys
import argparse
import rospy
import baxter_interface
import tf
import Image, ImageDraw, ImageFont
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from transitions import Machine
from kinematics.srv import *
from headcam.srv import *
#from board_geometry import loc_to_marker, marker_to_loc

global printstream

check_launch = True
is_quiet = False

class PenteFSM(object):

    originPoint = {'trans': None, 'rot': None}
    nullCmd = ["None"]

    states = ['startup', 'standby', 'eval_board', 'mk_move',
              'exec_move', 'reset_pos', 'terminate', 'error']
    transitions = [
        {'trigger':'startup_done', 'source':'startup', 'dest':'standby',
            'before':'startup_done_cb'},
        {'trigger':'game_over', 'source':'standby', 'dest':'terminate', 
            'after':'game_over_cb'},
        {'trigger':'board_ready', 'source':'standby', 'dest':'eval_board',
            'before':'board_ready_cb', 'after':'print_to_head'},
        {'trigger':'state_ready', 'source':'eval_board', 'dest':'mk_move',
            'after':'state_ready_cb'},
        {'trigger':'mv_ready', 'source':'mk_move', 'dest':'exec_move',
            'after':'mv_ready_cb'},
        {'trigger':'mv_done', 'source':'exec_move', 'dest':'reset_pos',
            'after':'mv_done_cb'},
        {'trigger':'wait_for_player', 'source':'reset_pos', 'dest':'standby'},
    ]

    def __init__(self, head):
        self.machine = Machine(model=self,
                               states=PenteFSM.states,
                               transitions=PenteFSM.transitions,
                               initial='startup',
                               auto_transitions=True)
        self.command = self.nullCmd
        self.head = head

    def update_state(self):
        cmd = self.command[0]
        if self.state == 'startup' and cmd == 'start':
            # Callback: finds the new tf of large AR Tag
            self.startup_done()
        elif cmd == 'c':
            print_to_stream("Given keyboard error signal.")
            self.to_error()
            self.flush_cmd()
        elif self.state == 'error' and cmd == 'reset':
            self.to_standby()
            self.flush_cmd()
        elif self.state == 'standby' and cmd == 'end':
            # Callback: Terminates game
            self.game_over()
            self.flush_cmd()
        elif self.state == 'standby' and cmd == 'read':
            # Callback: Call service to get board state
            # Post-Callback: Prints board state to headscreen
            self.board_ready()
            self.flush_cmd()
        elif self.state == 'eval_board' and cmd == 'plan':
            # Callback: create target points
            self.state_ready()
            self.flush_cmd()
        elif self.state == 'mk_move' and cmd == 'exec':
            # Callback: plan with moveit and execute for each position
            self.mv_ready()
            self.flush_cmd()
        elif self.state == 'mv_done' and cmd == 'standby':
            # Callback: move to ready position
            self.mv_done()
            self.flush_cmd()
        else:
            if not is_quiet:
                print_to_stream(str(self.state))

    # Callback functions
    def startup_done_cb(self):
        print_to_stream("Ready to play!")
        self.calculate_origin_point()
        self.nod_head()

    def game_over_cb(self):
        print_to_stream("Ending game...")
        self.nod_head()
        rospy.signal_shutdown("Clean shutdown")

    def board_ready_cb(self):
        print_to_stream("Getting board state information")
        self.nod_head()

    def state_ready_cb(self):
        # Move sequence for closed-loop pickup and dropoff

        pickupSquare = eval(self.command[1])
        dropoffSquare = eval(self.command[2])
        self.targetPoints = []
        # Arm moves to origin point
        self.targetPoints.append(('low_ng', self.originPoint))

        # Arm uses closed loop control to go to pickup location
        infoTuple = self.boardState[pickupSquare]
        self.targetPoints.append(('high', infoTuple[1], infoTuple[2]))

        # Arm moves back to origin point
        self.targetPoints.append(('low_wg', self.originPoint))

        # Arm uses closed loop control to go to dropoff location
        infoTuple = self.boardState[dropoffSquare]
        self.targetPoints.append(('high', infoTuple[1], infoTuple[2]))

    def mv_ready_cb(self):
        for inst in self.targetPoints:
            if inst[0] == 'low_ng':
                result = lowLevelSrv(target_rans=inst[1]['trans'], target_rot=inst[1]['rot'], 
                                     grip='False', keep_orient='False')
            elif inst[0] == 'low_wg':
                result = lowLevelSrv(target_rans=inst[1]['trans'], target_rot=inst[1]['rot'], 
                                     grip='True', keep_orient='False')
            elif inst[0] == 'high':
                result = closedLoopSrv(inst[1], inst[2], "pick")

            # if result != True:
            #     print_to_stream("FAILED INSTRUCTION: "+str(inst))
            #     self.to_error()

    def mv_done_cb(self):
        result = lowLevelSrv()
        # if result != True:
        #     print_to_stream("FAILED INSTRUCTION: <return to standy position>")
        #     self.to_error()
        self.nod_head()
        self.to_standby()

    # Helper methods
    def calculate_origin_point(self):
        # Still not sure how to set this up...
        pass

    def print_to_head(self):
        # Right now, just prints to pente_ctrl/status
        boardstr = printBoard(self.boardState)
        for row in boardstr: print_to_stream(data=row)

    def nod_head(self):
        self.head.command_nod()

    def flush_cmd(self):
        self.command = self.nullCmd

    def handle_keyboard(self, msg):
        print_to_stream("recieved command: "+str(msg.data))
        self.command = str(msg.data).split()

    def handle_board_state(self, msg):
        self.boardState = eval(msg.data)


def print_to_stream(statement):
    global printstream

    printstream.publish(data=statement)

def getBoardRBT(board, location):
    return location

def printBoard(board):
    def char_to_int(c):
        return ord(c) - 65

    arr = [[[] for j in range(8)] for i in range(8)]
    for char, num in board.keys():
        if char_to_int(char) < 8 and num < 8:
            arr[char_to_int(char)][num] = [board[(char, num)][0]]
    return arr

def main():
    global printstream

    print("Initializing node...")
    rospy.init_node("pente_ctrl")
    r = rospy.Rate(10)

    h = baxter_interface.Head()
    fsm = PenteFSM(head=h)
    
    print("Initializing status feed...")
    printstream = rospy.Publisher('/pente_ctrl/status', String, queue_size=10)

    rospy.sleep(5)

    print_to_stream("Initializing keyboard input...")
    rospy.Subscriber('pente_ctrl/keyboard', String, fsm.handle_keyboard)

    print_to_stream("Connecting to closed_loop service...")
    rospy.wait_for_service("closed_loop")
    closedLoopSrv = rospy.ServiceProxy('closed_loop', closed_loop_request)

    print_to_stream("Connecting to low_level_arm service...")
    rospy.wait_for_service("low_level_arm")
    lowLevelSrv = rospy.ServiceProxy('low_level_arm', kinematics_request)

    print_to_stream("Initializing board state input...")
    rospy.Subscriber("pente_ctrl/board_state", String, fsm.handle_board_state)

    print_to_stream("Connecting to TransSrv service...")
    rospy.wait_for_service("new_tf_frames")
    newTFSrv = tf.TransformListener()

    # print("Initializing head_image feed...")
    # headImg = rospy.Publisher('pente_ctrl/head_image', image)


    print_to_stream("Use pente_ctrl/keyboard topic to interface with pente_ctrl.")
    print_to_stream("Echo pente_ctrl/status topic to view debug info.")
    print_to_stream("Press CTRL+C to shutdown.")

    while not rospy.is_shutdown():
        fsm.update_state()
        r.sleep()



if __name__ == '__main__':
    '''
    parser = argparse.ArgumentParser(description="The One Pente Controller to rule them all.")
    parser.add_argument('-f', '--force', help="start pente_ctrl without running launch checks.",
                                         action='store_true')
    parser.add_argument('-q', '--quiet', help="disable print statements",
                                          action='store_true')
    args = parser.parse_args()
    if args.force:
        check_launch = False
    if args.quiet:
        is_quiet = True

    '''
    main()