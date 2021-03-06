#!/usr/bin/env python
import sys
import argparse
import rospy
import baxter_interface
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from transitions import Machine
from kinematics.srv import *
from gridworld.srv import *

global printstream

check_launch = True
is_quiet = False


class PenteFSM(object):
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

    def __init__(self, head, listener):
        self.machine = Machine(model=self,
                               states=PenteFSM.states,
                               transitions=PenteFSM.transitions,
                               initial='startup',
                               auto_transitions=True)
        self.command = self.nullCmd
        self.head = head

        last_seen = listener.getLatestCommonTime('handoff_point', 'base')
        self.originPoint = listener.getTransform('handoff_point', 'base', last_seen)
        last_seen = listener.getLatestCommonTime('dropoff_point', 'base',)
        self.dropoffPoint = listener.getTransform('dropoff_point', 'base', last_seen)

    def update_state(self):
        cmd = self.command[0]
        if self.state == 'startup':
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
            print_to_stream("cannot act on command: "+cmd)
            if not is_quiet:
                print_to_stream(str(self.state))

    def startup_done_cb(self):
        print_to_stream("Ready to play!")
        self.nod_head()

    def game_over_cb(self):
        print_to_stream("Ending game...")
        self.nod_head()
        rospy.signal_shutdown("Clean shutdown")

    def board_ready_cb(self):
        print_to_stream("Getting board state information")
        result = self.boardGeometrySrv(request="GIMME YO INFO")
        self.boardState = eval(result.board_state)
        self.nod_head()

    def state_ready_cb(self):
        pieces = {sq: self.boardState[sq] for sq in self.boardState 
                                          if self.boardState[sq][1] != None}

        for square, infoTuple in pieces.items():
            # Arm moves to high-y start point
            self.targetPoints.append(('low_ng', self.originPoint))

            # Arm uses closed loop control to go to pickup location
            self.targetPoints.append(('high', infoTuple[1], infoTuple[2]))

            # Arm moves back to origin point
            self.targetPoints.append(('low_wg', self.originPoint))

            # Arm uses open loop control to go to dropoff location
            self.targetPoints.append(('low_wg', self.dropoff_point))

    def mv_ready_cb(self):
        for inst in self.targetPoints:
            if inst[0] == 'low_ng':
                result = self.lowLevelSrv(trans=inst[1]['trans'], rot=inst[1]['rot'], 
                                     grip='False', keep_orient='False')
            elif inst[0] == 'low_wg':
                result = self.lowLevelSrv(trans=inst[1]['trans'], rot=inst[1]['rot'], 
                                     grip='True', keep_orient='False')
            elif inst[0] == 'high':
                result = self.closedLoopSrv(inst[1], inst[2], "pick")

            # if result != True:
            #     print_to_stream("FAILED INSTRUCTION: "+str(inst))
            #     self.to_error()

    def mv_done_cb(self):
        result = self.lowLevelSrv()
        # if result != True:
        #     print_to_stream("FAILED INSTRUCTION: <return to standy position>")
        #     self.to_error()
        self.nod_head()
        self.to_standby()

    def nod_head(self):
        self.head.command_nod()

    def flush_cmd(self):
        self.command = self.nullCmd

    def handle_keyboard(self, msg):
        print_to_stream("recieved command: "+str(msg.data))
        self.command = str(msg.data).split()


def print_to_stream(statement):
    global printstream
    printstream.publish(data=statement)


def main():
    global printstream

    print("Initializing node...")
    rospy.init_node("pente_ctrl")
    r = rospy.Rate(10)
    
    print("Initializing FSM...")
    l = tf.TransformListener()
    h = baxter_interface.Head()
    fsm = PenteFSM(head=h, listener=l)
    
    print("Initializing status feed...")
    printstream = rospy.Publisher('/pente_ctrl/status', String, queue_size=10)

    print("Initializing keyboard input...")
    rospy.Subscriber('pente_ctrl/keyboard', String, fsm.handle_keyboard)

    print("Connecting to closed_loop service...")
    rospy.wait_for_service("closed_loop")
    closedLoopSrv = rospy.ServiceProxy('closed_loop', closed_loop_request)

    print("Connecting to low_level_arm service...")
    rospy.wait_for_service("low_level_arm")
    lowLevelSrv = rospy.ServiceProxy('low_level_arm', kinematics_request)

    print("Connecting to board geometry service...")
    rospy.wait_for_service("board_geometry")
    boardGeometrySrv = rospy.ServiceProxy('board_state', board_state)

    fsm.closedLoopSrv = closedLoopSrv
    fsm.lowLevelSrv = lowLevelSrv
    fsm.boardGeometrySrv = boardGeometrySrv

    print("Use pente_ctrl/keyboard topic to interface with pente_ctrl.")
    print("Echo pente_ctrl/status topic to view debug info.")
    print("Press CTRL+C to shutdown.")

    while not rospy.is_shutdown():
        fsm.update_state()
        r.sleep()



if __name__ == '__main__':
    # parser = argparse.ArgumentParser(description="The One Pente Controller to rule them all.")
    # parser.add_argument('-f', '--force', help="start pente_ctrl without running launch checks.",
    #                                      action='store_true')
    # parser.add_argument('-q', '--quiet', help="disable print statements",
    #                                       action='store_true')
    # args = parser.parse_args()
    # if args.force:
    #     check_launch = False
    # if args.quiet:
    #     is_quiet = True

    main()