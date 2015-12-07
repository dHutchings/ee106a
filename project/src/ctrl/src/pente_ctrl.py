#!/usr/bin/env python
import sys
import argparse
import rospy
from std_msgs.msg import String, Uint32
from transitions import Machine
from kinematics.srv import *

global printstream

check_launch = True
is_quiet = False

class PenteFSM(object):

    originPoint = {'trans': [0.5, 0.5, 0.5], 'rot': [0., 0., 0., 0.]}
    nullCmd = ["None"]

    states = ['startup', 'standby', 'eval_board', 'mk_move',
              'exec_move', 'reset_pos', 'terminate', 'error']
    transitions = [
        {'trigger':'startup_done', 'source':'startup', 'dest':'standby'},
        {'trigger':'game_over', 'source':'standby', 'dest':'terminate', 'after':'game_over_cb'},
        {'trigger':'board_ready', 'source':'standby', 'dest':'eval_board'
            'before':'board_ready_cb', 'after':'print_to_head'},
        {'trigger':'state_ready', 'source':'eval_board', 'dest':'mk_move'},
        {'trigger':'mv_ready', 'source':'mk_move', 'dest':'exec_move'},
        {'trigger':'mv_done', 'source':'exec_move', 'dest':'reset_pos'},
        {'trigger':'wait_for_player', 'source':'reset_pos', 'dest':'standby'},

        # Error transitions
        {'trigger':'recieved_error', 'source':'exec_move', 'dest':'error'}
    ]

    def __init__(self):
        self.machine = Machine(model=self,
                               states=PenteFSM.states,
                               transitions=PenteFSM.transitions,
                               initial='startup',
                               auto_transitions=True)
        self.command = self.nullCmd

    def update_state(self):
        cmd = self.command[0]
        if self.state == 'startup':
            self.startup_done()
        elif cmd == 'c':
            print_to_stream("Given keyboard error signal.")
            self.to_error()
            flush_cmd()
        elif self.state == 'error' and cmd == 'reset':
            self.to_standby()
            flush_cmd()
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
            pass

    def game_over_cb(self):
        print_to_stream("Ending game...")
        rospy.shutdown_signal("Clean shutdown")

    def board_ready_cb(self):
        print_to_stream("Getting board state information")
        self.boardState = boardGeometrySrv()

    def print_to_head(self):
        pass

    def state_ready_cb(self):
        pickupSquare = self.command[1]
        dropoffSquare = self.command[2]
        self.targetPoints = []
        # Arm moves to high-y start point
        self.targetPoints.append(('low', self.originPoint))

        # Arm uses closed loop control to go to pickup location
        self.targetPoints.append(('high', getBoardRBT(targetSquare)))

        # Arm moves back to origin point
        self.targetPoints.append(('low', self.originPoint))

        # Arm uses closed loop control to go to dropoff location
        self.targetPoints.append(('high', getBoardRBT(dropoffSquare)))


    def flush_cmd(self):
        self.command = self.nullCmd

    def handle_keyboard(self, msg):
        print_to_stream("recieved command: "+str(msg))
        self.command = str(msg).split()


def print_to_stream(statement):
    global printstream
    if not is_quiet:
        printstream.publish(String(statement))


def main():
    global printstream

    fsm = PenteFSM()
    r = rospy.rate(10)

    print("Initializing node...")
    rospy.init_node("pente_ctrl")
    
    print("Initializing status feed...")
    printstream = rospy.Publisher('pente_ctrl/status', String, queue_size=10)
    
    print("Initializing keyboard input...")
    rospy.Subscriber('pente_ctrl/keyboard', Uint32, fsm.handle_keyboard)

    print("Connecting to closed_loop service...")
    rospy.wait_for_service("closed_loop")
    closedLoopSrv = rospy.ServiceProxy('closed_loop', closed_loop_request)

    print("Connecting to low_level_arm service...")
    rospy.wait_for_service("low_level_arm")
    lowLevelSrv = rospy.ServiceProxy('low_level_arm', kinematics_request)

    print("Connecting to board geometry service...")
    rospy.wait_for_service("board_geometry")
    boardGeometrySrv = rospy.ServiceProxy('board_geometry')

    # print("Initializing head_image feed...")
    # headImg = rospy.Publisher('pente_ctrl/head_image', image)


    print("Use pente_ctrl/keyboard topic to interface with pente_ctrl.")
    print("Echo pente_ctrl/status topic to view debug info.")
    print("Press CTRL+C to shutdown.")

    while not rospy.is_shutdown():
        fsm.update_state()
        r.sleep()



if __name__ == '__main__':
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