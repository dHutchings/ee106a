#!/usr/bin/env python
import sys
import argparse
import rospy
from std_msgs.msg import String, Uint32
from transitions import Machine

global printstream

check_launch = True
is_quiet = False

class PenteFSM(object):

    states = ['startup', 'standby', 'eval_board', 'mk_move',
              'exec_move', 'reset_pos', 'terminate', 'error']
    transitions = [
        {'trigger':'startup_done', 'source':'startup', 'dest':'standby'},
        {'trigger':'game_over', 'source':'standby', 'dest':'terminate'},
        {'trigger':'board_ready', 'source':'standby', 'dest':'eval_board'},
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
                               auto_transitions=False)

    def update_state(self):
    	return


def print_to_stream(statement):
	global printstream
	if not is_quiet:
		printstream.publish(String(statement))


def handle_keyboard(msg):
	pass


def main():
	global printstream

	fsm = PenteFSM()
	r = rospy.rate(10)

	print("Initializing node...")
	rospy.init_node("pente_ctrl")
	print("Initializing status feed...")
	printstream = rospy.Publisher('pente_ctrl/status', String, queue_size=10)
	print("Initializing keyboard input...")
	rospy.Subscriber('pente_ctrl/keyboard', Uint32, handle_keypress)

	print("Use pente_ctrl/keyboard topic to interface with pente_ctrl.")
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