#!/usr/bin/env python
import sys
import rospy
from transitions import Machine

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




if __name__ == '__main__':
    fsm = PenteFSM()