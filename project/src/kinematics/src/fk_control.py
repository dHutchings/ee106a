#!/usr/bin/env python
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import joint_pos

import sys
import curses

def keyboard_ctrl():
    screen = cures.initscr()

    while True:
        try:
            curses.noecho()
            curses.curs_set(0)
            screen.keypad(1)
            screen.addstr("Press Z, X, or arrow keys: ")
            event = screen.getch()
        finally:
            curses.endwin()

        if event == curses.KEY_LEFT:

        elif event == curses.KEY_RIGHT:

        elif event == curses.KEY_DOWN:

        elif event == curses.KEY_UP:

        elif event == 122:  # Z

        elif event == 120:  # X

        else:
            print("invalid key: %r" % event)

        

def main():
    print("Initializing node... ")
    rospy.init_node("fk_control")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    # Initialize grippers
    right_gripper = baxter_interface.gripper.Gripper('right')
    left_gripper = baxter_interface.gripper.Gripper('left')

    print('Calibrating...')
    right_gripper.calibrate()
    left_gripper.calibrate()
    rospy.sleep(2.0)

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    # Starting keyboard control
    keyboard_ctrl()

usage_str = \
"""
Forward kinematics engine: Requres baxter.sh environment.
When prompted to, you can control the end-effector position via arrow keys.

[UP, DOWN]      move along y-axis
[LEFT, RIGHT]   move along x-axis
[Z, X]          move along z-axis

Flags:
-h      print usage instructions

"""

if __name__ == '__main__':
    if len(sys.argv) == 1:
        main()
    elif sys.argv[1] == '-h':
        print(usage_str)
    else:
        print("invalid arguments")
        print(usage_str)