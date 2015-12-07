#!/usr/bin/env python
import sys
import rospy
# import roslib; roslib.load_manifest('kinematics')
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped


usage_str = \
"""
Inverse kinematics script

Requires baxter.sh and abxter_moveit_noexec.launch to be run beforehand.
Interactive mode allows control of the end-effector position via 
text input or arrow keys.

Keyboard-interactive controls:
[UP, DOWN]      move along y-axis
[LEFT, RIGHT]   move along x-axis
[Z, X]          move along z-axis
[G]             toggle gripper

Flags:
--help, -h      print usage instructions
-k              run interactive keyboard interface
-t              run interactive text interface
"""




def move_to_coord(trans, rot, arm, which_arm='left', keep_oreint=False):
    goal = PoseStamped()
    goal.header.frame_id = "base"

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
        orien_const.orientation.y = -1.0;
        orien_const.absolute_x_axis_tolerance = 0.1;
        orien_const.absolute_y_axis_tolerance = 0.1;
        orien_const.absolute_z_axis_tolerance = 0.1;
        orien_const.weight = 1.0;
        consts = Constraints()
        consts.orientation_constraints = [orien_const]
        arm.set_path_constraints(consts)

    # Plan a path
    arm_plan = arm.plan()

    # Execute the plan
    arm.execute(arm_plan)

def text_ctrl():
    return

def keyboard_ctrl(which_arm, arm, gripper):
    gripper_closed = False
    limb = baxter_interface.Limb(which_arm)
    pose = limb.endpoint_pose()
    pose = {'rot': list(pose['orientation']), 
            'trans' : list(pose['position'])}
    screen = curses.initscr()

    try:
        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        screen.keypad(1)

        while True:
            event = screen.getch()
            # print(pose)

            if event == curses.KEY_LEFT:
                pose['trans'][0] += 0.1
                print(pose['trans'])
                move_to_coord(pose['trans'], pose['rot'], arm, which_arm)
            elif event == curses.KEY_RIGHT:
                pose['trans'][0] -= 0.1
                move_to_coord(pose['trans'], pose['rot'], arm, which_arm)
            elif event == curses.KEY_DOWN:
                pose['trans'][1] += 0.1
                move_to_coord(pose['trans'], pose['rot'], arm, which_arm)
            elif event == curses.KEY_UP:
                pose['trans'][1] -= 0.1
                move_to_coord(pose['trans'], pose['rot'], arm, which_arm)
            elif event == 122:  # Z
                pose['trans'][2] += 0.1
                move_to_coord(pose['trans'], pose['rot'], arm, which_arm)
            elif event == 120:  # X
                pose['trans'][2] -= 0.1
                move_to_coord(pose['trans'], pose['rot'], arm, which_arm)
            elif event == 103:  # G
                if gripper_closed:
                    gripper.open(block=False)
                    gripper_closed = False
                else:
                    gripper.close(block=False)
                    gripper_closed = True

            elif event == 27:   # Esc
                break
            else:
                print("invalid key: %r" % event)
    finally:
        screen.keypad(0)
        curses.curs_set(1)
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        rospy.signal_shutdown("Shutdown signal recieved")
        

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
        keyboard_ctrl('left', left_arm, left_gripper)
    elif mode == 'text':
        text_ctrl('left', left_arm, left_gripper)
    elif mode == 'ROS':
        # Set up listener node
        
        rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        init()
    elif sys.argv[1] == '-h':
        print(usage_str)
    elif sys.argv[1] == '--help':
        print(usage_str)
    elif sys.argv[1] == '-k':
        import curses
        init('keyboard')
    elif sys.argv[1] == '-t':
        init('text')
    else:
        print("invalid arguments")
        print(usage_str)