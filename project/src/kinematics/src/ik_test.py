#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Set up the left gripper
    right_gripper = baxter_gripper.Gripper('right')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

# {'position': Point(x=0.5654423741921746, y=-0.39991433652757463, z=-0.19052138622665094), 
# 'orientation': Quaternion(x=0.8802143160297243, y=-0.4698440469197714, z=0.04719427050757002, w=0.047350081960167295)}


    #x, y, and z position
    goal_1.pose.position.x = 0.5654423741921746
    goal_1.pose.position.y = -0.39991433652757463
    goal_1.pose.position.z = -0.19052138622665094
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.8802143160297243
    goal_1.pose.orientation.y = -0.4698440469197714
    goal_1.pose.orientation.z = 0.04719427050757002
    goal_1.pose.orientation.w = 0.047350081960167295

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal_1)

    #Set the start state for the left arm
    right_arm.set_start_state_to_current_state()

    #Plan a path
    right_plan = right_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
    right_arm.execute(right_plan)

    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    # rospy.sleep(2.0)

    #Close the left gripper
    print('Closing...')
    right_gripper.close(block=False)
    # rospy.sleep(1.0)

    # #Open the left gripper
    # print('Opening...')
    # right_gripper.open(block=True)
    # rospy.sleep(1.0)
    # print('Done!')

    #Second goal pose -----------------------------------------------------
    # rospy.sleep(2.0)
    goal_2 = PoseStamped()
    goal_2.header.frame_id = "base"

# {'position': Point(x=0.5670790297803552, y=-0.3636742117673457, z=-0.1469551741576185), 
# 'orientation': Quaternion(x=0.8613858578807841, y=-0.506673657464994, z=-0.0033540760180909047, w=0.03584632265792695)}



    #x, y, and z position
    goal_2.pose.position.x = 0.5670790297803552
    goal_2.pose.position.y = -0.3636742117673457
    goal_2.pose.position.z = -0.1469551741576185
    
    #Orientation as a quaternion
    goal_2.pose.orientation.x = 0.8613858578807841
    goal_2.pose.orientation.y = -0.506673657464994
    goal_2.pose.orientation.z = -0.0033540760180909047
    goal_2.pose.orientation.w = 0.03584632265792695

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal_2)

    #Set the start state for the left arm
    right_arm.set_start_state_to_current_state()

    #Create a path constraint for the arm
    #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    orien_const = OrientationConstraint()
    orien_const.link_name = "left_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    right_arm.set_path_constraints(consts)

    #Plan a path
    right_plan = right_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 2: ')
    right_arm.execute(right_plan)

    #Calibrate the gripper (other commands won't work unless you do this first)
    # print('Calibrating...')
    # right_gripper.calibrate()
    rospy.sleep(2.0)

    # #Close the left gripper
    # print('Closing...')
    # right_gripper.close(block=True)
    rospy.sleep(1.0)

    #Open the left gripper
    print('Opening...')
    right_gripper.open(block=True)
    # rospy.sleep(1.0)
    print('Done!')


#     #Third goal pose -----------------------------------------------------
#     # rospy.sleep(2.0)
#     goal_3 = PoseStamped()
#     goal_3.header.frame_id = "base"

# # {'position': Point(x=0.5637407148029354, y=-0.3373444357001083, z=-0.1817724898310274), 
# # 'orientation': Quaternion(x=0.9129644602452153, y=-0.40726680121266795, z=0.014017317687426981, w=0.020812538624921138)}



#     #x, y, and z position
#     goal_3.pose.position.x = 0.5637407148029354
#     goal_3.pose.position.y = -0.3373444357001083
#     goal_3.pose.position.z = -0.1817724898310274
    
#     #Orientation as a quaternion
#     goal_3.pose.orientation.x = 0.9129644602452153
#     goal_3.pose.orientation.y = -0.40726680121266795
#     goal_3.pose.orientation.z = 0.014017317687426981
#     goal_3.pose.orientation.w = 0.020812538624921138

#     #Set the goal state to the pose you just defined
#     right_arm.set_pose_target(goal_3)

#     #Set the start state for the left arm
#     right_arm.set_start_state_to_current_state()

#     #Create a path constraint for the arm
#     #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
#     orien_const = OrientationConstraint()
#     orien_const.link_name = "left_gripper";
#     orien_const.header.frame_id = "base";
#     orien_const.orientation.y = -1.0;
#     orien_const.absolute_x_axis_tolerance = 0.1;
#     orien_const.absolute_y_axis_tolerance = 0.1;
#     orien_const.absolute_z_axis_tolerance = 0.1;
#     orien_const.weight = 1.0;
#     consts = Constraints()
#     consts.orientation_constraints = [orien_const]
#     right_arm.set_path_constraints(consts)

#     #Plan a path
#     right_plan = right_arm.plan()

#     #Execute the plan
#     raw_input('Press <Enter> to move the left arm to goal pose 3: ')
#     right_arm.execute(right_plan)

#     #Calibrate the gripper (other commands won't work unless you do this first)
#     print('Calibrating...')
#     right_gripper.calibrate()
#     # rospy.sleep(2.0)

#     #Close the left gripper
#     print('Closing...')
#     right_gripper.close(block=False)
#     # rospy.sleep(1.0)

#     # #Open the left gripper
#     # print('Opening...')
#     # right_gripper.open(block=True)
#     # rospy.sleep(1.0)
#     # print('Done!')

#     #Fourth goal pose -----------------------------------------------------
#     # rospy.sleep(2.0)
#     goal_4 = PoseStamped()
#     goal_4.header.frame_id = "base"

# # {'position': Point(x=0.5637407148029354, y=-0.3373444357001083, z=-0.1817724898310274), 
# # 'orientation': Quaternion(x=0.9129644602452153, y=-0.40726680121266795, z=0.014017317687426981, w=0.020812538624921138)}



#     #x, y, and z position
#     goal_4.pose.position.x = 0.5637407148029354
#     goal_4.pose.position.y = -0.3373444357001083
#     goal_4.pose.position.z = 0
    
#     #Orientation as a quaternion
#     goal_4.pose.orientation.x = 0.9129644602452153
#     goal_4.pose.orientation.y = -0.40726680121266795
#     goal_4.pose.orientation.z = 0.014017317687426981
#     goal_4.pose.orientation.w = 0.020812538624921138

#     #Set the goal state to the pose you just defined
#     right_arm.set_pose_target(goal_3)

#     #Set the start state for the left arm
#     right_arm.set_start_state_to_current_state()

#     #Create a path constraint for the arm
#     #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
#     orien_const = OrientationConstraint()
#     orien_const.link_name = "left_gripper";
#     orien_const.header.frame_id = "base";
#     orien_const.orientation.y = -1.0;
#     orien_const.absolute_x_axis_tolerance = 0.1;
#     orien_const.absolute_y_axis_tolerance = 0.1;
#     orien_const.absolute_z_axis_tolerance = 0.1;
#     orien_const.weight = 1.0;
#     consts = Constraints()
#     consts.orientation_constraints = [orien_const]
#     right_arm.set_path_constraints(consts)

#     #Plan a path
#     right_plan = right_arm.plan()

#     #Execute the plan
#     raw_input('Press <Enter> to move the left arm to goal pose 4: ')
#     right_arm.execute(right_plan)

#     #Calibrate the gripper (other commands won't work unless you do this first)
#     # print('Calibrating...')
#     # right_gripper.calibrate()
#     rospy.sleep(2.0)

#     # #Close the left gripper
#     # print('Closing...')
#     # right_gripper.close(block=True)
#     rospy.sleep(1.0)

#     #Open the left gripper
#     print('Opening...')
#     right_gripper.open(block=True)
#     rospy.sleep(1.0)
#     print('Done!')

if __name__ == '__main__':
    main()