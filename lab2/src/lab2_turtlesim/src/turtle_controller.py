#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy

#Import topic to run the turtle
from geometry_msgs.msg import Twist
import sys
import curses
import signal
import time

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1) #enables keypad
stdscr.nodelay(1) #non-blocking!
#stdscr.timeout(1)

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        curses.endwin()
        print("Done")
        sys.exit(0)

turtleToControl = 'turtle1'


#Define the method which contains the main functionality of the node.
def talker():

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('talker', anonymous=True)

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type std_msgs/String to the topic as listed below by inspection using
  # "rosnode info /teleop_turtle 
  pub = rospy.Publisher(turtleToControl + '/cmd_vel', Twist, queue_size=10)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  oldTwist = Twist() 
  key = ''
  string = "foo"
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    #print("CMDS are WSADQE, w fwd, S back, A strafe right, B strafe left, Q rotate left, E rotate right")
    #s = raw_input('-->')

    if string is "foo":
        string = "FOO"
    else:
        string = "foo"
    stdscr.addstr(1,10,string)
    #heartbeat

    twist = oldTwist

    key = stdscr.getch() #returns -1 for no character

    if key is not -1:
        stdscr.addch(20,25,key)
        stdscr.refresh()
        twist.linear.x = oldTwist.linear.x
        if key == curses.KEY_UP: 
            stdscr.addstr(2, 20, "Up")
            twist.linear.x = oldTwist.linear.x + 1
        else:
            stdscr.addstr(2,20, "  ")
        if key == curses.KEY_DOWN: 
            stdscr.addstr(3, 20, "Down")
            twist.linear.x = oldTwist.linear.x - 1
        else:
            stdscr.addstr(3,20,"    ")

        if key == curses.KEY_LEFT: 
            stdscr.addstr(4, 20, "Left")
            twist.angular.z = oldTwist.angular.z + 1
        else:
            stdscr.addstr(4,20, "    ")
        if key == curses.KEY_RIGHT: 
            stdscr.addstr(5, 20, "Right")
            twist.angular.z = oldTwist.angular.z - 1
        else:
            stdscr.addstr(5,20,"      ")

    
    oldTwist = twist

    twist.linear.z = 0  #should always be zero, we're not levitating
    twist.angular.x = 0  #rotate about the x axis... this should be zero
    twist.angular.y = 0  #should also always be zero

    # Publish our string to the 'chatter_talk' topic
    pub.publish(twist)
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  if len(sys.argv) >1:
    print("New Turtle, it's name is "+ sys.argv[1])
    turtleToControl = sys.argv[1]
  try:
    stdscr.addstr(0,10,"Use ctrl-c to quit")
    stdscr.refresh()
    signal.signal(signal.SIGINT, signal_handler)
    talker()
  except rospy.ROSInterruptException: pass
  
