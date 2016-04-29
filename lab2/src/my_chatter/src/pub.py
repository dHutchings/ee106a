#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy

#Import the String message type from the /msg directory of
#the std_msgs package.
from std_msgs.msg import String
from my_chatter.msg import TimestampString

#Define the method which contains the main functionality of the node.
def talker():

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('talker', anonymous=True)

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type TimestampString to the topic /user_messages
  pub = rospy.Publisher('user_messages', TimestampString, queue_size=10)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    print("Please enter a line of text and press <Enter>:")
    s = raw_input('-->')
    
    # Construct an array that we want to publish
    #include s
    #data = [s , rospy.get_time()]
    
    #explicit way to instantiate a message
    data = TimestampString()
    #fill each data type
    data.userInput = s
    data.timestamp = rospy.get_time()

    pub.publish(data)

    # One way to Publish our data to the 'chatter_talk' topic
    # uses implicit type definitions to get the message right.
    # pub.publish(s,rospy.get_time())
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  try:
    talker()
  except rospy.ROSInterruptException: pass
