# #!/usr/bin/env python

# # Copyright (c) 2013-2015, Rethink Robotics
# # All rights reserved.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions are met:
# #
# # 1. Redistributions of source code must retain the above copyright notice,
# #    this list of conditions and the following disclaimer.
# # 2. Redistributions in binary form must reproduce the above copyright
# #    notice, this list of conditions and the following disclaimer in the
# #    documentation and/or other materials provided with the distribution.
# # 3. Neither the name of the Rethink Robotics nor the names of its
# #    contributors may be used to endorse or promote products derived from
# #    this software without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.

import argparse
import random
from baxter_interface import head
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

import baxter_interface

from baxter_interface import CHECK_VERSION


class Nod(object):

    def __init__(self):
        """
        'Wobbles' the head
        """
        self._done = False
        self._head = baxter_interface.Head()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting example...")
        if self._done:
            self.set_neutral()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0)

    def nod(self):
        self.set_neutral()
        """
        Performs the wobbling
        """
        self._head.command_nod()

        # command_rate = rospy.Rate(1)
        # control_rate = rospy.Rate(100)
        # start = rospy.get_time()
        # while not rospy.is_shutdown() and (rospy.get_time() - start < 10.0):
        #     angle = random.uniform(-1.5, 1.5)
        #     while (not rospy.is_shutdown() and
        #            not (abs(self._head.pan() - angle) <=
        #                baxter_interface.HEAD_PAN_ANGLE_TOLERANCE)):
        #         self._head.set_pan(angle, speed=30, timeout=0)
        #         control_rate.sleep()
        #     command_rate.sleep()

        # self._done = True
        rospy.signal_shutdown("Example finished.")


def callback():
    """RSDK Head Example: Wobbler

    Nods the head and pans side-to-side towards random angles.
    Demonstrates the use of the baxter_interface.Head class.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    nod = Nod()
    rospy.on_shutdown(nod.clean_shutdown)
    print("Nodding.. ")
    nod.nod()
    print("Done.")

def listener():

    
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("nod_messages", Bool, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

if __name__ == '__main__':
    listener()
