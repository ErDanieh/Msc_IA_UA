#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for receiving and interpreting the status of a 2F gripper.

This serves as an example for receiving messages from the 'Robotiq2FGripperRobotInput' topic using the 'Robotiq2FGripper_robot_input' msg type and interpreting the corresponding status of the 2F gripper.
"""

import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg


def printStatus(status):
    """Print the status string generated by the statusInterpreter function."""

    print(statusInterpreter(status))


def Robotiq2FGripperStatusListener():
    """Initialize the node and subscribe to the Robotiq2FGripperRobotInput topic."""

    rospy.init_node("Robotiq2FGripperStatusListener")
    rospy.Subscriber(
        "Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, printStatus
    )
    rospy.spin()


def statusInterpreter(status):
    """Generate a string according to the current value of the status variables."""

    output = "\n-----\n2F gripper status interpreter\n-----\n"

    # gACT
    output += "gACT = " + str(status.gACT) + ": "
    if status.gACT == 0:
        output += "Gripper reset\n"
    if status.gACT == 1:
        output += "Gripper activation\n"

    # gGTO
    output += "gGTO = " + str(status.gGTO) + ": "
    if status.gGTO == 0:
        output += "Standby (or performing activation/automatic release)\n"
    if status.gGTO == 1:
        output += "Go to Position Request\n"

    # gSTA
    output += "gSTA = " + str(status.gSTA) + ": "
    if status.gSTA == 0:
        output += "Gripper is in reset ( or automatic release ) state. see Fault Status if Gripper is activated\n"
    if status.gSTA == 1:
        output += "Activation in progress\n"
    if status.gSTA == 2:
        output += "Not used\n"
    if status.gSTA == 3:
        output += "Activation is completed\n"

    # gOBJ
    output += "gOBJ = " + str(status.gOBJ) + ": "
    if status.gOBJ == 0:
        output += "Fingers are in motion (only meaningful if gGTO = 1)\n"
    if status.gOBJ == 1:
        output += "Fingers have stopped due to a contact while opening\n"
    if status.gOBJ == 2:
        output += "Fingers have stopped due to a contact while closing \n"
    if status.gOBJ == 3:
        output += "Fingers are at requested position\n"

    # gFLT
    output += "gFLT = " + str(status.gFLT) + ": "
    if status.gFLT == 0x00:
        output += "No Fault\n"
    if status.gFLT == 0x05:
        output += "Priority Fault: Action delayed, initialization must be completed prior to action\n"
    if status.gFLT == 0x07:
        output += "Priority Fault: The activation bit must be set prior to action\n"
    if status.gFLT == 0x09:
        output += "Minor Fault: The communication chip is not ready (may be booting)\n"
    if status.gFLT == 0x0B:
        output += "Minor Fault: Automatic release in progress\n"
    if status.gFLT == 0x0E:
        output += "Major Fault: Overcurrent protection triggered\n"
    if status.gFLT == 0x0F:
        output += "Major Fault: Automatic release completed\n"

    # gPR
    output += "gPR = " + str(status.gPR) + ": "
    output += (
        "Echo of the requested position for the Gripper: " + str(status.gPR) + "/255\n"
    )

    # gPO
    output += "gPO = " + str(status.gPO) + ": "
    output += "Position of Fingers: " + str(status.gPO) + "/255\n"

    # gCU
    output += "gCU = " + str(status.gCU) + ": "
    output += "Current of Fingers: " + str(status.gCU * 10) + " mA\n"

    return output


if __name__ == "__main__":
    Robotiq2FGripperStatusListener()
