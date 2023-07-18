#!/usr/bin/python3

"""
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
"""

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from jenga_header import *
from jenga_kinematics import *
from std_srvs.srv import Empty

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""


def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = (
        digital_in_0 & 1
    )  # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""


def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position
    global digital_in_0

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while at_goal == 0:

        if (
            abs(thetas[0] - driver_msg.destination[0]) < 0.0005
            and abs(thetas[1] - driver_msg.destination[1]) < 0.0005
            and abs(thetas[2] - driver_msg.destination[2]) < 0.0005
            and abs(thetas[3] - driver_msg.destination[3]) < 0.0005
            and abs(thetas[4] - driver_msg.destination[4]) < 0.0005
            and abs(thetas[5] - driver_msg.destination[5]) < 0.0005
            and digital_in_0 == io_0
        ):

            # rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        # if(spin_count >  SPIN_RATE*5):

        #     pub_cmd.publish(driver_msg)
        #     rospy.loginfo("Just published again driver_msg")
        #     spin_count = 0

        if spin_count > SPIN_RATE:
            error = 1
            return error

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""

'''
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while at_goal == 0:

        if (
            abs(thetas[0] - driver_msg.destination[0]) < 0.0005
            and abs(thetas[1] - driver_msg.destination[1]) < 0.0005
            and abs(thetas[2] - driver_msg.destination[2]) < 0.0005
            and abs(thetas[3] - driver_msg.destination[3]) < 0.0005
            and abs(thetas[4] - driver_msg.destination[4]) < 0.0005
            and abs(thetas[5] - driver_msg.destination[5]) < 0.0005
        ):

            at_goal = 1
            # rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if spin_count > SPIN_RATE * 5:

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error
'''
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    if type(dest[0]) is not list: dest = [ dest ]

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest[0]
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()
    i = 0
    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):
            
            i += 1
            if i==len(dest):
                return error
            driver_msg.destination = dest[i]
            pub_cmd.publish(driver_msg)

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    # global variable1
    # global variable2
    error = 0

    offset_height = 0.1

    joint_angles = lab_invk(
        start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] + offset_height, 0
    )
    move_arm(pub_cmd, loop_rate, joint_angles, vel, accel)

    # Move Down Linear
    for i in reversed(range(10)):
        joint_angles = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] + offset_height * i/10.0, 0)
        move_arm(pub_cmd, loop_rate, joint_angles, vel, accel)
    if gripper(pub_cmd, loop_rate, suction_on) == 1:
        gripper(pub_cmd, loop_rate, suction_off)
        joint_angles = lab_invk(
            start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] + offset_height, 0
        )
        move_arm(pub_cmd, loop_rate, joint_angles, vel, accel)
        error = 1
        return error
    
    print("Picked Block")
    joint_angles = lab_invk(
        start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2] + offset_height, 0
    )
    move_arm(pub_cmd, loop_rate, joint_angles, vel, accel)

    joint_angles = lab_invk(
        target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2] + offset_height, 0
    )
    move_arm(pub_cmd, loop_rate, joint_angles, vel, accel)
    joint_angles = lab_invk(
        target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0
    )
    move_arm(pub_cmd, loop_rate, joint_angles, vel, accel)
    gripper(pub_cmd, loop_rate, suction_off)
    joint_angles = lab_invk(
        target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2] + offset_height, 0
    )
    move_arm(pub_cmd, loop_rate, joint_angles, vel, accel)

    # ========================= Student's code ends here ===========================

    return error


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize rospack
    rospack = rospkg.RosPack()
    jenga_bob_path = rospack.get_path("jenga_bob")
    yamlpath = os.path.join(jenga_bob_path, "scripts", "jenga_data.yaml")

    with open(yamlpath, "r") as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            Q = data["sim_pos"]
        except:
            print("YAML not found")
            sys.exit()

    rospy.init_node("jenga_bob")

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher("ur3/command", command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber("ur3/position", position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while rospy.is_shutdown():
        print("ROS is shutdown!")

    loop_rate = rospy.Rate(SPIN_RATE)

    time.sleep(2.0)
    #move_block(pub_command, loop_rate, (0.2, 0.25, 0.242), (0.12, 0.1, 0.02), 4.0, 4.0)
    #move_block(pub_command, loop_rate, (0.2, 0.25, 0.02), (0.12, 0.1, 0.02), 4.0, 4.0)
    
    side_gripper_on = rospy.ServiceProxy('/side_gripper/on', Empty)
    side_gripper_off = rospy.ServiceProxy('/side_gripper/off', Empty)

    h = 0.130

    joint_angles = lab_invk(0.2, -0.1, h, -90)
    move_arm(pub_command, loop_rate, joint_angles, 4.0, 4.0)

    joint_angles = lab_invk(0.2, 0.10, h, -90)
    move_arm(pub_command, loop_rate, joint_angles, 0.5, 1.0)
    side_gripper_on()
   
    traj = []
    for i in range(15):
        joint_angles = lab_invk(0.2, 0.10 - (i*0.01), h, -90)
        traj.append(joint_angles)

    move_arm(pub_command, loop_rate, traj, 4.0, 0.5)

    side_gripper_off()

    """
    rospy.loginfo("Sending Goals ...")


    loop_count = 10
    while loop_count > 0:

        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        rospy.loginfo("Sending goal 1 ...")
        move_arm(pub_command, loop_rate, Q[0][0][1], 4.0, 4.0)

        gripper(pub_command, loop_rate, suction_on)
        time.sleep(1.0)

        rospy.loginfo("Sending goal 2 ...")
        move_arm(pub_command, loop_rate, Q[1][1][1], 4.0, 4.0)

        rospy.loginfo("Sending goal 3 ...")
        move_arm(pub_command, loop_rate, Q[2][0][1], 4.0, 4.0)

        loop_count = loop_count - 1
    """


if __name__ == "__main__":

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
