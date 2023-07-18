#!/usr/bin/python3

import rospy
import rospkg
import os
import sys
import yaml
import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

if __name__ == "__main__":

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    # Initialize ROS node
    rospy.init_node("ur3_gazebo_spawner", anonymous=True)
    # Initialize ROS pack
    rospack = rospkg.RosPack()
    # Get path to block
    jenga_bob_path = rospack.get_path("jenga_bob")
    block1_path = os.path.join(jenga_bob_path, "description", "block_red.urdf")
    block2_path = os.path.join(jenga_bob_path, "description", "block_yellow.urdf")
    block3_path = os.path.join(jenga_bob_path, "description", "block_green.urdf")
    block_paths = [block3_path, block3_path, block3_path]
    # Wait for service to start
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    # Delete previous blocks
    for stack in range(10):
        for block in range(3):
            block_name = "block" + str(block + 1) + str(stack + 1)
            delete(block_name)

    tower_center = (0.2, 0.25)
    stack_height = 0.03

    # Spawn blocks
    for stack in range(1, 9):
        for block in range(3):
            block_name = "block" + str(block + 1) + str(stack + 1)

            if stack % 2 == 0:
                pose = Pose(
                    Point(
                        tower_center[0],
                        tower_center[1] - 0.033 * (block - 1),
                        stack_height * stack + 0.005,
                    ),
                    Quaternion(0, 0, -(2 ** (0.5)) / 2, 2 ** (0.5) / 2),
                )
            else:
                pose = Pose(
                    Point(
                        tower_center[0] - 0.033 * (block - 1),
                        tower_center[1],
                        stack_height * stack + 0.005,
                    ),
                    Quaternion(0, 0, 0, 0),
                )
            spawn(
                block_name,
                open(block_paths[2 - block], "r").read(),
                "block",
                pose,
                "world",
            )
            print(block_name)
