#!/usr/bin/env python3
import rospy
import time
from libs.robotarm import *

# roslaunch open_manipulator_controller open_manipulator_controller.launch usb_port:=/dev/arm

if __name__ == '__main__':
    rospy.init_node("robot_arm_demo")

    xyz_init = (0.288, 0.0, 0.194)
    xyz_home = (0.134, 0.0, 0.240)
    t = 3.0
    
    move_to(xyz_home[0], xyz_home[1], xyz_home[2], t)
    time.sleep(t)

    open_gripper(t)
    time.sleep(t)
    
    move_to(xyz_init[0], xyz_init[1], xyz_init[2], t)
    time.sleep(t)

    close_gripper(t)
    time.sleep(t)
    
    move_to(xyz_home[0], xyz_home[1], xyz_home[2], t)
    time.sleep(t)
    
    open_gripper(t)

