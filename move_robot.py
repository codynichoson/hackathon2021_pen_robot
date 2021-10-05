from interbotix_xs_modules.arm import InterbotixManipulatorXS
import modern_robotics as mr
import pyrealsense2 as rs
import numpy as np
import time

# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

def sleep_pose():
    robot.arm.go_to_sleep_pose()
    return

def home_pose():
    robot.arm.go_to_home_pose()
    return

def move_waist(X, Y, D):
    theta = np.arctan2((D - 0.3), (0.3 + X))
    #print(f"Adjacent = {0.25 + X}")
    #print(f"Opposite = {D - 0.3}")
    #print(f"Theta = {np.degrees(theta)}")
    robot.arm.set_single_joint_position("waist", theta)
    return

def move_forward(X, Y, D):
    c = ((0.25 + X)**(2) + (0.3 - D)**(2))**(1/2)
    status = robot.arm.set_ee_cartesian_trajectory(x=c-0.08, z=0.05)
    return status

def grab_pen():
    robot.gripper.set_pressure(2.0)
    robot.gripper.close()
    robot.arm.go_to_home_pose()
    time.sleep(1)
    robot.gripper.open()
    robot.arm.go_to_sleep_pose()
    robot.gripper.close()
    return

def open_grip():
    robot.gripper.open()
    return

def check_pos():
    joints = robot.arm.get_joint_commands()
    T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
    [R, p] = mr.TransToRp(T) # get the rotation matrix and the displacement
    print(f"R = {R}\np = {p}")

def move_wrist():
    robot.arm.set_single_joint_position("wrist_angle", 0.0)


    

