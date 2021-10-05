from interbotix_xs_modules.arm import InterbotixManipulatorXS
import modern_robotics as mr
import numpy as np
import time

# TOGGLE BETWEEN HOME AND SLEEP
# # The robot object is what you use to control the robot
# robot = InterbotixManipulatorXS("px100", "arm", "gripper")
# mode = 'h'
# # Let the user select the position
# while mode != 'q':
#     mode=input("[h]ome, [s]leep, [q]uit ")
#     if mode == "h":
#         robot.arm.go_to_home_pose()
#     elif mode == "s":
#         robot.arm.go_to_sleep_pose()

# OPEN AND CLOSE GRIPPER
# def main():
#     arm = InterbotixManipulatorXS("px100", "arm", "gripper")
#     arm.gripper.close(5.0)
#     arm.gripper.open(2.0)
#     arm.gripper.set_pressure(1.0)
#     arm.gripper.close(2.0)
#     arm.gripper.open(2.0)

# if __name__=='__main__':
#     main()


# BARTENDER ROUTINE
# def main():
#     bot = InterbotixManipulatorXS("px100", "arm", "gripper")
#     bot.arm.go_to_home_pose()
#     bot
#     bot.arm.set_single_joint_position("waist", np.pi/2.0)
#     bot.gripper.open()
#     bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
#     bot.gripper.close()
#     bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
#     bot.arm.set_single_joint_position("waist", -np.pi/2.0)
#     bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
#     bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
#     bot.arm.set_single_joint_position("waist", np.pi/2.0)
#     bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
#     bot.gripper.open()
#     bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
#     bot.arm.go_to_home_pose()
#     bot.arm.go_to_sleep_pose()

# if __name__=='__main__':
#     main()

# Moving Arm to Coordinates
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
robot.arm.go_to_home_pose()
robot.gripper.set_pressure(2)
robot.gripper.open()
robot.gripper.close()
time.sleep(2)
robot.gripper.open()

#robot.arm.go_to_home_pose()
#x = robot.arm.set_ee_cartesian_trajectory(x = 0.15)
#print(x)

#robot.arm.go_to_home_pose()
#robot.arm.set_single_joint_position("waist", np.pi/2.0)
