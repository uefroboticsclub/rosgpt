
import sys

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time 
# This script makes the end-effector perform pick, pour, and place tasks
# Note that this script may not work for every arm as it was designed for the wx250
# Make sure to adjust commanded joint positions and poses as necessary
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python bartender.py  # python3 bartender.py if using ROS Noetic'

def main():
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    
    #bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_home_pose()
    #bot.arm.set_single_joint_position("elbow", np.pi/3.0)
    #bot.arm.set_single_joint_position("wrist_angle", 100.0)
    #bot.arm.set_single_joint_position("shoulder", np.pi/7)
    #bot.arm.set_single_joint_position("wrist_angle", 0.00000000000050)
    time.sleep(3)
    bot.gripper.grasp()
    bot.arm.set_single_joint_position("waist", np.pi/-2.0)
    bot.gripper.release()
    """bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    bot.arm.set_single_joint_position("waist", np.pi/2.0)bot.arm.go_to_sleep_pose()
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.open()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()"""

    bot.arm.go_to_sleep_pose()
    bot.shutdown()


if __name__=='__main__':
    main()
