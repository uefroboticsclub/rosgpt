from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time
import numpy as np

# This script commands some arbitrary positions to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
# Then change to this directory and type 'python joint_position_control.py  # python3 bartender.py if using ROS Noetic'

def main():
    #joint_positions = [-1.0, 0.5 , 0.5, 0, -0.5, 1.57]

    joint_positions1 = [0, 0.8 , 0, -0.8]
    bot = InterbotixManipulatorXS("px100", "arm", "gripper")
    
    bot.arm.go_to_home_pose()
    bot.gripper.grasp()
    
    bot.arm.set_joint_positions(joint_positions1)
    
    time.sleep(1)
    
    bot.arm.go_to_home_pose()
    bot.arm.set_single_joint_position("waist", np.pi/-2.0)
    joint_positions2 = [np.pi/-2.0, 0.8 , 0, -0.8]
    bot.arm.set_joint_positions(joint_positions2)
    bot.gripper.release()
    joint_positions3 = [np.pi/-2.0, -0.8 , 0, 0]
    bot.arm.set_joint_positions(joint_positions3)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()  
    bot.shutdown()

if __name__=='__main__':
    main()
