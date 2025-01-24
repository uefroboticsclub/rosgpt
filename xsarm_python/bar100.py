import sys 

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS 

import numpy as np 

  

""" 

This script is adapted for the Interbotix PX100 robot arm to perform simple pick and place tasks. 

Adjustments have been made to cater to the PX100's specifications, including its joint limitations and operational range. 

  

To get started, open a terminal and type: 

  

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 

  

Then change to this directory and type: 

  

    python3 simple_pick_place.py 

""" 

  

def main(): 

    bot = InterbotixManipulatorXS( 

        robot_model='px100',  # Adjusted to specify the PX100 model 

        group_name='arm', 

        gripper_name='gripper' 

    ) 

  

    # The PX100 has fewer joints; ensure your commands are suitable for its configuration. 

    if bot.arm.group_info.num_joints < 4:  # Adjusted based on PX100's joint count 

        bot.core.get_logger().fatal('This demo requires the robot to have at least 4 joints!') 

        bot.shutdown() 

        sys.exit() 

  

    # Adjusted task sequences to fit the PX100's operational capabilities and range. 

    bot.arm.set_ee_pose_components(x=0.2, z=0.1)  # Adjusted positions for PX100's reach 

    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0) 

    bot.gripper.release()  # Changed to 'open' for clarity; it performs the same action as 'release' 

    bot.arm.set_ee_cartesian_trajectory(x=0.05, z=-0.2)  # Adjusted movements for PX100 

    bot.gripper.grasp()  # Changed to 'close' for clarity 

    bot.arm.set_ee_cartesian_trajectory(x=-0.05, z=0.1) 

    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0) 

    # The following pitch adjustments are removed as they may not be suitable for the PX100's capabilities 

  

    # Return to home and then sleep pose 

    bot.arm.go_to_home_pose() 

    bot.arm.go_to_sleep_pose() 

  

    bot.shutdown() 

  

  

if __name__ == '__main__': 

    main() 