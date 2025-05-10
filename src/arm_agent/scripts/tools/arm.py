# import rospy
# import numpy as np
# from langchain.agents import tool
# from std_msgs.msg import Float64, Header
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from std_srvs.srv import Empty
# import subprocess
# import time
# import yaml
# from typing import List, Dict, Optional

# # Global variables to store joint states and publishers
# current_joint_states = {}
# trajectory_publishers = {}
# joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'gripper', 'left_finger', 'right_finger']

# # Initialize publishers
# def init_publishers():
#     global trajectory_publishers
#     # The arm typically uses a joint trajectory controller
#     trajectory_publishers['arm'] = rospy.Publisher(
#         '/px100/arm_controller/joint_trajectory',
#         JointTrajectory,
#         queue_size=10
#     )
#     trajectory_publishers['gripper'] = rospy.Publisher(
#         '/px100/gripper_controller/command',
#         Float64,
#         queue_size=10
#     )

# @tool
# def get_joint_states() -> Dict[str, float]:
#     """
#     Get the current joint states of the PincherX-100 arm.
    
#     Returns the current position of all joints in radians.
#     """
#     try:
#         msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
#         joint_positions = {}
#         for i, name in enumerate(msg.name):
#             if i < len(msg.position):
#                 joint_positions[name] = msg.position[i]
        
#         global current_joint_states
#         current_joint_states = joint_positions
#         return joint_positions
#     except rospy.ROSException as e:
#         return {"error": f"Failed to get joint states: {e}"}

# @tool
# def move_to_home_position() -> str:
#     """
#     Move the arm to its home position.
    
#     Home position is typically all joints at 0 radians.
#     """
#     joint_positions = {
#         'waist': 0.0,
#         'shoulder': 0.0,
#         'elbow': 0.0,
#         'wrist_angle': 0.0,
#         'wrist_rotate': 0.0
#     }
    
#     return execute_joint_trajectory.invoke({
#         "joint_positions": joint_positions,
#         "duration": 3.0
#     })

# @tool
# def move_to_sleep_position() -> str:
#     """
#     Move the arm to its sleep position.
    
#     Sleep position is a compact configuration for storage.
#     """
#     joint_positions = {
#         'waist': 0.0,
#         'shoulder': -1.88,
#         'elbow': 1.5,
#         'wrist_angle': 0.8,
#         'wrist_rotate': 0.0
#     }
    
#     return execute_joint_trajectory.invoke({
#         "joint_positions": joint_positions,
#         "duration": 3.0
#     })

# @tool
# def execute_joint_trajectory(
#     joint_positions: Dict[str, float],
#     duration: float = 2.0
# ) -> str:
#     """
#     Execute a joint trajectory to move the arm to specified positions.
    
#     :param joint_positions: Dictionary of joint names and their target positions in radians
#     :param duration: Time in seconds to complete the movement
#     """
#     global trajectory_publishers
    
#     if 'arm' not in trajectory_publishers:
#         init_publishers()
    
#     try:
#         trajectory = JointTrajectory()
#         trajectory.header = Header()
#         trajectory.header.stamp = rospy.Time.now()
        
#         # Set joint names for the trajectory
#         trajectory.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        
#         # Create trajectory point
#         point = JointTrajectoryPoint()
#         point.positions = [
#             joint_positions.get('waist', 0.0),
#             joint_positions.get('shoulder', 0.0),
#             joint_positions.get('elbow', 0.0),
#             joint_positions.get('wrist_angle', 0.0),
#             joint_positions.get('wrist_rotate', 0.0)
#         ]
#         point.time_from_start = rospy.Duration(duration)
        
#         trajectory.points = [point]
        
#         # Publish the trajectory
#         trajectory_publishers['arm'].publish(trajectory)
        
#         # Wait for movement to complete
#         rospy.sleep(duration)
        
#         # Get final position
#         final_states = get_joint_states.invoke({})
        
#         return f"Moved to positions: {joint_positions}. Final states: {final_states}"
    
#     except Exception as e:
#         return f"Failed to execute trajectory: {e}"

# @tool
# def control_gripper(position: float) -> str:
#     """
#     Control the gripper opening.
    
#     :param position: Gripper position (0.0 = fully closed, 0.037 = fully open)
#     """
#     global trajectory_publishers
    
#     if 'gripper' not in trajectory_publishers:
#         init_publishers()
    
#     try:
#         # Clamp position to valid range
#         position = max(0.0, min(0.037, position))
        
#         msg = Float64()
#         msg.data = position
#         trajectory_publishers['gripper'].publish(msg)
        
#         rospy.sleep(1.0)  # Wait for gripper to move
        
#         return f"Gripper moved to position: {position} ({'closed' if position < 0.01 else 'open'})"
    
#     except Exception as e:
#         return f"Failed to control gripper: {e}"

# @tool
# def open_gripper() -> str:
#     """Open the gripper fully."""
#     return control_gripper.invoke({"position": 0.037})

# @tool
# def close_gripper() -> str:
#     """Close the gripper fully."""
#     return control_gripper.invoke({"position": 0.0})

# @tool
# def move_to_pose(
#     x: float,
#     y: float,
#     z: float,
#     roll: float = 0.0,
#     pitch: float = 0.0,
#     yaw: float = 0.0,
# ) -> str:
#     """
#     Move the arm to a specific 3D pose using inverse kinematics.
    
#     :param x: X position in meters
#     :param y: Y position in meters
#     :param z: Z position in meters
#     :param roll: Roll angle in radians
#     :param pitch: Pitch angle in radians
#     :param yaw: Yaw angle in radians
#     """
#     try:
#         # This is a simplified example - real implementation would use IK service
#         # For now, we'll just move to a predefined position
#         return f"Moving to pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}. Note: IK service integration needed."
    
#     except Exception as e:
#         return f"Failed to move to pose: {e}"

# @tool
# def execute_predefined_motion(motion_name: str) -> str:
#     """
#     Execute a predefined motion sequence.
    
#     :param motion_name: Name of the predefined motion (e.g., "wave", "pick_and_place")
#     """
#     predefined_motions = {
#         "wave": [
#             {"waist": 0.0, "shoulder": -0.5, "elbow": -1.0, "wrist_angle": -0.5, "wrist_rotate": 0.0},
#             {"waist": 0.5, "shoulder": -0.5, "elbow": -1.0, "wrist_angle": -0.5, "wrist_rotate": 0.0},
#             {"waist": -0.5, "shoulder": -0.5, "elbow": -1.0, "wrist_angle": -0.5, "wrist_rotate": 0.0},
#             {"waist": 0.0, "shoulder": -0.5, "elbow": -1.0, "wrist_angle": -0.5, "wrist_rotate": 0.0},
#         ],
#         "nod": [
#             {"waist": 0.0, "shoulder": 0.0, "elbow": 0.0, "wrist_angle": 0.3, "wrist_rotate": 0.0},
#             {"waist": 0.0, "shoulder": 0.0, "elbow": 0.0, "wrist_angle": -0.3, "wrist_rotate": 0.0},
#             {"waist": 0.0, "shoulder": 0.0, "elbow": 0.0, "wrist_angle": 0.0, "wrist_rotate": 0.0},
#         ]
#     }
    
#     if motion_name not in predefined_motions:
#         return f"Unknown motion: {motion_name}. Available motions: {list(predefined_motions.keys())}"
    
#     try:
#         for i, position in enumerate(predefined_motions[motion_name]):
#             result = execute_joint_trajectory.invoke({
#                 "joint_positions": position,
#                 "duration": 1.0
#             })
#             rospy.sleep(0.5)  # Small pause between movements
            
#         return f"Completed motion: {motion_name}"
    
#     except Exception as e:
#         return f"Failed to execute motion {motion_name}: {e}"

# @tool
# def set_torque_enable(enable: bool) -> str:
#     """
#     Enable or disable torque for all joints.
    
#     :param enable: True to enable torque, False to disable
#     """
#     try:
#         service_name = '/px100/torque_enable'
#         rospy.wait_for_service(service_name, timeout=5)
#         torque_service = rospy.ServiceProxy(service_name, Empty)
        
#         if enable:
#             torque_service()
#             return "Torque enabled for all joints"
#         else:
#             # Note: There might be a separate disable service
#             return "Torque disable not implemented - check robot documentation"
    
#     except Exception as e:
#         return f"Failed to set torque: {e}"

# @tool
# def get_workspace_info() -> dict:
#     """
#     Get information about the robot's workspace and limits.
#     """
#     workspace_info = {
#         "robot_model": "PincherX-100",
#         "workspace_radius": "0.35 meters",
#         "max_payload": "0.05 kg",
#         "joint_limits": {
#             "waist": [-3.14, 3.14],
#             "shoulder": [-1.88, 1.99],
#             "elbow": [-2.16, 1.61],
#             "wrist_angle": [-1.75, 2.24],
#             "wrist_rotate": [-3.14, 3.14],
#             "gripper": [0.0, 0.037]
#         },
#         "home_position": {
#             "waist": 0.0,
#             "shoulder": 0.0,
#             "elbow": 0.0,
#             "wrist_angle": 0.0,
#             "wrist_rotate": 0.0
#         }
#     }
    
#     return workspace_info

# @tool
# def record_trajectory(duration: float = 10.0) -> dict:
#     """
#     Record arm movements for a specified duration.
    
#     :param duration: Recording duration in seconds
#     """
#     try:
#         recorded_trajectory = []
#         start_time = rospy.Time.now()
        
#         while (rospy.Time.now() - start_time).to_sec() < duration:
#             current_states = get_joint_states.invoke({})
#             if "error" not in current_states:
#                 recorded_trajectory.append({
#                     "timestamp": (rospy.Time.now() - start_time).to_sec(),
#                     "positions": current_states
#                 })
#             rospy.sleep(0.1)  # Record at 10Hz
        
#         return {
#             "duration": duration,
#             "samples": len(recorded_trajectory),
#             "trajectory": recorded_trajectory[:10]  # Return first 10 samples
#         }
    
#     except Exception as e:
#         return {"error": f"Failed to record trajectory: {e}"}

# @tool
# def emergency_stop() -> str:
#     """
#     Emergency stop - immediately stop all arm movements.
#     """
#     try:
#         # Set current position as target to stop movement
#         current_states = get_joint_states.invoke({})
#         if "error" not in current_states:
#             # Extract only the arm joints
#             arm_joints = {k: v for k, v in current_states.items() 
#                          if k in ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']}
            
#             result = execute_joint_trajectory.invoke({
#                 "joint_positions": arm_joints,
#                 "duration": 0.1  # Very short duration for immediate stop
#             })
            
#             return "Emergency stop executed - arm movement halted"
#         else:
#             return "Failed to get current joint states for emergency stop"
    
#     except Exception as e:
#         return f"Emergency stop failed: {e}"

import rospy
import numpy as np
from langchain.agents import tool
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty
import subprocess
import time
import yaml
from typing import List, Dict, Optional

# Global variables to store joint states and publishers
current_joint_states = {}
trajectory_publishers = {}
joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'gripper', 'left_finger', 'right_finger']

# Initialize publishers - this will be called by the agent
def init_publishers():
    global trajectory_publishers
    # Create publishers for simulation
    trajectory_publishers['arm'] = rospy.Publisher(
        '/px100/arm_controller/joint_trajectory',
        JointTrajectory,
        queue_size=10
    )
    trajectory_publishers['gripper'] = rospy.Publisher(
        '/px100/gripper_controller/command',
        Float64,
        queue_size=10
    )
    
    # Also create a simple joint state publisher for simulation
    trajectory_publishers['joint_state'] = rospy.Publisher(
        '/joint_states',
        JointState,
        queue_size=10
    )
    
    rospy.loginfo("ARM AGENT: Publishers initialized for simulation mode")

# Add a function to simulate joint states
def simulate_joint_states():
    """Publish simulated joint states for visualization"""
    global current_joint_states
    
    if not current_joint_states:
        # Initialize with home position if empty
        current_joint_states = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'wrist_angle': 0.0,
            'wrist_rotate': 0.0,
            'gripper': 0.0,
            'left_finger': 0.0,
            'right_finger': 0.0
        }
    
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = list(current_joint_states.keys())
    msg.position = list(current_joint_states.values())
    msg.velocity = [0.0] * len(msg.name)
    msg.effort = [0.0] * len(msg.name)
    
    if 'joint_state' in trajectory_publishers:
        trajectory_publishers['joint_state'].publish(msg)

@tool
def get_joint_states() -> Dict[str, float]:
    """
    Get the current joint states of the PincherX-100 arm.
    
    Returns the current position of all joints in radians.
    """
    global current_joint_states
    
    try:
        # Try to get real joint states first
        msg = rospy.wait_for_message('/joint_states', JointState, timeout=2)
        joint_positions = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_positions[name] = msg.position[i]
        
        current_joint_states = joint_positions
        return joint_positions
    except rospy.ROSException:
        # If no real joint states, use simulated ones
        if not current_joint_states:
            # Initialize with home position
            current_joint_states = {
                'waist': 0.0,
                'shoulder': 0.0,
                'elbow': 0.0,
                'wrist_angle': 0.0,
                'wrist_rotate': 0.0,
                'gripper': 0.0,
                'left_finger': 0.0,
                'right_finger': 0.0
            }
        
        # Publish simulated states
        simulate_joint_states()
        
        return current_joint_states