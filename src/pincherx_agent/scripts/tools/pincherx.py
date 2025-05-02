from typing import List, Dict, Any

import rospy
from langchain.agents import tool
from interbotix_xs_modules.arm import InterbotixManipulatorXS

arm = None


def initialize_arm():
    """Initialize the robot arm if not already initialized."""
    global arm
    if arm is None:
        try:
            arm = InterbotixManipulatorXS(
                robot_model="px100", group_name="arm", gripper_name="gripper"
            )
            return True
        except Exception as e:
            return f"Failed to initialize robot arm: {e}"
    return True


@tool
def get_joint_positions() -> Dict[str, float]:
    """
    Get the current positions of all robot joints.

    Returns:
        Dict[str, float]: Dictionary of joint names and their current positions in radians.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        positions = arm.arm.get_joint_positions()
        return positions
    except Exception as e:
        return {"error": f"Failed to get joint positions: {e}"}


@tool
def set_joint_positions(
    positions: Dict[str, float], moving_time: float = 2.0, accel_time: float = 0.3
) -> str:
    """
    Set the positions of specific robot joints.

    Args:
        positions (Dict[str, float]): Dictionary mapping joint names to target positions in radians.
        moving_time (float): Time in seconds the movement should take. Default is 2.0.
        accel_time (float): Acceleration time in seconds. Default is 0.3.

    Returns:
        str: Success message or error description.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        arm.arm.set_joint_positions(positions, moving_time, accel_time)
        return f"Successfully set joint positions to {positions}"
    except Exception as e:
        return f"Failed to set joint positions: {e}"


@tool
def go_to_home_pose(moving_time: float = 2.0) -> str:
    """
    Move the robot arm to its home position.

    Args:
        moving_time (float): Time in seconds the movement should take. Default is 2.0.

    Returns:
        str: Success message or error description.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        arm.arm.go_to_home_pose(moving_time)
        return "Successfully moved to home pose"
    except Exception as e:
        return f"Failed to go to home pose: {e}"


@tool
def go_to_sleep_pose(moving_time: float = 2.0) -> str:
    """
    Move the robot arm to its sleep position.

    Args:
        moving_time (float): Time in seconds the movement should take. Default is 2.0.

    Returns:
        str: Success message or error description.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        arm.arm.go_to_sleep_pose(moving_time)
        return "Successfully moved to sleep pose"
    except Exception as e:
        return f"Failed to go to sleep pose: {e}"


@tool
def set_ee_pose(
    x: float,
    y: float,
    z: float,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    moving_time: float = 2.0,
) -> str:
    """
    Set the end-effector pose (position and orientation).

    Args:
        x (float): X position in meters (forward/backward).
        y (float): Y position in meters (left/right).
        z (float): Z position in meters (up/down).
        roll (float): Roll angle in radians. Default is 0.0.
        pitch (float): Pitch angle in radians. Default is 0.0.
        yaw (float): Yaw angle in radians. Default is 0.0.
        moving_time (float): Time in seconds the movement should take. Default is 2.0.

    Returns:
        str: Success message or error description.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        arm.arm.set_ee_pose_components(
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, moving_time=moving_time
        )
        return f"Successfully set end-effector pose to x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}"
    except Exception as e:
        return f"Failed to set end-effector pose: {e}"


@tool
def get_ee_pose() -> Dict[str, float]:
    """
    Get the current end-effector pose.

    Returns:
        Dict[str, float]: Dictionary with x, y, z, roll, pitch, yaw values.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        pose = arm.arm.get_ee_pose()
        x, y, z = pose.position
        roll, pitch, yaw = pose.rpy
        return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}
    except Exception as e:
        return {"error": f"Failed to get end-effector pose: {e}"}


@tool
def gripper_open(moving_time: float = 1.0) -> str:
    """
    Open the robot gripper.

    Args:
        moving_time (float): Time in seconds the movement should take. Default is 1.0.

    Returns:
        str: Success message or error description.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        arm.gripper.open(moving_time)
        return "Successfully opened gripper"
    except Exception as e:
        return f"Failed to open gripper: {e}"


@tool
def gripper_close(moving_time: float = 1.0) -> str:
    """
    Close the robot gripper.

    Args:
        moving_time (float): Time in seconds the movement should take. Default is 1.0.

    Returns:
        str: Success message or error description.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        arm.gripper.close(moving_time)
        return "Successfully closed gripper"
    except Exception as e:
        return f"Failed to close gripper: {e}"


@tool
def gripper_set(position: float, moving_time: float = 1.0) -> str:
    """
    Set the gripper to a specific position.

    Args:
        position (float): Position value between 0 (closed) and 1 (open).
        moving_time (float): Time in seconds the movement should take. Default is 1.0.

    Returns:
        str: Success message or error description.
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    if position < 0 or position > 1:
        return "Error: position must be between 0 (closed) and 1 (open)"

    try:
        arm.gripper.set_pressure(1.0)  # Set full pressure
        if position == 0:
            arm.gripper.close(moving_time)
        elif position == 1:
            arm.gripper.open(moving_time)
        else:
            # Scale position to the right range
            arm.gripper.set(position, moving_time)
        return f"Successfully set gripper to position {position}"
    except Exception as e:
        return f"Failed to set gripper position: {e}"


@tool
def set_trajectory(waypoints: List[Dict[str, Any]], moving_time: float = 2.0) -> str:
    """
    Execute a trajectory through multiple waypoints.

    Args:
        waypoints: List of dictionaries containing either:
                  - joint positions: {"joint_positions": {"joint1": pos1, "joint2": pos2, ...}}
                  - or ee_pose: {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}
        moving_time: Time to take between each waypoint

    Returns:
        str: Success message or error description
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        for waypoint in waypoints:
            if "joint_positions" in waypoint:
                arm.arm.set_joint_positions(waypoint["joint_positions"], moving_time)
            else:
                x = waypoint.get("x", 0)
                y = waypoint.get("y", 0)
                z = waypoint.get("z", 0)
                roll = waypoint.get("roll", 0)
                pitch = waypoint.get("pitch", 0)
                yaw = waypoint.get("yaw", 0)
                arm.arm.set_ee_pose_components(
                    x=x,
                    y=y,
                    z=z,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    moving_time=moving_time,
                )

            # Wait for the movement to complete
            rospy.sleep(moving_time)

        return f"Successfully executed trajectory through {len(waypoints)} waypoints"
    except Exception as e:
        return f"Failed to execute trajectory: {e}"


@tool
def get_workspace_bounds() -> Dict[str, Dict[str, float]]:
    """
    Get the bounds of the robot's workspace.

    Returns:
        Dict: Dictionary containing min and max values for x, y, and z.
    """
    # These are approximate values for the PincherX-100
    return {
        "min": {"x": 0.0, "y": -0.15, "z": 0.0},
        "max": {"x": 0.2, "y": 0.15, "z": 0.25},
    }


@tool
def pick_and_place(
    pick_position: Dict[str, float],
    place_position: Dict[str, float],
    approach_height: float = 0.05,
    moving_time: float = 1.5,
) -> str:
    """
    Perform a pick and place operation.

    Args:
        pick_position: Dictionary with x, y, z coordinates for picking
        place_position: Dictionary with x, y, z coordinates for placing
        approach_height: Height to approach objects from
        moving_time: Time for each movement segment

    Returns:
        str: Success message or error description
    """
    init_result = initialize_arm()
    if init_result is not True:
        return {"error": init_result}

    try:
        # Open gripper
        arm.gripper.open(moving_time)

        # Approach pick position
        pick_x, pick_y, pick_z = (
            pick_position["x"],
            pick_position["y"],
            pick_position["z"],
        )
        arm.arm.set_ee_pose_components(
            x=pick_x, y=pick_y, z=pick_z + approach_height, moving_time=moving_time
        )
        rospy.sleep(moving_time)

        # Move down to pick position
        arm.arm.set_ee_pose_components(
            x=pick_x, y=pick_y, z=pick_z, moving_time=moving_time
        )
        rospy.sleep(moving_time)

        # Close gripper
        arm.gripper.close(moving_time)
        rospy.sleep(moving_time)

        # Lift up
        arm.arm.set_ee_pose_components(
            x=pick_x, y=pick_y, z=pick_z + approach_height, moving_time=moving_time
        )
        rospy.sleep(moving_time)

        # Move to place position approach
        place_x, place_y, place_z = (
            place_position["x"],
            place_position["y"],
            place_position["z"],
        )
        arm.arm.set_ee_pose_components(
            x=place_x, y=place_y, z=place_z + approach_height, moving_time=moving_time
        )
        rospy.sleep(moving_time)

        # Lower to place position
        arm.arm.set_ee_pose_components(
            x=place_x, y=place_y, z=place_z, moving_time=moving_time
        )
        rospy.sleep(moving_time)

        # Open gripper
        arm.gripper.open(moving_time)
        rospy.sleep(moving_time)

        # Lift up
        arm.arm.set_ee_pose_components(
            x=place_x, y=place_y, z=place_z + approach_height, moving_time=moving_time
        )
        rospy.sleep(moving_time)

        return "Successfully completed pick and place operation"
    except Exception as e:
        return f"Failed to complete pick and place operation: {e}"
