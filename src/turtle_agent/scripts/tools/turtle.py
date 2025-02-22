from math import cos, sin, sqrt
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from langchain.agents import tool
from std_srvs.srv import Empty as EmptySrv
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, TeleportRelative, Kill, SetPen

cmd_vel_pubs = {}


class TurtleNode(Node):
    def __init__(self):
        super().__init__("turtle_node")
        self.cmd_vel_pubs = {"turtle1": self.create_publisher(Twist, "/turtle1/cmd_vel", 10)}

    def add_cmd_vel_pub(self, name: str) -> None:
        """Add a publisher for a turtle's cmd_vel topic."""
        self.cmd_vel_pubs[name] = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)

    def remove_cmd_vel_pub(self, name: str) -> None:
        """Remove a publisher for a turtle's cmd_vel topic."""
        self.cmd_vel_pubs.pop(name, None)

    def get_publisher(self, name: str) -> Optional[rclpy.publisher.Publisher]:
        """Get the publisher for a specific turtle."""
        return self.cmd_vel_pubs.get(name)


turtle_node = TurtleNode()


def within_bounds(x: float, y: float) -> tuple:
    """
    Check if the given x, y coordinates are within the bounds of the turtlesim environment.

    :param x: The x-coordinate.
    :param y: The y-coordinate.
    """
    if 0 <= x <= 11 and 0 <= y <= 11:
        return True, "Coordinates are within bounds."
    else:
        return False, f"({x}, {y}) will be out of bounds. Range is [0, 11] for each."


def will_be_within_bounds(
    name: str, velocity: float, lateral: float, angle: float, duration: float = 1.0
) -> tuple:
    """Check if the turtle will be within bounds after publishing a twist command."""
    pose = get_turtle_pose({"names": [name]})
    if "error" in pose:
        return False, pose["error"]
    current_x = pose[name].x
    current_y = pose[name].y
    current_theta = pose[name].theta

    if abs(angle) < 1e-6:  
        new_x = (
            current_x
            + (velocity * cos(current_theta) - lateral * sin(current_theta)) * duration
        )
        new_y = (
            current_y
            + (velocity * sin(current_theta) + lateral * cos(current_theta)) * duration
        )
    else:  
        radius = sqrt(velocity**2 + lateral**2) / abs(angle)
        center_x = current_x - radius * sin(current_theta)
        center_y = current_y + radius * cos(current_theta)
        angle_traveled = angle * duration
        new_x = center_x + radius * sin(current_theta + angle_traveled)
        new_y = center_y - radius * cos(current_theta + angle_traveled)

        for t in range(int(duration) + 1):
            angle_t = current_theta + angle * t
            x_t = center_x + radius * sin(angle_t)
            y_t = center_y - radius * cos(angle_t)
            in_bounds, _ = within_bounds(x_t, y_t)
            if not in_bounds:
                return (
                    False,
                    f"The circular path will go out of bounds at ({x_t:.2f}, {y_t:.2f}).",
                )

    in_bounds, message = within_bounds(new_x, new_y)
    if not in_bounds:
        return (
            False,
            f"This command will move the turtle out of bounds to ({new_x:.2f}, {new_y:.2f}).",
        )

    return True, f"The turtle will remain within bounds at ({new_x:.2f}, {new_y:.2f})."


@tool
def spawn_turtle(name: str, x: float, y: float, theta: float) -> str:
    """
    Spawn a turtle at the given x, y, and theta coordinates.

    :param name: name of the turtle.
    :param x: x-coordinate.
    :param y: y-coordinate.
    :param theta: angle.
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    name = name.replace("/", "")

    client = turtle_node.create_client(Spawn, "/spawn")
    while not client.wait_for_service(timeout_sec=5.0):
        if not rclpy.ok():
            return f"Failed to spawn {name}: /spawn service not available."
    
    request = Spawn.Request()
    request.x = x
    request.y = y
    request.theta = theta
    request.name = name

    future = client.call_async(request)
    rclpy.spin_until_future_complete(turtle_node, future)
    if future.result() is not None:
        turtle_node.add_cmd_vel_pub(name)
        return f"{name} spawned at x: {x}, y: {y}, theta: {theta}."
    else:
        return f"Failed to spawn {name}: {future.exception().message}"


@tool
def kill_turtle(names: List[str]) -> str:
    """
    Removes a turtle from the turtlesim environment.

    :param names: List of names of the turtles to remove (do not include the forward slash).
    """
    names = [name.replace("/", "") for name in names]
    response = ""

    for name in names:
        client = turtle_node.create_client(Kill, f"/{name}/kill")
        while not client.wait_for_service(timeout_sec=5.0):
            if not rclpy.ok():
                response += f"Failed to kill {name}: /{name}/kill service not available.\n"
                continue

        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(turtle_node, future)
        if future.result() is not None:
            turtle_node.remove_cmd_vel_pub(name)
            response += f"Successfully killed {name}.\n"
        else:
            response += f"Failed to kill {name}: {future.exception().message}\n"

    return response


@tool
def clear_turtlesim() -> str:
    """Clears the turtlesim background and sets the color to the value of the background parameters."""
    client = turtle_node.create_client(EmptySrv, "/clear")
    while not client.wait_for_service(timeout_sec=5.0):
        if not rclpy.ok():
            return "Failed to clear the turtlesim background: /clear service not available."
    
    request = EmptySrv.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(turtle_node, future)
    if future.result() is not None:
        return "Successfully cleared the turtlesim background."
    else:
        return f"Failed to clear the turtlesim background: {future.exception().message}"


@tool
def get_turtle_pose(names: List[str]) -> dict:
    """
    Get the pose of one or more turtles.

    :param names: List of names of the turtles to get the pose of.
    """
    names = [name.replace("/", "") for name in names]
    poses = {}

    for name in names:
        sub = turtle_node.create_subscription(Pose, f"/{name}/pose", lambda msg: poses.update({name: msg}), 10)
        rclpy.spin_once(turtle_node, timeout_sec=5.0)
        if name not in poses:
            return {"error": f"Failed to get pose for {name}: /{name}/pose not available."}
    
    return poses


@tool
def teleport_absolute(name: str, x: float, y: float, theta: float, hide_pen: bool = True) -> str:
    """
    Teleport a turtle to the given x, y, and theta coordinates.

    :param name: name of the turtle
    :param x: The x-coordinate, range: [0, 11]
    :param y: The y-coordinate, range: [0, 11]
    :param theta: angle
    :param hide_pen: True to hide the pen (do not show movement trace on screen), False to show the pen
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    client = turtle_node.create_client(TeleportAbsolute, f"/{name}/teleport_absolute")
    while not client.wait_for_service(timeout_sec=5.0):
        if not rclpy.ok():
            return f"Failed to teleport {name}: /{name}/teleport_absolute service not available."

    request = TeleportAbsolute.Request()
    request.x = x
    request.y = y
    request.theta = theta

    if hide_pen:
        set_pen({"name": name, "r": 0, "g": 0, "b": 0, "width": 1, "off": 1})
    future = client.call_async(request)
    rclpy.spin_until_future_complete(turtle_node, future)
    if future.result() is not None:
        if hide_pen:
            set_pen({"name": name, "r": 30, "g": 30, "b": 255, "width": 1, "off": 0})
        current_pose = get_turtle_pose({"names": [name]})
        return f"{name} new pose: ({current_pose[name].x}, {current_pose[name].y}) at {current_pose[name].theta} radians."
    else:
        return f"Failed to teleport {name}: {future.exception().message}"


@tool
def teleport_relative(name: str, linear: float, angular: float) -> str:
    """
    Teleport a turtle relative to its current position.

    :param name: name of the turtle
    :param linear: linear distance
    :param angular: angular distance
    """
    in_bounds, message = will_be_within_bounds(name, linear, 0.0, angular)
    if not in_bounds:
        return message

    client = turtle_node.create_client(TeleportRelative, f"/{name}/teleport_relative")
    while not client.wait_for_service(timeout_sec=5.0):
        if not rclpy.ok():
            return f"Failed to teleport {name}: /{name}/teleport_relative service not available."

    request = TeleportRelative.Request()
    request.linear = linear
    request.angular = angular

    future = client.call_async(request)
    rclpy.spin_until_future_complete(turtle_node, future)
    if future.result() is not None:
        current_pose = get_turtle_pose({"names": [name]})
        return f"{name} new pose: ({current_pose[name].x}, {current_pose[name].y}) at {current_pose[name].theta} radians."
    else:
        return f"Failed to teleport {name}: {future.exception().message}"


@tool
def publish_twist_to_cmd_vel(name: str, velocity: float, lateral: float, angle: float, steps: int = 1) -> str:
    """
    Publish a Twist message to the /{name}/cmd_vel topic to move a turtle robot.
    Use a combination of linear and angular velocities to move the turtle in the desired direction.

    :param name: name of the turtle (do not include the forward slash)
    :param velocity: linear velocity, where positive is forward and negative is backward
    :param lateral: lateral velocity, where positive is left and negative is right
    :param angle: angular velocity, where positive is counterclockwise and negative is clockwise
    :param steps: Number of times to publish the twist message
    """
    name = name.replace("/", "")

    in_bounds, message = will_be_within_bounds(name, velocity, lateral, angle, duration=steps)
    if not in_bounds:
        return message

    vel = Twist()
    vel.linear.x, vel.linear.y, vel.linear.z = velocity, lateral, 0.0
    vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, angle

    pub = turtle_node.get_publisher(name)
    if not pub:
        return f"No publisher found for turtle '{name}'."

    for _ in range(steps):
        pub.publish(vel)
        rclpy.spin_once(turtle_node, timeout_sec=1.0)

    current_pose = get_turtle_pose({"names": [name]})
    return (
        f"New Pose ({name}): x={current_pose[name].x}, y={current_pose[name].y}, "
        f"theta={current_pose[name].theta} rads, "
        f"linear_velocity={current_pose[name].linear_velocity}, "
        f"angular_velocity={current_pose[name].angular_velocity}."
    )


@tool
def stop_turtle(name: str) -> str:
    """
    Stop a turtle by publishing a Twist message with zero linear and angular velocities.

    :param name: name of the turtle
    """
    return publish_twist_to_cmd_vel(
        name=name,
        velocity=0.0,
        lateral=0.0,
        angle=0.0,
    )


@tool
def reset_turtlesim() -> str:
    """
    Resets the turtlesim, removes all turtles, clears any markings, and creates a new default turtle at the center.
    """
    client = turtle_node.create_client(EmptySrv, "/reset")
    while not client.wait_for_service(timeout_sec=5.0):
        if not rclpy.ok():
            return "Failed to reset the turtlesim environment: /reset service not available."

    request = EmptySrv.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(turtle_node, future)
    if future.result() is not None:
        turtle_node.cmd_vel_pubs.clear()
        turtle_node.add_cmd_vel_pub("turtle1")
        return "Successfully reset the turtlesim environment. Ignore all previous commands, failures, and goals."
    else:
        return f"Failed to reset the turtlesim environment: {future.exception().message}"


@tool
def set_pen(name: str, r: int, g: int, b: int, width: int, off: int) -> str:
    """
    Set the pen color and width for the turtle. The pen is used to draw lines on the turtlesim canvas.

    :param name: name of the turtle
    :param r: red value
    :param g: green value
    :param b: blue value
    :param width: width of the pen.
    :param off: 0=on, 1=off
    """
    name = name.replace("/", "")

    client = turtle_node.create_client(SetPen, f"/{name}/set_pen")
    while not client.wait_for_service(timeout_sec=5.0):
        if not rclpy.ok():
            return f"Failed to set the pen color for {name}: /{name}/set_pen service not available."

    request = SetPen.Request()
    request.r = r
    request.g = g
    request.b = b
    request.width = width
    request.off = off

    future = client.call_async(request)
    rclpy.spin_until_future_complete(turtle_node, future)
    if future.result() is not None:
        return f"Successfully set the pen color for {name}."
    else:
        return f"Failed to set the pen color for {name}: {future.exception().message}"


@tool
def has_moved_to_expected_coordinates(name: str, expected_x: float, expected_y: float, tolerance: float = 0.1) -> str:
    """
    Check if the turtle has moved to the expected position.

    :param name: name of the turtle
    :param expected_x: expected x-coordinate
    :param expected_y: expected y-coordinate
    :param tolerance: tolerance level for the comparison
    """
    current_pose = get_turtle_pose({"names": [name]})
    if "error" in current_pose:
        return current_pose["error"]
    current_x = current_pose[name].x
    current_y = current_pose[name].y

    distance = ((current_x - expected_x) ** 2 + (current_y - expected_y) ** 2) ** 0.5
    if distance <= tolerance:
        return f"{name} has moved to the expected position ({expected_x}, {expected_y})."
    else:
        return f"{name} has NOT moved to the expected position ({expected_x}, {expected_y})."


def main(args=None):
    rclpy.init(args=args)
    node = TurtleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()