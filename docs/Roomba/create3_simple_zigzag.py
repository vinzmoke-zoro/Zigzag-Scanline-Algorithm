#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    """
    Create a PoseStamped message with given position and orientation.

    Args:
        navigator: An instance of the BasicNavigator class.
        position_x: The x-coordinate of the position.
        position_y: The y-coordinate of the position.
        rotation_z: The rotation around the z-axis in radians.

    Returns:
        A PoseStamped message.
    """
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def generate_random_waypoint():
    """
    Generate random waypoint coordinates and orientation.

    Returns:
        Random x-coordinate, y-coordinate, and rotation around the z-axis.
    """
    # Generate random x and y coordinates within a range
    position_x = random.uniform(-1.9, 1.9)
    position_y = random.uniform(-1.9, 1.9)
    # Generate a random orientation
    rotation_z = random.uniform(0, 1.5 * 3.14159)
    return position_x, position_y, rotation_z

def main():
    # --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose ---
    # !!! Comment if the initial pose is already set !!!
    # initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    # nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 ---
    nav.waitUntilNav2Active()

    # --- Create waypoints ---
    waypoints = []

    waypoints.append(create_pose_stamped(nav, -1.0, -1.75, 0))
    waypoints.append(create_pose_stamped(nav, -1.7084, -1.5931, 0))
    waypoints.append(create_pose_stamped(nav, -1.1039, -0.3426, 0))
    waypoints.append(create_pose_stamped(nav, -1.8759, -0.3938, 0))
    waypoints.append(create_pose_stamped(nav, -2.3081, -1.4054, 0))
    waypoints.append(create_pose_stamped(nav, -2.7871, -1.2062, 0))
    waypoints.append(create_pose_stamped(nav, -2.4679, -0.5127, 0))
    waypoints.append(create_pose_stamped(nav, 0.0, 0.0, 0))

    # Follow waypoints
    nav.followWaypoints(waypoints)

    # Wait until task is complete
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()

    # --- Get the result ---
    print(nav.getResult())

    # --- Shutdown ROS2 communications ---
    rclpy.shutdown()

if __name__ == '__main__':
    main()
