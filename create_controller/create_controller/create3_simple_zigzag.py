# Authors: Imtiaz Rahman and Liam Maher

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random

def create_pose_stamped_from_file(navigator, x, y):
    """
    Create a PoseStamped message from x and y coordinates.

    Args:
        navigator: Instance of BasicNavigator.
        x: X-coordinate.
        y: Y-coordinate.

    Returns:
        PoseStamped: PoseStamped message with specified coordinates.
    """
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)  # No rotation for now
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def main():
    """
    Main function to follow waypoints from a file.
    """
    # --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose ---
    # !!! Comment if the initial pose is already set !!!
    # initial_pose = create_pose_stamped_from_file(nav, 'initial_pose.txt')
    # nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 ---
    nav.waitUntilNav2Active()

    # --- Create waypoints from file ---
    waypoints_file = '' # add path to the .txt file containing all the coordinates here
    waypoints = []
    with open(waypoints_file, 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split())
            waypoints.append(create_pose_stamped_from_file(nav, x, y))

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
