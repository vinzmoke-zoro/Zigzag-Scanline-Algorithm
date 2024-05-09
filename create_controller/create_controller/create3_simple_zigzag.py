#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
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
    # num_waypoints = 5
    waypoints = []
    # for _ in range(num_waypoints):
    #     waypoint = generate_random_waypoint()
    #     waypoints.append(create_pose_stamped(nav, *waypoint))
    # waypoints.append(create_pose_stamped(nav, 0, 0, 0)) 

    # for squareKitchen.yaml
    # waypoints.append(create_pose_stamped(nav, -0.651, -1.875, 0)) # 1 
    # waypoints.append(create_pose_stamped(nav, -1.145, -1.875, 0))
    # waypoints.append(create_pose_stamped(nav, -0.651, -1.225, 0))
    # waypoints.append(create_pose_stamped(nav, -0.651, -0.832, 0))
    # waypoints.append(create_pose_stamped(nav, -1.692, -1.908, 0)) # 5
    # waypoints.append(create_pose_stamped(nav, -2.139, -1.875, 0))
    # waypoints.append(create_pose_stamped(nav, -0.615, -0.237, 0))
    # waypoints.append(create_pose_stamped(nav, -1.002, -0.174, 0))
    # waypoints.append(create_pose_stamped(nav, -2.006, -1.303, 0))
    # waypoints.append(create_pose_stamped(nav, -2.087, -0.947, 0)) # 10
    # waypoints.append(create_pose_stamped(nav, -1.476, -0.233, 0))
    # waypoints.append(create_pose_stamped(nav, -2.045, -0.176, 0)) # 12

    #for reg_kitchen.yaml
    # waypoints.append(create_pose_stamped(nav, -0.572, -2.063, 0)) # 1 
    # waypoints.append(create_pose_stamped(nav, -0.755, -2.075, 0)) 
    # waypoints.append(create_pose_stamped(nav, -0.546, -1.601, 0)) 
    # waypoints.append(create_pose_stamped(nav, -0.552, -1.329, 0)) 
    # waypoints.append(create_pose_stamped(nav, -1.245, -2.078, 0)) # 5

    # waypoints.append(create_pose_stamped(nav, -1.634, -2.145, 0))  
    # waypoints.append(create_pose_stamped(nav, -0.533, -0.986, 0)) 
    # waypoints.append(create_pose_stamped(nav, -0.499, -0.711, 0)) 
    # waypoints.append(create_pose_stamped(nav, -1.840, -2.032, 0)) 
    # waypoints.append(create_pose_stamped(nav, -2.254, -1.888, 0)) # 10

    # waypoints.append(create_pose_stamped(nav, -0.756, -0.779, 0)) 
    # waypoints.append(create_pose_stamped(nav, -1.126, -0.748, 0)) 
    # waypoints.append(create_pose_stamped(nav, -2.531, -1.774, 0)) 
    # waypoints.append(create_pose_stamped(nav, -2.335, -1.498, 0)) 
    # waypoints.append(create_pose_stamped(nav, -1.398, -0.746, 0)) # 15

    # waypoints.append(create_pose_stamped(nav, -1.691, -0.638, 0))  
    # waypoints.append(create_pose_stamped(nav, -2.152, -1.070, 0)) 
    # waypoints.append(create_pose_stamped(nav, -2.003, -0.833, 0)) # 18 
    # waypoints.append(create_pose_stamped(nav, 0.0, 0.0, 0.0)) # final waypoint-- return to (0, 0, 0) 

    # for map55.yaml

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


    