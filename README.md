# **Create3 Zigzag Scanline Algorithm** 

### Firstly, some tools need to be downloaded: 

Follow the instructions in the link to install **ROS2 Humble**:
https://docs.ros.org/en/humble/Installation.html 

Follow the instructions in the link to install **NAV2**:
https://navigation.ros.org/getting_started/index.html 

Then install **Turtlebot3** with the following command:
```
sudo apt install ros-humble-turtlebot3* 
```
### With all these packages installed, clone this repository, follow the instructions provided in the link below: 
**Create3_examples github repository**: https://github.com/iRobotEducation/create3_examples/tree/humble 

### Verify lidar connection
Now make sure your lidar is plugged in and able to transmit messages. You can verify this by checking the topic list in **ROS2** with the following command:
```
ros2 topic list
```
There should be a topic called **/scan**. If you can't find but your lidar is running this might be a permission issue. Check the serial port of your lidar with this command:
```
ls -l /dev|grep ttyUSB
```
By default it should be **ttyUSB0**. Now provide read/write permission to this serial port:
```
sudo chmod 666 /dev/ttyUSB0
```
### Creating the map
Next, we need to create a map of the area to be covered. To do this we need to run the following commands: 
```
ros2 launch create3_lidar_slam sensors_launch.py
```
then in a separate terminal: 
```
ros2 launch create3_lidar_slam slam_toolbox_launch.py
```
there may be some warnings and errors upon startup, but you should eventually see the message: “[async_slam_toolbox_node-1] Registering sensor: [Custom Described Lidar]”
next, in a third terminal, run: 
```
ros2 launch turtlebot3_cartographer cartographer.launch.py 
```
This will cause turtlebot3’s cartographer to launch. This is the software used to map the area using the LiDAR data. 
With everything still running, in a fourth terminal run: 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Drive the robot around until the map is satisfactory.

### Saving the map
Once you are satisfied with the map, you’ll need to save the map. To do this, with everything still running, run the following commands:
```
mkdir <maps_directory> 
ros2 run nav2_map_server map_saver_cli -f <maps_directory>/<name_of_map> 
```
For example, if you wanted to store a map named, “my_map” in a folder directory named, “myMaps,” the command to be run would be: 
```
mkdir myMaps
ros2 run nav2_map_server map_saver_cli -f myMaps/my_map
```
>[!NOTE]
>You may need to run this command multiple times until the map is successfully saved. 

### Launching the map
Now that the map was successfully saved, you can now start up nav2 and navigate within the area that you mapped. First, use ctrl + c to kill the process in the terminal for cartographer and teleop_twist. Then run the following command to run nav2. 
To get nav2 running with the desired map, run:
```
ros2 launch turtbot3_navigation2 navigation2.launch.py use_sim_time:=False map:=<map_directory>/<desired_map_name>.yaml
```
>[!IMPORTANT]
>You should append “.yaml” to the end of the name of the desired map. The yaml file contains the parameters for the map. 

### Creating waypoints for the robot to follow
Now we need to find the proper waypoints to get the robot to move in the desired zigzag pattern.  
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/3659c54f-aeb8-41d1-8b07-5dbf0355cf06>
</p>
You will need to move the robot to the proper locations. Move the robot to position 1, and then in a new terminal run: 

```
ros2 topic echo /odom | grep -A 2 'position:'
```
This will print the robot's current position in the terminal. The robot's position will keep printing as long as the position is being changed. When it's idle nothing will output but the positions already printed will still be there.

**This is what you should see when you run the above command:**

<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/82364307-28a9-4956-bd8f-3ac8251aa6d1>
</p>

We want to take note of the x and y positions. 
Write these down and make sure to take note of which position this is. 
If this is position 1, I might write down: “-2.7871, -1.2062”
Make sure not to make any of the waypoints too close to the wall. Additionally, make sure to keep the robot out of the cyan area on the map. 

**This is an example of poor robot placement:**
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/b083d2ce-b40f-4768-88b4-5e638b3993ec>
</p>
The robot here is too far into the cyan area, this will cause nav2 to have a hard time making the robot navigate to that point. 
<br><br>

**Here is an example of good robot placement:** 
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/098e4b45-1459-4e31-989f-75290e6568ff>
</p>

This placement is much better. The robot is just barely touching the cyan area and majority of the robot is in the purple area. 
The farther away from the cyan, the better. 

### Cloning the repository and building the package:

Once the waypoints are all taken note of, clone this repository into a workspace if you haven't already.
you can edit the script named, “create3_simple_zigzag”
Go to the **src** directory in **your_ws** and launch the folder in VS code: 
```
cd your_ws/src
code . 
```
this will pull VS Code. You may need to download VS Code. 
Once in VS Code, use the bar on the left to navigate to the create3_simple_zigzag file. 
But at first you need to create a .txt file with all the coordinates. You have to seperate x and y coordinate with a space in between and make sure they are of data type float/double.
So, now you will copy the path to your file with the coordinates and in the create3_simple_zigzag file you will add that path to the waypoints_file variable.

**A complete set of coordinates may look like:** 
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/4e2e8728-2198-41fd-a447-1cb368cdbccd>
</p>

The final waypoint will make the robot return to the origin of the map. 
>[!IMPORTANT]
>make sure to use 0.0 instead of just 0 for the x and y-coordinates. 

###
Once the those are taken care of go back to the root directory which is **your_ws**: 
```
cd ..
```
Then build and source the package:
```
colcon build --symlink-install
source ~/your_ws/install/setup.bash
```
Now to run the script, keep the **sensors**, the **slam_toolbox**, and **nav2** running. 
With all these processes running, run the script: 
```
ros2 run create_controller create3_simple_zigzag
```
the robot will now follow the waypoints that you put into the script. 

## Gazebo Instruction
Alternatively, if you are using gazebo, the steps are similar.
At first you have to set up gazebo. Follow the instructions in this link:
https://classic.gazebosim.org/tutorials?tut=install_ubuntu
Once you are done installing and setting up gazebo, you can go directly to mapping using cartographer, as you don't need to launch the sensors and other tools, its already integrated into gazebo:
```
ros2 launch turtlebot3_cartographer cartographer.launch.py 
```
This will cause turtlebot3’s cartographer to launch. This is the software used to map the area using the LiDAR data. 
With everything still running, in another terminal run: 
```
ros2 run turtlebot3_teleop teleop_keyboard
```
Drive the robot around until the map is satisfactory.

### Saving the map
Once you are satisfied with the map, you’ll need to save the map. To do this, with everything still running, run the following commands:
```
mkdir <maps_directory> 
ros2 run nav2_map_server map_saver_cli -f <maps_directory>/<name_of_map> 
```
For example, if you wanted to store a map named, “my_map” in a folder directory named, “myMaps,” the command to be run would be: 
```
mkdir myMaps
ros2 run nav2_map_server map_saver_cli -f myMaps/my_map
```
>[!NOTE]
>You may need to run this command multiple times until the map is successfully saved. 

### Launching the map
Now that the map was successfully saved, you can now start up nav2 and navigate within the area that you mapped. First, use ctrl + c to kill the process in the terminal for cartographer and teleop_twist. Then run the following command to run nav2. 
To get nav2 running with the desired map, run:
```
ros2 launch turtbot3_navigation2 navigation2.launch.py use_sim_time:=False map:=<map_directory>/<desired_map_name>.yaml
```
>[!IMPORTANT]
>You should append “.yaml” to the end of the name of the desired map. The yaml file contains the parameters for the map. 

### Creating waypoints for the robot to follow
Now we need to find the proper waypoints to get the robot to move in the desired zigzag pattern.  
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/3659c54f-aeb8-41d1-8b07-5dbf0355cf06>
</p>
You will need to move the robot to the proper locations. Move the robot to position 1, and then in a new terminal run: 

```
ros2 topic echo /amcl_pose | grep -A 2 'position:'
```
This will print the robot's current position in the terminal. The robot's position will keep printing as long as the position is being changed. When it's idle nothing will output but the positions already printed will still be there.

**This is what you should see when you run the above command:**

<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/82364307-28a9-4956-bd8f-3ac8251aa6d1>
</p>

We want to take note of the x and y positions. 
Write these down and make sure to take note of which position this is. 
If this is position 1, I might write down: “-2.7871, -1.2062”
Make sure not to make any of the waypoints too close to the wall. Additionally, make sure to keep the robot out of the cyan area on the map. 

**This is an example of poor robot placement:**
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/b083d2ce-b40f-4768-88b4-5e638b3993ec>
</p>
The robot here is too far into the cyan area, this will cause nav2 to have a hard time making the robot navigate to that point. 
<br><br>

**Here is an example of good robot placement:** 
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/098e4b45-1459-4e31-989f-75290e6568ff>
</p>

This placement is much better. The robot is just barely touching the cyan area and majority of the robot is in the purple area. 
The farther away from the cyan, the better. 

### Cloning the repository and building the package:

Once the waypoints are all taken note of, clone this repository into a workspace if you haven't already.
you can edit the script named, “create3_simple_zigzag”
Go to the **src** directory in **your_ws** and launch the folder in VS code: 
```
cd your_ws/src
code . 
```
this will pull VS Code. You may need to download VS Code. 
Once in VS Code, use the bar on the left to navigate to the create3_simple_zigzag file. 
But at first you need to create a .txt file with all the coordinates. You have to seperate x and y coordinate with a space in between and make sure they are of data type float/double.
So, now you will copy the path to your file with the coordinates and in the create3_simple_zigzag file you will add that path to the waypoints_file variable.

**A complete set of coordinates may look like:** 
<p align="center">
 <img src = https://github.com/vinzmoke-zoro/Zigzag-Scanline-Algorithm/assets/63388102/4e2e8728-2198-41fd-a447-1cb368cdbccd>
</p>

The final waypoint will make the robot return to the origin of the map. 
>[!IMPORTANT]
>make sure to use 0.0 instead of just 0 for the x and y-coordinates. 

###
Once the those are taken care of go back to the root directory which is **your_ws**: 
```
cd ..
```
Then build and source the package:
```
colcon build --symlink-install
source ~/your_ws/install/setup.bash
```
Now to run the script, keep the **sensors**, the **slam_toolbox**, and **nav2** running. 
With all these processes running, run the script: 
```
ros2 run create_controller create3_simple_zigzag
```
the robot will now follow the waypoints that you put into the script. 
