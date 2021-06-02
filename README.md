# Alphabot2pi ROS real
This is the code used to run AlphaBot2pi using ROS. 2 devices will be used: the raspberryPi inside alphabot2pi AND a computer running Ubuntu. Keep in mind that ROS has to be installed in both devices. The raspberry image used in this project can be found [here](https://downloads.ubiquityrobotics.com/pi.html) with ROS already installed in it.

## Installation

In this section, all the necessary steps to install the code are explained.

**NOTE: The installation process has to be carried out in both the Raspberry and the Computer**

1. Create catkin workspace. Make sure you are in a path where you want your workspace to be and run the following commands on the terminal.
```
mkdir alphabot2pi_real_ws
cd alphabot2pi_real_ws
git clone https://github.com/nestoregon/alphabot2pi_real/
```
2. Change the name to src. The file downloaded from GitHub has to change its name to src.
```
mv alphabot2pi_real src
```
3. Import raspicam_node to be able to use the camera. This package enables you to publish a stream video into a topic. Only used by the Raspberry camera.
```
cd src; git clone https://github.com/UbiquityRobotics/raspicam_node.git; cd ..
```
4. Create build and devel files. The following command creates all the necessary build and devel files. This is why you only need to download the src, the other files are automatically generated after the following command
```
catkin_make
```
5. Source the workspace. To be able to use ```roslaunch``` ```roscd``` or ```rosrun``` the workspace must be sourced. Note: each new terminal window will not be sourced unless you edit the ~/.bashrc file, further details below.
```
source devel/setup.bash
```
6. Go to the directory with control nodes
```
cd src/control//src
```
7. Make the code executable. The code will not work unless it is executable.
```
chmod  +x control_robot_node.py find_ball_blue_node.py servo_node.py drive_node.py remote_node.py
```
Type "ls" on the terminal to confirm that all the code is green! This means that we can run the code. You should see something like this:
```diff
+ $ control_robot_node.py find_ball_blue_node.py servo_node.py drive_node.py remote_node.py
```

## Configure /.bashrc files. ROS_MASTER and ROS_URI

To be able to connect the RaspberryPi and the Computer the user must configure both /.bashrc files. This file is launched every time a new terminal window is open.

The following steps set up the Raspberry as the ROS Master and your computer as a connexion to it. More on how to edit /.bashrc files [here](http://answers.ros.org/question/272065/specification-of-ros_master_uri-and-ros_hostname/)

You will need to know the IP address of your device (Raspberry & computer) in order to proceed. There are 2 ways of doing this:
* Open the terminal and write ```ip address``` on the Linux device.
* Use a third party app such as [Fing](https://www.fing.com/) on your mobile device to search for it.

### 1. Raspberry /.bashrc (from Computer)
1. Access Raspberry Remotely. Type your raspberry IP address instead.

```
ssh ubuntu@ipAddressRaspberry
```
2. Open editor for the /.bashrc file
```
nano ~/.bashrc
```
3. Write the following at the bottom of the /.bashrc file. The first line is to source the workspace (**NOTE: change the path for your own workspace path!**) and the second to establish the ROS_MASTER and ROS_IP. The Raspberry is the Master in this example. The ROS_IP is the raspberry one because we are working on the raspberry.
```
# Identify the workspace every time the terminal is opened
source /home/nestoregon/ROS/alphabot2pi_real_ws/devel/setup.bash
# office
export ROS_MASTER_URI=http://ipAddressRaspberry:11311
export ROS_IP=ipAddressRaspberry
```
4. Save changes
```
Ctrl + x
y
Enter
```
5. Run /.bashrc file again to update changes
```
source ~/.bashrc
```

### 2. Computer /.bashrc (from Computer)

1. Open editor for the /.bashrc file in a new terminal window
```
nano ~/.bashrc
```
2. Write the following at the bottom of the /.bashrc file. The first line is to source the workspace (**NOTE: change the path for your own workspace path!**) and the second to establish the ROS_MASTER and ROS_IP. The Raspberry is the Master in this example. The ROS_IP is the computer one because we are working on the computer.
```
# Identify the workspace every time the terminal is opened
source /home/nestoregon/ROS/alphabot2pi_real_ws/devel/setup.bash
# office
export ROS_MASTER_URI=http://ipAddressRaspberry:11311
export ROS_IP=ipAddressComputer
```
3. Save changes
```
Ctrl + x
y
Enter
```
4. Run /.bashrc file again to update changes
```
source ~/.bashrc
```
## Run the code

Run the following commands to launch the whole ROS network. As mentioned above, the MASTER of the ROS network is the Raspberry and the Computer as a connexion

Firstly, ssh into the Raspberry and run:
```
roslaunch control raspberry.launch
```
There are 2 ways of running the code in the computer. The computer is in charge of only 2 nodes: control_robot_node.py and find_ball_blue_node.py.
* Option 1: both nodes are launched at the same time, no results nor images can be seen
```
roslaunch control computer.launch
```
* Option 2: each node is launched independently in a different terminal window. This allows the user to see the internal processes of each node. Run both commands in two separe terminal windows.
When the find_ball_blue_node.py is launched independently, the user is able to see the images and masks.
```
rosrun control find_ball_blue_node.py
```
When the control_robot_node.py is launched independently, the user is able to see the direction of the robot and each mode selected
```
rosrun control control_robot_node.py
```
## Instructions
Use the 3 buttons on the remote to select different tasks for the robot.
* **Task 1: Manual Mode** Use the keys to manually control the robot
* **Task 2: Ball Following Drive** The robot uses the wheels to follow the blue ball
* **Task 3: Ball Following Servo** The robot uses the servo to follow the blue ball
In Modes 2 & 3 the robot follows a blue ball. The find_ball_blue_node.py uses the HSV color system.

![#365FB3](https://placehold.it/15/365FB3/000000?text=+) `HSV: 220, 70%, 70%`, `HEX: 365FB3`, `RGB: 54, 95, 179`

## Future work
You can add as many input and output nodes as you like.
* Modify the control_robot_node.py code to set new algorithms. The brain of the robot. Set speeds. Set frequency.
* Modify the remote_node.py node to set new keys and publish new topics
* Modify the find_ball_blue_node.py to set new values for different colors to find

## Authors

* **Nestor Morales** - *Control Code and Gazebo Simulation* - [nestoregon](https://github.com/nestoregon)
