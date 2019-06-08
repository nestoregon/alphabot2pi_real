# Alphabot2pi ROS real
This is the code used to run AlphaBot2pi using ROS

## Installation
Create catkin workspace. Make sure you are in a path where you want your worskpace to be and run the following commands on the terminal.
```
mkdir alphabot2pi_real_ws
cd alphabot2pi_real_ws
git clone https://github.com/nestoregon/alphabot2pi_real/
git clone https://github.com/UbiquityRobotics/raspicam_node.git
```
**Change the name to src**
```
mv alphabot2pi_real src 
```
**Create build and devel files**
```
catkin_make
```
**Source the workspace**
```
source devel/setup.bash
```
**Go to the directory with control nodes**
```
cd src/control//src
```
**Make the code executable**
```
chmod  +x control_robot_node.py find_ball_blue_node.py servo_node.py drive_node.py remote_node.py
```
Type "ls" on the terminal to confirm that all the code is green! This means that we can run the code.

## ROS master. Configure /.bashrc files

To be able to connect the RaspberryPi and the Computer the user must configure both /.bashrc files. This file is launched everytime a new terminal window is open.

The following steps setup the Raspberry as the ROS Master and your computer as a connexion to it. More on how to edit /.bashrc files [here](http://answers.ros.org/question/272065/specification-of-ros_master_uri-and-ros_hostname/)

### 1. Raspberry /.bashrc (from Computer)
#### Access Raspberry Remotely

```
ssh ubuntu@ipAddressRaspberry
```
**Open editor for the /.bashrc file**
```
nano ~/.bashrc
```
**Write the following at the bottom of the /.bashrc file**
```
# office
export ROS_MASTER_URI=http://ipAddressRaspberry:11311
export ROS_IP=ipAddressRaspberry
```
**Save changes**
```
Ctrl + x
y
Enter
```
#### Run /.bashrc file again to update changes
```
source ~/.bashrc
```

### 2. Computer /.bashrc (from Computer)

**Open editor for the /.bashrc file in a new terminal window**
```
nano ~/.bashrc
```
**Write the following at the bottom of the /.bashrc file**
```
# office
export ROS_MASTER_URI=http://ipAddressRaspberry:11311
export ROS_IP=ipAddressComputer
```
**Save changes**
```
Ctrl + x
y
Enter
```
#### Run /.bashrc file again to update changes
```
source ~/.bashrc
```
## Run the code

Run the following commands to launch the whole ROS network. As mentioned above, the MASTER of the ROS network is the Raspberry and the Computer as a connexion

Firstly, ssh into the Raspberry and run:
```
roslaunch control computer.launch
```
Secondly, run the following command in your computer
```
roslaunch control raspberry.launch
```

## Instructions
Use the 3 buttons on the remote to select different tasks for the robot
* **Task 1: Manual Mode** Use the keys to manually control the robot
* **Task 2: Ball Following Drive** The robot uses the wheels to follow a blue ball (Hue value of 220)
* **Task 3: Ball Following Servo** The robot uses the servo to follow a blue ball (Hue value of 220)

## Future work
You can add as many input and output nodes as you like.
* Modify the control_robot_node.py code to set new algorithms. Brain of the robot. Set speeds. Set frequency. 
* Modify the remote_node.py node to set new keys and publish new topics
* Modify the find_ball_blue_node.py to set new values for different colors to find

## Authors

* **Nestor Morales** - *Control Code and Gazebo Simulation* - [nestoregon](https://github.com/nestoregon)
* **Manuel Serrano** - *Control Code and Gazebo Simulation*
