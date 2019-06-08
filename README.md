# Alphabot2pi ROS real
This is the code used to run AlphaBot2pi using ROS. 2 devices will be used: the raspberryPi inside alphabot2pi AND a computer running Ubuntu. Keep in mind that ROS has to be installed in both devices. The raspberry image used in this project can be found [here](https://downloads.ubiquityrobotics.com/pi.html) with ROS already installed in it.

## Installation

In this section all the necessary steps to install the code are explained.

**NOTE: The installation process has to be carried out in both the Raspberry and the Computer**

Create catkin workspace. Make sure you are in a path where you want your worskpace to be and run the following commands on the terminal.
```
mkdir alphabot2pi_real_ws
cd alphabot2pi_real_ws
git clone https://github.com/nestoregon/alphabot2pi_real/
```
Change the name to src
```
mv alphabot2pi_real src 
```
Import raspicam_node to be able to use the camera. This package enables you to publish a stream video into a topic
```
cd src; git clone https://github.com/UbiquityRobotics/raspicam_node.git; cd ..
```
Create build and devel files
```
catkin_make
```
Source the workspace
```
source devel/setup.bash
```
Go to the directory with control nodes
```
cd src/control//src
```
Make the code executable
```
chmod  +x control_robot_node.py find_ball_blue_node.py servo_node.py drive_node.py remote_node.py
```
Type "ls" on the terminal to confirm that all the code is green! This means that we can run the code.

## ROS master. Configure /.bashrc files

To be able to connect the RaspberryPi and the Computer the user must configure both /.bashrc files. This file is launched everytime a new terminal window is open.

The following steps setup the Raspberry as the ROS Master and your computer as a connexion to it. More on how to edit /.bashrc files [here](http://answers.ros.org/question/272065/specification-of-ros_master_uri-and-ros_hostname/)

### 1. Raspberry /.bashrc (from Computer)
Access Raspberry Remotely

```
ssh ubuntu@ipAddressRaspberry
```
Open editor for the /.bashrc file
```
nano ~/.bashrc
```
Write the following at the bottom of the /.bashrc file. The first line is to source the workspace (**NOTE: change the path for your own workspace path!**) and the second to stablish the ROS_MASTER and ROS_IP. The Raspberry is the Master in this example. The ROS_IP is the raspberry one because we are working on the raspberry.
```
# Identify the workspace every time the terminal is opened
source /home/nestoregon/ROS/alphabot2pi_real_ws/devel/setup.bash
# office
export ROS_MASTER_URI=http://ipAddressRaspberry:11311
export ROS_IP=ipAddressRaspberry
```
Save changes
```
Ctrl + x
y
Enter
```
Run /.bashrc file again to update changes
```
source ~/.bashrc
```

### 2. Computer /.bashrc (from Computer)

Open editor for the /.bashrc file in a new terminal window
```
nano ~/.bashrc
```
Write the following at the bottom of the /.bashrc file. The first line is to source the workspace (**NOTE: change the path for your own workspace path!**) and the second to stablish the ROS_MASTER and ROS_IP. The Raspberry is the Master in this example. The ROS_IP is the computer one because we are working on the computer.
```
# Identify the workspace every time the terminal is opened
source /home/nestoregon/ROS/alphabot2pi_real_ws/devel/setup.bash
# office
export ROS_MASTER_URI=http://ipAddressRaspberry:11311
export ROS_IP=ipAddressComputer
```
Save changes
```
Ctrl + x
y
Enter
```
Run /.bashrc file again to update changes
```
source ~/.bashrc
```
## Run the code

Run the following commands to launch the whole ROS network. As mentioned above, the MASTER of the ROS network is the Raspberry and the Computer as a connexion

Firstly, ssh into the Raspberry and run:
```
roslaunch control raspberry.launch
```
Secondly, run the following command in your computer
```
roslaunch control computer.launch
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
