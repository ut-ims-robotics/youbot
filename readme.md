# Using KUKA youBot

Instructions on how to move the arm and drive around with the robot.

## 0 Things you will need
* KUKA youBot - this robot has an onboard computer in it, so it will be referred to as such.

* A computer with Ubuntu 16.04 and ROS Kinetic - referred to as a remote computer.

* Access to IMS Lab wiki.

If you need to install Ubuntu 16.04 or ROS Kinetic, the tutorials are here:

- Install Ubuntu 16.04 using [this tutorial](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0).

- Install the full desktop version of ROS Kinetic using [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

---

## 1 Setting up the remote computer

Everything in this chapter will be done on the remote computer and all the commands(lines beginning with $) will have to be entered in the terminal.

The first step is to download all the needed packages.

```bash
$ sudo apt-get install ros-kinetic-youbot-driver ros-kinetic-pr2-msgs ros-kinetic-brics-actuator ros-kinetic-moveit git
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control
```

Next we will have to make the catkin workspace if it does not already exist. You can check in your home folder if there is a folder called catkin_ws and a folder called src in catkin_ws. If yes, you will not have to do the next part, but nothing will break if you do.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now you will have to source the setup file. So you don't have to source it every time you open a new terminal we will write the source command in the .bashrc file.

```bash
$ gedit ~/.bashrc
```

This will open the text editor. You will need to add this line to the end of the file if it not already there:

```bash
source ~/catkin_ws/devel/setup.bash
```

Save the file and exit.

Before continuing you will have to close and reopen the terminal for the sourcing to work.

Now we will have to clone the youbot repository to our catkin workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone -b kinetic --recursive https://github.com/ut-ims-robotics/youbot.git
```

Now you should be able to compile the workspace.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

If this completed without any errors, you can go to the next step.

---

## 2 Simulating the robot

To simulate the youBot you will only need the remote computer which you should already have set up in the previous step.

The only thing you will have to do is to run a gazebo simulation which will visualize the simulated robot and environment. This should only be run when simulating.

```bash
$ roslaunch youbot_gazebo_robot youbot.launch 
```

The steps to manipulating the robot are the same as with the real youBot, so look under chapter 4 "Driving the robot" and chapter 5 "Moving the robot arm".

---

## 3 Running the real youBot
### 3.1 Powering the robot

Before turning on the robot you will have to connect the external power supply or battery. If you're not planning to drive the robot around, you should use only the external power supply to power the robot. The external power supply connects to a 24V input on the top of the base of the robot. The battery slides into the slot on the side of the robot and connects to a connector there.

To charge the battery connect both the battery and external power supply to the robot at the same time.

**NB! Always disconnect the battery when not in use.**

### 3.2 Turning on the robot

Press the "ON/OFF"-button on the top of the robot's base for a few seconds to turn on the power for the system. An orange light on the display will indicate that the system has been powered on successfully. Now you can turn on the motors and/or PC. To do this, press and hold the power button and when the screen shows *PC on*, release the button to turn on the PC. To turn on the motors, repeat the same step, only now release the button when the screen shows *Motor on*.

The arm has to be turned on separately. When the motors are on, there should be a button on the base of the arm with a red light. To turn on the arm, press the button. The light is green when the arm is turned on.

**NB! When you turn off the power of the arm, it will NOT hold its position and will collapse, so make sure to support the arm when turning it off.**

To turn off the system press and hold the same button and release when the screen shows *System off*. You should also power off the PC manually before turning off the system.

If you are logged in with ssh, you can do this by entering:

```bash
$ sudo shutdown -h now
```

### 3.3 Establishing an ssh connection

**NB! For the next steps the robot and your remote computer have to be in the same network.**

By default the robot should be in the youBot's access point named youbot-wifi.

**NB! The password for the access point and the youBot's onboard computer are in IMS Lab wiki. You can search youbot in the wiki and you will find it. Note that it is a secure page, so you will have  to be logged in.**

Now you can use ssh on your remote computer to log into the youBot's onboard computer.

To establish an ssh connection, enter this to your remote computer's new terminal window: 

```bash
$ ssh youbot@10.42.0.1
```

You will be asked to enter the onboard computer's password, which you can get from IMS Lab wiki.

Now you should be logged into the onboard computer. That means that when you enter commands in the terminal window you have the ssh connection in, the commands will be run on the robot's onboard computer.

If you need more terminal windows with an ssh connection, just use the same command above to  log in.


### 3.4 Setting up a remote ROS connection

First open a new terminal window(don't close the old one).

Now you will have to set up the remote connection. For that you will need to know the youBot's and the remote computer's IP addresses.

The youBot's IP address should be 10.42.0.1.

To find the remote computer's IP address enter this command to the terminal:

```bash
$ ifconfig
```

Now you should see multiple ethernet devices. The device you need should be wlp2s0 or something similar. You will need the inet addr of that device which should be something like 10.42.x.x, where the x's are specific to your computer.

If you have both IP addresses, you will have to specify ROS master aka youBot IP address on the remote computer:

```bash
$ export ROS_MASTER_URI=http://10.42.0.1:11311
```

Next, specify the remote computer's own IP(change the IP to the one you got from your remote computer):

```bash
$ export ROS_IP=10.42.x.x
```

Now check if your IP addresses are actually changed:

```bash
$ echo $ROS_MASTER_URI
$ echo $ROS_IP
```

See if the outputs are the same as you entered previously. If not, enter the commands again.

Modify /etc/hosts file in order to resolve master's name to IP:

```bash
$ gedit /etc/hosts
```

Add the following line under the hosts in the file:

```bash
10.42.0.1   youbot
```
Save and exit.

If you need multiple terminals with a ROS master, you just need to insert the two export commands in a new terminal window.


### 3.5 Running the youBot driver

For this driver to work, the youBot's motors and arm have to be turned on.

To work with the real youBot you will first have to run the youBot driver. This must be run on the youBot's onboard computer, so if you're using a remote computer you will have to run this in the terminal with an ssh connection.

This will also start roscore so you don't have to run that manually.

```bash
$ roslaunch youbot_driver_ros_interface youbot_driver.launch
```

The steps to manipulating the robot are the same as with simulating youBot, so look under chapter 4 "Driving the robot" and 5 "Moving the robot arm".

---

## 4 Driving the robot

**NB! If you're using it on the real youBot and the robot is on the table, make sure it is on a platform so it can't actually move.**

To control the robot, run keyboard teleop. If you're using the real youBot, you will have to run this in the terminal where you set your ROS master in.

The instructions on which keys to press will be displayed in the terminal window.

```bash
$ rosrun youbot_driver_ros_interface youbot_keyboard_teleop.py
```

Now you should be able to control the wheels with your keyboard. Note that the keyboard teleop only works when the terminal window where you gave the command is active.

---

## 5 Moving the robot arm

You must have the youbot driver running before doing any of the following commands.

To move the arm, you need to launch this demo file. If you're using the real youBot, you will have to run this in the terminal where you set your ROS master in.

```bash
$ roslaunch youbot_moveit demo.launch
```

By default this will open rviz, where you should now be able to see the robot model.

**NB! Make sure the robot model arm position in rviz is the same as in real life before proceeding.**

To move the arm, first you can drag the marker on the top of the robot arm in rviz to the wanted position. In the conext tab you can select a planning library(for example RRTkConfigDefault). 

Next, on the planning tab press *Plan*. Now you can see the trajectory the arm will make when moving into the wanted position. Make sure that the planned trajectory will not damage the robot.

To execute the planned trajectory, press *Execute*.

**Note:** Every time you plan/execute the trajectory, make sure that  "Select Start State" option is "current", in section "Query" under tab "Planning". Otherwise the robot will plan from previous starting state, which may end up with damaging movement.

