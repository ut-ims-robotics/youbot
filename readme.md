# This tutorial is not complete yet
# Using KUKA youBot

Instructions on how to move the arm and drive around with the robot.

## Things you will need
* KUKA youBot - this robot has an onboard computer in it, so it will be referred to as such.

* A computer with Ubuntu 16.04 and ROS Kinetic - referred to as a remote computer.

If you need to install Ubuntu 16.04 or ROS Kinetic, the tutorials are here:

- Install Ubuntu 16.04 using [this tutorial](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0).

- Install the full desktop version of ROS Kinetic using [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

---

## Setting up the remote computer
The first step is to download all the needed packages.

```bash
$ sudo apt-get install ros-kinetic-youbot-driver ros-kinetic-pr2-msgs ros-kinetic-brics-actuator ros-kinetic-moveit git
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control
```

Next we will have to make the catkin workspace if it does not already exist. You can check in your home folder if there is a folder called catkin_ws and a folder called src in it. If yes, you will not have to do the next part, but nothing will break if you do.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now you will have to source the setup file. So you don't have to source it every time you open a new terminal we will write the source command in the .bashrc file.

```bash
$ sudo gedit .bashrc
```

This will open the text editor. You will need to add this line to the end of the file if it not already there:

```bash
source ~/catkin_ws/devel/setup.bash
```

Save the file and exit.

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

## Simulating the robot

To simulate the youBot you will only need the remote computer which you should already have set up in the previous step.

the only thing you will have to do is to run a gazebo simulation which will visualize the simulated robot and environment. This should only be run when simulating.

```bash
$ roslaunch youbot_gazebo_robot youbot.launch 
```

The steps to manipulating the robot are the same as with the real youBot, so look under "Driving the robot" and "Moving the robot arm".

---

## Running the real youBot
### Powering the robot

Before turning on the robot you will have to connect the external power supply or battery. If you're not planning to drive the robot around, you should use only the external power supply to power the robot. The external power supply connects to a 24V input on the top of the base of the robot. The battery slides into the slot on the side of the robot and connects to a connector there.

To charge the battery connect both the battery and external power supply to the robot at the same time.

**NB! Always disconnect the battery when not in use.**

### Turning on the robot

Press the "ON/OFF"-button for a few seconds to turn on the power for the system. An orange light on the display will indicate that the System has been powered on successfully. Now you can turn on the motors and/or PC. To do this, press and hold the power button and when the screen shows PC on release the button to turn on the PC. Repeat the same step, only now release the button when the screen shows Motor on.

The arm has to be turned on separately. When the motors are on, there should be a button on the arm with a red light. To turn on the arm, press the button. The light is green when the arm is turned on.

**NB! When you turn off the power of the arm, it will NOT hold its position and will collapse, so make sure to support the arm when turning it off.**

To turn off the system press and hold the same button and release when the screen shows System off. You should also power off the PC manually before turning off the system.

If you are logged in with ssh, you can do this by entering:

```bash
$ sudo shutdown -h now
```

### Setting up a remote connection

**NB! For the next steps the robot and your remote computer have to be in the same network.**

By default the robot should be in the ??? network.

Now you will have to set up the remote connection. For that you will need to know the youBot's and the remote computer's IP addresses.

To find the youBot's IP address enter this command to the terminal:

```bash
$ ifconfig
```

Now you should see multiple ethernet devices. The one you need should be wlp2s0's inet addr which should be something like 192.168.x.x.

To get your remote computer's IP, you need to repeat the same step, only the ethernet device name is probably different.

If you have both IP addresses, you will have to specify ROS master aka youBot IP address on the remote computer:

```bash
$ export ROS_MASTER_URI=http://192.168.x.x:11311
```

Next, specify the remote computer's own IP:

```bash
$ export ROS_IP=192.168.x.x
```

Modify /etc/hosts file in order to resolve master's name to IP:

```bash
$ sudo gedit /etc/hosts
```

Add the following line to the end of the file:

```bash
192.168.x.x    youbot
```

### Establishing an ssh connection

Now you can use ssh on your remote computer to log into the youBot's onboard computer.

To establish an ssh connection, enter this to your remote computer's terminal: 

```bash
$ ssh youbot@192.168.x.x
```

Now you should be logged into the onboard computer. That means that when you enter commands in the terminal window you have the ssh connection in, the commands will be run on the robot's onboard computer.

If you need more terminal windows with an ssh connection, just use the same command above to  log in.

### Running the youBot driver

To work with the real youBot you will first have to run the youBot driver. This must be run on the youBot's onboard computer.

```bash
$ roslaunch youbot_driver_ros_interface youbot_driver.launch
```

The steps to manipulating the robot are the same as with simulating youBot, so look under "Driving the robot" and "Moving the robot arm".

---

## Driving the robot

**NB! If you're using it on the real youBot and the robot is on the table, make sure it is on a platform so it can't actually move.**

To control the robot, run keyboard teleop. The instructions on which keys to press will be displayed in the terminal window.

```bash
$ rosrun youbot_driver_ros_interface youbot_keyboard_teleop.py
```

Now you should be able to control the wheels with your keyboard. Note that the keyboard teleop only works when the terminal window where you gave the command is active.

---

## Moving the robot arm

You must have the youbot driver running before doing any of the following commands.

To move the arm, enter this on the remote computer:

```bash
$ roslaunch youbot_moveit real_demo.launch
```

By default this will open rviz, where you should now be able to see the robot model.

**NB! Make sure the robot model arm position in rviz is the same as in real life before proceeding.**

To move the arm, first you can drag the marker on the top of the robot arm in rviz to the wanted position. In the conext tab you can select a planning library(for example RRTkConfigDefault). 

Next, on the planning tab press *Plan*. Now you can see the trajectory the arm will make when moving into the wanted position. Make sure that the planned trajectory will not damage the robot.

To execute the planned trajectory, press *Execute*.

**Note:** Every time you plan/execute the trajectory, make sure that  "Select Start State" option is "current", in section "Query" under tab "Planning". Otherwise the robot will plan from previous starting state, which may end up with damaging movement.

