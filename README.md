# About this repo
Start-from-here for IMS_Robotics group. This repo contains instructions for setting up the working environment with KUKA youbot, and the usage of packages.

---
# Table of contents


- [Part 1: Knowledge about KUKA youbot](#part-1-knowledge-about-kuka-youbot)
- [Part 2: Getting started](#part-2-getting-started)
    + [Step  1: Remote computer](#step-1-remote-computer)
    + [Step 2: Onboard computer](#step-2-onboard-computer)
      - [Driver and ROS wrapper](#driver-and-ros-wrapper)
      - [Communication between youbot and remote computer](#communication-between-youbot-and-remote-computer)
    + [Step 3: Installing packages](#step-3-installing-packages)
      - [Pre-requists](#pre-requists)
    + [Step 4: Running node with youbot](#step-4-running-node-with-youbot)
      - [Running the real youbot](#running-the-real-youbot)
- [Part 3: List of available packages](#part-3-list-of-available-packages)


---
## Part 1: Knowledge about KUKA youbot
Most of the information of youbot could be found in [youbot store](http://www.youbot-store.com/wiki/index.php/Main_Page).

---
## Part 2: Getting started


### Step 1: Remote computer

We work mostly ROS (Robot Operating System). ROS is supported by different Linux distributions, OS X and Windows. However, combination of Unbuntu and ROS is the most popular one, due to its stability and wide user population. We use the combination of Ubuntu + ROS as our development environment.
**NB**: the release of ROS and Ubuntu should correspond. E.g., Ubuntu 14.04 + ROS Indigo, or Ubuntu 16.04 + ROS Kinetic. The following instruction will be given an example of Ubuntu 14.04 + ROS Ingido, as they have by far the largest user group. Therefore, it'll be easier to ask questions when we encounter any.

- Install Ubuntu 14.04 following [this official tutorial](http://howtoubuntu.org/how-to-install-ubuntu-14-04-trusty-tahr).
- Install ROS Indigo following [this official tutorial](http://wiki.ros.org/indigo/Installation/Ubuntu). 
- Get familiar with ROS under [this tutorial](http://wiki.ros.org/ROS/Tutorials) (optional for now, but must for later)
- Geting to know catkin following [this tutorial](http://wiki.ros.org/catkin/Tutorials).


### Step 2: Onboard computer
(For how to turn on youbot, ask the people in the room.)

#### Driver and ROS wrapper

The onboard computer is running Ubuntu 14.04 + ROS Indigo. The driver for youbot is running on the onboard computer, which offers the interface between the hardware and higher level applications. Install the driver with the following command (you **don't** have to run it, as the driver is already installed on youbot):
```bash
sudo apt-get install ros-indigo-youbot-driver
```
In order to use ROS, we need ROS wrapper for youbot. Depending on you need to have effort states of the joints or not, you can choose one command to run (you **don't** have to actually run any, as the ROS wrapper is already installed on youbot as well):

1) if you want to have effort joint states: 
```bash
sudo apt-get install ros-indigo-brics-actuator 
cd catkin_ws/src 
git clone https://github.com/uzh-rpg/youbot_driver_ros_interface.git 
```

2) if you don't need effort joint states:
```bash
sudo apt-get install ros-indigo-youbot-driver-ros-interface ros-indigo-youbot-description 
sudo setcap cap_net_raw+ep /opt/ros/indigo/lib/youbot_driver_ros_interface/youbot_driver_ros_interface 
sudo ldconfig /opt/ros/indigo/lib 
 ```

#### Communication between youbot and remote computer

Firstly, make sure youbot and the remote computer are in the same network, either via cable or wirelessly. In our case, we have a wireless network setted up,  and youbot will connect to the prefered network automatically. For more information, please ask the people in the room.
 
For convenience, you can use ssh to log in youbot onboard computer. For more details, please ask the people in the room.
 
Set ROS master in youbot, by running the following command on youbot (optional, as this is the default setting):
```bash
export ROS_MASTER_URI=http://localhost:11311
```

Accordingly, set ROS master on youbot, by running the following command on remote computer:
```bash
export ROS_MASTER_URI=http://XXXX:11311 
# XXXX denotes the ip address of youbot. For more information, please ask people in the room.
```

By now, youbot onboard computer and remote computer are both managed by ROS master. 
 
 ---

### Step 3: Installing packages

Due to the principle of ROS, packages are transparent to each other. This means (either on onboard computer or remote computer), regardless of the location of the packages, the packages communicate flawlessly under ROS. 
The following commands can be run either on onboard computer or remote computer. Here we install on **remote computer** as an example. 

Get the available packages used/developed by IMS_Robotics group by running the following commands in **remote computer**:
```bash
cd ~/catkin_ws/src  # make sure it's the right catkin workspace location 
git clone https://github.com/ut-ims-robotics/youbot.git # get packages source code. 
cd ~/catkin_ws      # go back to the catkin workspace 
catkin_make         # compile the packages 
```
This may take a while depending on the performance of the remote computer. When finished without error, do the following:

```bash
source ~/catkin_ws/dvel/setup.bash 
```
In the future, when you develop new packages or use packages from other resources, do Step 3. 

\* For writing your own package, please check the tutorial ["How to create a ROS package"](http://wiki.ros.org/catkin/Tutorials/CreatingPackage).

#### Pre-requists
- Install [youbot-description](http://www.youbot-store.com/wiki/index.php?title=ROS_Wrapper&hmswSSOID=06b9430b85365cf0f83b50aae80e5d9820566adf) on **remote computer**:
    ```bash
    sudo apt-get install ros-indigo-youbot-description
    ```

- Install [Moveit!](http://moveit.ros.org/install/) on **remote computer**:
    ```bash
    sudo apt-get install ros-indigo-moveit
    source /opt/ros/indigo/setup.bash
    ```

---
### Step 4: Running node with youbot
The following uses running *youbot-manipulation* package as an example. For the usage of different packges, please refer to the REDAME on each package.

Start *demo* node by running
```bash
roslaunch youbot_moveit demo.launch # note that roslaunch will start a roscore if there wasn't one.
```

By default, this will open *rviz* (a visualization software), in which you could plan and visualize the movement of youbot arm.
**NB**: right now, the youbot in rviz has the state from "fake state publisher". In this way, you can safely do simulations in this environment. 

\* Tips: by editing *~/catkin_ws/src/youbot/youbot-manipulation/youbot_moveit/launch/moveit.rviz*, you can customize the visualization environment. E.g., by changing "Interactive Marker Size: 0" to "Interactive Marker Size: 0.2", you can see a draggable marker in rviz when relaunching the node. Dragging this marker to an valid position/orientation, then click on "Plan" button in "Planning" tab, you will see a trajectory is planned to move the arm from start state to goal state. By clikcing on "Execute" button in the same tab, the arm will move to the goal state. 

#### Running the real youbot
When you want to test your code on the real youbot, you should do the following:

1) running youbot-driver-ros-wrapper **on youbot**:
    ```bash
    roslaunch youbot_driver_ros_interface youbot_driver.launch
    ```

2) editing *~/catkin_ws/src/youbot/youbot-manipulation/youbot_moveit/launch/demo.launch*:
  - comment out the following lines to listen to the real joint states:
  
    ```xml
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="/use_gui" value="false"/> 
        <rosparam param="/source_list">[/move_group/fake_controller_joint_states]</rosparam>
    </node>
    ```
    
  - modify the value from "true" to "false" in the following line to disable fake execution:
  
    ```xml
    <arg name="fake_execution" value="true"/>
    ```

3) save the editing and relaunch the node by:
    ```bash
    roslaunch youbot_moveit demo.launch
    ```

Now in rviz, the youbot model should has the same state as the real youbot. If not, ask the people in the room.

You can drag the marker in rviz, then plan the trajectory. If a reasonable trajectory is found, then you can make it happen on the realy youbot by clikcing on "Execute" button.

**NB**: everytime when you plan the trajectory, make sure that you clicked on "Update" button with "Select Start State" option being "current", in section "Query" under tab "Planning". Otherwise the robot will plan from previous starting state, which may end up with demaging movement.

---
## Part 3: List of available packages:
 - ***youbot_manipulation***: essential interactive simulation environment
 - ***youbot_trajectory_plotter***: a node that listens and plots trajectory for arm (position and velocity)
    

