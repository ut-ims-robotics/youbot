/*
* This node subscribes to the topic joy and converts joystick data to real SI velocity commands
* Velocity commands will be published on cmd_vel topic
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "std_srvs/Empty.h"

#include "brics_actuator/CartesianWrench.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#define JOY_TIMEOUT_SEC 0.5
#define MAX_TRANSLATION_SPEED 0.5
#define MAX_ROTATION_SPEED 1


class YoubotJoy
{
public:
  YoubotJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void timerCallback(const ros::TimerEvent&);
  void resetArm();
  void raiseArm();
  void lowerArm();
  void setArmPosition(std::vector<double> values);
  void toggleGripper();
  void toggleMotors();

  ros::NodeHandle nh_;

  ros::Publisher armPositionsPublisher;
  ros::Publisher gripperPositionPublisher;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer; 
  ros::Time last_joy_time;
  const std::vector<std::string> joint_names = 
    {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
  bool gripperOpen = false;
  bool motorsEnabled = true;
};


YoubotJoy::YoubotJoy()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &YoubotJoy::joyCallback, this);
  timer = nh_.createTimer(ros::Duration(0.1), &YoubotJoy::timerCallback, this);
  last_joy_time = ros::Time::now();
  armPositionsPublisher = nh_.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 100);
  gripperPositionPublisher = nh_.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
}

void YoubotJoy::toggleMotors()
{
  if (!motorsEnabled)
    {
        ROS_INFO("Enabling the motors ...");
        //kill motors
        std_srvs::Empty empty;
        ros::service::call("arm_1/switchOnMotors", empty);
        ros::service::call("base/switchOnMotors", empty);
        motorsEnabled = true;
    }

    else
    {
        ROS_INFO("Disabling the motors ...");
        //kill motors
        std_srvs::Empty empty;
        ros::service::call("arm_1/switchOffMotors", empty);
        ros::service::call("base/switchOffMotors", empty);
        motorsEnabled = false;
    }
}

void YoubotJoy::toggleGripper()
{
brics_actuator::JointPositions command;
std::vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(2);

    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";

    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

    //Close the gripper
    if (gripperOpen == true)
    {
        ROS_INFO("Closing the gripper ...");
        gripperJointPositions[0].value = 0.0;
        gripperJointPositions[1].value = 0.0;
        gripperOpen = false;
    }

    //Close the gripper
    else if (gripperOpen == false)
    {
        ROS_INFO("Opening the gripper ...");
        gripperJointPositions[0].value = 0.0114;
        gripperJointPositions[1].value = 0.0114;
        gripperOpen = true;
    }

    command.positions = gripperJointPositions;
    gripperPositionPublisher.publish(command);
}

void YoubotJoy::setArmPosition(std::vector<double> values) 
{
  brics_actuator::JointPositions command;
  std::vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(5);

  for (size_t i = 0; i < joint_names.size(); i++)
  {
    armJointPositions[i].joint_uri = joint_names[i];
    armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    armJointPositions[i].value = values[i];
  }
  command.positions = armJointPositions;
  armPositionsPublisher.publish(command);
}

void YoubotJoy::resetArm()
{
  const std::vector<double> values = {2.896296696429213, 0.0637984969652081, -0.0744557458900781, 0.10856193768426137, -0.08991149679006281};
  setArmPosition(values);
}

void YoubotJoy::lowerArm()
{
  ROS_INFO_STREAM("Lowering arm");
  const std::vector<double> values = {2.9598837400427374, 2.31433679576615, -1.662075301344948, 1.3820574047683154, -0.17665927703460915};
  setArmPosition(values);
}

void YoubotJoy::raiseArm()
{
  ROS_INFO_STREAM("Raising arm");
  const std::vector<double> values = {2.9598837400427374, 1.1758820195071027, -2.6163654857993834, 3.4172120992124806, -0.22199113159239428};
  setArmPosition(values);
}

void YoubotJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // check the deadman's switch
  if (!joy->buttons[5])  // RB
  {
    return; //We have a problem
  }

  if (joy->buttons[0]) // A/X
  {
    lowerArm();
  }
  else if (joy->buttons[2]) // TRIANGLE/Y
  {
    raiseArm();
  }
  if (joy->buttons[1])  // CIRCLE/A
  {
    toggleGripper();
  } 
  if (joy->buttons[4]) // LB
  {
    toggleMotors();
  }
  if (joy->buttons[3]) // SQUARE/X
  {
    resetArm();
  }

  last_joy_time = ros::Time::now();

  geometry_msgs::Twist twist;
  twist.linear.x = joy->axes[1]*MAX_TRANSLATION_SPEED;
  twist.linear.y = joy->axes[0]*MAX_TRANSLATION_SPEED;
  twist.angular.z = joy->axes[3]*MAX_ROTATION_SPEED;

  vel_pub_.publish(twist);
  // ROS_INFO_STREAM(twist);
}

void YoubotJoy::timerCallback(const ros::TimerEvent&)
{
  if (ros::Time::now() - last_joy_time > ros::Duration(JOY_TIMEOUT_SEC))
  {
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;

    vel_pub_.publish(twist);
    
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_joy");
  YoubotJoy youbot_joy;
  ROS_INFO_STREAM("Started joy listener.");
  ROS_INFO_STREAM("Dead man's switch on RB, has to be held down to operate");
  ROS_INFO_STREAM("Move using joysticks, toggle gripper using Circle/A");
  ros::spin();
}
