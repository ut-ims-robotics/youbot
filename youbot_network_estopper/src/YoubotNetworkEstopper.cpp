///

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "actionlib_msgs/GoalID.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <math.h>
#include <unistd.h>

using namespace std;

ros::Publisher cmd_vel_pub;
ros::Publisher arm_pub;
ros::Publisher gripper_pub;
ros::Publisher trajectory_cancel_pub;

bool received_trajectory=false;
std::string lastGoalID;

double joint[5];
double gripperr = 0;
double gripperl = 0;

bool has_received_heartbeat=false;
int heartbeat_timeout=0;
const int heartbeat_timeout_max=50;
bool bad=false;

void armCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (msg->position.size()!=7) {
    ROS_ERROR("Wrong number of joints in JointState message");
    return;
  }
  joint[0] = msg->position[0];
  joint[1] = msg->position[1];
  joint[2] = msg->position[2];
  joint[3] = msg->position[3];
  joint[4] = msg->position[4];

  gripperl = msg->position[5];
  gripperr = msg->position[6];
}

void trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg) {
  received_trajectory=true;
  lastGoalID = msg->goal_id.id;
}

void heartbeatCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (!has_received_heartbeat) ROS_INFO("Got first heartbeat");
  has_received_heartbeat=true;
  heartbeat_timeout=0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_network_estopper");

  ros::NodeHandle n;
  ROS_INFO("Starting network estopper");
  std::fill_n(joint, 5, 0);
  gripperl = 0;
  gripperr = 0;

  ROS_INFO("Subscribing to heartbeat");
  ros::Subscriber heartbeatSubscriber;
  heartbeatSubscriber = n.subscribe("youbot_network_estopper/heartbeat",1,heartbeatCallback);

  ROS_INFO("Subscribing to joint states");
  ros::Subscriber armPositionSubscriber;
  armPositionSubscriber = n.subscribe("joint_states", 1, armCallback);

  ROS_INFO("Subscribing to trajectory follower");
  ros::Subscriber jointFollowerSubscriber;
  jointFollowerSubscriber = n.subscribe("arm_1/arm_controller/follow_joint_trajectory/goal",1,trajectoryCallback);

  ROS_INFO("Advertising publishers");
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  arm_pub = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
  gripper_pub = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
  trajectory_cancel_pub = n.advertise<actionlib_msgs::GoalID>("arm_1/arm_controller/follow_joint_trajectory/cancel",1);

  ros::Rate rate(100); //Input and output at the same time... (in Hz)

  while (n.ok()){

    if (bad) {

/*      int readValue;
      static const int numberOfArmJoints = 5;
      static const int numberOfGripperJoints = 2;

      brics_actuator::JointPositions command;
      vector<brics_actuator::JointValue> armJointPositions;
      vector<brics_actuator::JointValue> gripperJointPositions;

      armJointPositions.resize(numberOfArmJoints); //TODO:change that
      gripperJointPositions.resize(numberOfGripperJoints);

      std::stringstream jointName;
      //set arm joint positions to the last recorded position (freeze arm)
      for(int i = 0; i < numberOfArmJoints; i++)
      {
        jointName.str("");
        jointName << "arm_joint_" << (i + 1);

        armJointPositions[i].joint_uri = jointName.str();
        armJointPositions[i].value = joint[i];

        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
      }
      command.positions = armJointPositions;
      arm_pub.publish(command);

      //open gripper
      gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
      gripperJointPositions[0].value = 0.01;
      gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

      gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
      gripperJointPositions[1].value = 0.01;
      gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

      command.positions = gripperJointPositions;
      gripper_pub.publish(command);

      //stop base
      geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
      cmd_vel.linear.x=0; cmd_vel.linear.y=0; cmd_vel.linear.z=0;
      cmd_vel.angular.x=0; cmd_vel.angular.y=0; cmd_vel.angular.z=0;
      cmd_vel_pub.publish(cmd_vel);
*/
      if (received_trajectory) {
        actionlib_msgs::GoalID goal_id;
        goal_id.id=lastGoalID;
        trajectory_cancel_pub.publish(goal_id);
      }

      //kill motors
      std_srvs::Empty empty;
      ros::service::call("arm_1/switchOffMotors", empty);
      ros::service::call("base/switchOffMotors", empty);

      ros::spinOnce();
      //ros::shutdown();
      //return 0;

    }

    if (has_received_heartbeat && !bad) {
      heartbeat_timeout++;
      if (heartbeat_timeout>heartbeat_timeout_max)
      {
        ROS_ERROR("HEARTBEAT TIMED OUT, spewing stop commands");
        bad=true;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
