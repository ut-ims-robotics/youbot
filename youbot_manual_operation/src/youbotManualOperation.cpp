#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"
#include "keyboard_reader/Key.h"
#include <string>

#include "brics_actuator/CartesianWrench.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

bool monitorStates = false;
bool recordStates = false;
bool execute = false;
bool gripperOpen = false;

int numberOfJoints = 5;

brics_actuator::JointPositions command;
std::vector <brics_actuator::JointPositions> commandList;

ros::Publisher armPositionsPublisher;
ros::Publisher gripperPositionPublisher;

// Callback function for playing back/executing the recorded joint states
void playBackCallback()
{
    if (commandList.size() <= 0)
         ROS_ERROR("Cannot playBack, No entries in the commandList.");
    else
    {
        ROS_INFO("");
        ROS_INFO("- PLAYING BACK THE LIST OF RECORDED STATES -");
        ROS_INFO("- %d states in the list.", commandList.size());
        int i;
        for (i=0; i<commandList.size(); i++)
        {            
            if (commandList[i].positions[0].joint_uri == "gripper_finger_joint_l")
            {
                ROS_INFO("- Moving to recorded position: %d (gripper)", i+1);
                gripperPositionPublisher.publish(commandList[i]);
            }
            else
            {
                ROS_INFO("- Moving to recorded position: %d (arm)", i+1);
                armPositionsPublisher.publish(commandList[i]);
            }

            ros::Duration(7).sleep();	//sleep for 7 seconds
        }
        ROS_INFO("- playBack finished! -");
        ROS_INFO("");
    }
}

// Callback function for enabling or disabling the motors
void motorSwitchCallback(bool state)
{
    if (state == true)
    {
        ROS_INFO("Enabling the motors ...");
        //kill motors
        std_srvs::Empty empty;
        ros::service::call("arm_1/switchOnMotors", empty);
        ros::service::call("base/switchOnMotors", empty);
    }

    else if (state == false)
    {
        ROS_INFO("Disabling the motors ...");
        //kill motors
        std_srvs::Empty empty;
        ros::service::call("arm_1/switchOffMotors", empty);
        ros::service::call("base/switchOffMotors", empty);
    }
}

// Callback function for opening/closing the gripper
void gripperCallback()
{
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
    commandList.push_back(command);
    gripperPositionPublisher.publish(command);
}

// Callback function for clearing the recorded joint states
void clearCommandListCallback()
{
    ROS_INFO("Clearing the recorded joint state list");
    commandList.clear();
}

// Callback for recieving joint state information
void jointCallback(const sensor_msgs::JointState & msg)
{
    if (msg.name[0].compare("arm_joint_1") == 0)
    {
        if (monitorStates == true)
        {
            ROS_INFO("names: [%s, %s, %s, %s, %s]", msg.name[0].c_str(), msg.name[1].c_str(), msg.name[2].c_str(), msg.name[3].c_str(), msg.name[4].c_str() );
            ROS_INFO("values: [%f, %f, %f, %f, %f]", msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4] );
        }

        // record the states
        if (recordStates == true)
        {
            std::vector <brics_actuator::JointValue> armJointPositions;
            armJointPositions.resize(numberOfJoints); //TODO:change that

            int i;
            for (i=0; i<numberOfJoints; i++)
            {
                armJointPositions[i].joint_uri = msg.name[i];
                armJointPositions[i].value = msg.position[i];
                armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
            }

            command.positions = armJointPositions;
            commandList.push_back(command);
            recordStates = false;
        }
    }
}

// Callback for processing keyboard events
void keyboardCallback(keyboard_reader::Key kbCommand)
{
    if (kbCommand.key_pressed == true)
    {
        // Record joint states: "r" key
        if (kbCommand.key_code == 0x0013 )
        {
            recordStates = true;
            ROS_INFO("Recording the joint states ...");
        }

        // "Play back" eg. execute the list of recorded commands: "p" key
        else if (kbCommand.key_code == 0x0019)
        {
            playBackCallback();
        }

        // Clear the recorded joint state list: "c" key
        else if (kbCommand.key_code == 0x002e)
        {
            clearCommandListCallback();
        }

        // Start/stop monitoring joint states (does not record): "m" key
        else if (kbCommand.key_code == 0x0032)
        {
            if (monitorStates == false)
            {
                monitorStates = true;
                ROS_INFO("Starting to monitor the joint states ...");
            }
            else
            {
                monitorStates = false;
                ROS_INFO("Stopping monitoring the joint states ...");
            }
        }

        // eXecute the recorded state (move): "x" key
        else if (kbCommand.key_code == 0x002d)
        {
            armPositionsPublisher.publish(command);
            ROS_INFO("MOVING TO LAST RECORDED STATE ...");
        }

        // Take current off from the joints: "d" key
        else if (kbCommand.key_code == 0x0020)
        {
            motorSwitchCallback(false);
        }

        // Put current back on the motors: "e" key
        else if (kbCommand.key_code == 0x0012)
        {
            motorSwitchCallback(true);
        }
        // Open/close the gripper: "g" key
        else if (kbCommand.key_code == 0x0022)
        {
            gripperCallback();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_joint_recorder");
    ros::NodeHandle n;

    armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
    gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

    // Listen to joint states and keyboard events
    ros::Subscriber sub = n.subscribe("/joint_states", 10, jointCallback);
    ros::Subscriber sub_kb_event = n.subscribe<keyboard_reader::Key>("keyboard", 1, keyboardCallback);

    ROS_INFO("\n* e - Enable motors \n* d - Disable motors \n* r - Record current joint states \n* c - Clear recorded joint states \n* p - Play back the recorded joint states \n* * *");
    //
    ros::spin();

    return 0;
}

