#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "ar_tag_manipulations/getClosestTag.h"
#include "youbot_manual_operation/executeTrajectory.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdlib>
#include "moveit/move_group_interface/move_group.h"
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include "brics_actuator/CartesianWrench.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#include "math.h"
#include <map>

geometry_msgs::Pose closestTag;     // Variable for storing the pose of closest AR tag
geometry_msgs::PoseStamped boxHandlePose;
const uint8_t numberOfJoints = 5;
const double pi = 3.1416;
bool exitNode = false;

// States, that will be used in the state machine
enum boxDemoStates { init, scanTags, rotateToTag, planPath, executePlan, liftBox, turnAround, givePackage, error, visualizeBox, recovery};

// Map is used just for debug message purposes
std::map <boxDemoStates, std::string> stateNames = { {init, "init"}, {scanTags, "scanTags"}, {rotateToTag, "rotateToTag"},
                                                     {planPath, "planPath"}, {executePlan, "executePlan"}, {liftBox, "liftBox"},
                                                     {turnAround, "turnAround"}, {givePackage, "givePackage"}, {error, "error"},
                                                     {visualizeBox, "visualizeBox"}, {recovery, "recovery"} };

boxDemoStates state = recovery;
boxDemoStates previousState = state;

// Define timeout periods, can be members of the map but this is for speed considerations. 0 means infinite timeout period
const float initTO = 10;
const float scanTagsTO = 10*3;
const float rotateToTagTO = 15;
const float planPathTO = 10;
const float executePlanTO = 0;
const float liftBoxTO = 7;
const float turnAroundTO = 15;
const float givePackageTO = 0;
const float errorTO = 0;
const float visualizeBoxTO = 2;
const float recoveryTO = 0;

// A vector for accessing timeouts conveniently, order is obviously important
float timeoutPeriods[] = {initTO, scanTagsTO, rotateToTagTO, planPathTO,
                          executePlanTO, liftBoxTO, turnAroundTO, givePackageTO,
                          errorTO, visualizeBoxTO,recoveryTO};
ros::Time startingTime;

// Function for publishing different frames
void publishTransformation(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.position.x,
                                     pose.position.y,
                                     pose.position.z) );

    transform.setRotation(tf::Quaternion(pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w) );

    ROS_INFO("publishing transformation FROM: %s TO: %s", fromFrame.c_str(), toFrame.c_str());
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fromFrame, toFrame));
}

// Callback for processing timer ticks
void timerCallback(const ros::TimerEvent& msg)
{
    if (timeoutPeriods[int (state)] != 0)
    {
        double timeDiffSec = ros::Time::now().toSec() - startingTime.toSec();
        if (timeDiffSec > timeoutPeriods[state])
        {
            ROS_ERROR("Timed out at state=%s, with timeoutPeriod=%f", stateNames[state].c_str(), timeoutPeriods[int (state)]);

            previousState = state;
            state = error;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_box_demo");

    ros::NodeHandle n;
    startingTime = ros::Time::now();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Timer for monitoring durations, if a something gets stuck and times out then state machine goes to error state
    ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

    // Service Client for getting the closest AR tag
    ros::ServiceClient client = n.serviceClient<ar_tag_manipulations::getClosestTag>("get_closest_tag");
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("tag_marker", 1);

    // Service client for executing the trajectories
    ros::ServiceClient executeTrajectoryClient = n.serviceClient<youbot_manual_operation::executeTrajectory>("execute_trajectory");

    // Arm and gripper positions publisher
    ros::Publisher armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
    ros::Publisher gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

    // Rviz trajectory visualization publisher
    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Marker visualization and transofrmation things
    uint32_t shape = visualization_msgs::Marker::CUBE;
    tf::TransformBroadcaster tag_frame_broadcaster;
    tf::TransformListener listener;

    boxHandlePose.pose.position.x = 0.022; //0.03
    boxHandlePose.pose.position.y = 0.083; //0.075
    boxHandlePose.pose.position.z = -0.025;

    //Move group configuration
    moveit::planning_interface::MoveGroup robot("arm_1");
    moveit::planning_interface::MoveGroup::Plan latest_plan;

    robot.setPlannerId("RRTConnectkConfigDefault");
    robot.setNumPlanningAttempts(2);
    robot.setPlanningTime(3);

    // Playing around with tolerances
    robot.setGoalPositionTolerance(0.001);
    robot.setGoalOrientationTolerance(0.001);
    robot.setGoalJointTolerance(0.001);

    // Printing out some debug information
    ROS_INFO("[robot_move/main] Planning frame: %s", robot.getPlanningFrame().c_str());
    ROS_INFO("[robot_move/main] End effector link: %s", robot.getEndEffectorLink().c_str());

    // Create a command for initial position
    std::vector <brics_actuator::JointValue> armJointPositionsScan;
    armJointPositionsScan.resize(numberOfJoints); //TODO:change that

    std::vector <brics_actuator::JointValue> armJointPositionsLift;
    armJointPositionsLift.resize(numberOfJoints); //TODO:change that

    std::string jointNames[] = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
    double jointValuesScan[] = {2.8303333199110505, 0.07704957367073428, -0.7995510383018705, 3.1974333568127555, 4.469335903678088};
    double scanLeft = 2.280565;
    double scanRight = 3.370627;
    int scanCounter = 2;
    double yawDiff;

    double jointValuesLift[] = {3.0310025506590983, 1.5327548619062357, -0.0707643745221101, 0.4116371331879697, 4.467101390593492};


    for (int i=0; i<numberOfJoints; i++)
    {
        armJointPositionsScan[i].joint_uri = jointNames[i];
        armJointPositionsScan[i].value = jointValuesScan[i];
        armJointPositionsScan[i].unit = boost::units::to_string(boost::units::si::radians);

        armJointPositionsLift[i].joint_uri = jointNames[i];
        armJointPositionsLift[i].value = jointValuesLift[i];
        armJointPositionsLift[i].unit = boost::units::to_string(boost::units::si::radians);
    }

    brics_actuator::JointPositions commandScan;
    brics_actuator::JointPositions commandScanLeft;
    brics_actuator::JointPositions commandScanRight;

    brics_actuator::JointPositions commandTurn;
    brics_actuator::JointPositions commandLift;

    commandScan.positions = armJointPositionsScan;
    commandScanLeft.positions = armJointPositionsScan;
    commandScanRight.positions = armJointPositionsScan;
    commandScanLeft.positions[0].value = scanLeft;
    commandScanRight.positions[0].value = scanRight;

    brics_actuator::JointPositions scanCommands[] = {commandScan, commandScanLeft, commandScanRight};

    commandTurn.positions = armJointPositionsScan;
    commandLift.positions = armJointPositionsLift;

    // Create a command for the gripper
    std::vector <brics_actuator::JointValue> gripperJointPositions;
    brics_actuator::JointPositions gripperCommand;
    gripperJointPositions.resize(2);

    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";

    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

    bool firstCycle = true;

    // Main loop
    while(ros::ok() && !exitNode)
    {
        startingTime = ros::Time::now();

        switch(state)
        {
            std::cout << std::endl;
            // Case initialize: 1)Open the gripper,
            case init:
            {
                ROS_INFO("---------*** New cycle ***---------\n");

                // Open the gripper
                ROS_INFO("[state:%s] Opening the gripper ...", stateNames[state].c_str());
                gripperJointPositions[0].value = 0.0114;
                gripperJointPositions[1].value = 0.0114;

                gripperCommand.positions = gripperJointPositions;
                gripperPositionPublisher.publish(gripperCommand);

                ros::Duration(1.0).sleep();
                // Change states
                if (state != error)
                {
                    previousState = state;
                    state = scanTags;
                }
            }
            break;

            // Case Scan tags: 1)Go to scanning position
            case scanTags:
            {
                // Go to scanning position
                float stabilTime = 3;

                // Get the location of closest ar tag over a service
                ar_tag_manipulations::getClosestTag srv;    // Service message for storing info
                srv.request.scanDuration = 4.0;             // Scan for 4 seconds
                srv.response.tagID = 999;                   // Default response, indicating a failure

                // Call the server until tagID is not equal to default response or is error
                //scanCounter = 0;
                if (!firstCycle && (previousState != error))
                    scanCounter++;

                while ((srv.response.tagID == 999) && (state != error))
                {
                    ROS_INFO("[state:%s] Moving arm to scanning position and waiting %f seconds until scanning starts.",stateNames[state].c_str(), stabilTime);
                    armPositionsPublisher.publish(scanCommands[scanCounter%3]);
                    ros::Duration(stabilTime).sleep();

                    while ( (!client.call(srv)) && (state != error))
                    {
                        ROS_ERROR("[state:%s] Failed to call service /get_closest_tag, trying again...", stateNames[state].c_str());
                    }

                    if (srv.response.tagID == 999)
                        scanCounter++;
                }

                if (state == scanTags)
                {
                    ROS_INFO("[state:%s] Got a response, tagID = %d", stateNames[state].c_str(), srv.response.tagID);
                    closestTag = srv.response.tagPose;

                    // first, we'll publish the transform over tf
                    publishTransformation(closestTag, "base_link", "ar_tag_frame");

                    // Publish the box handle frame

                    boxHandlePose.pose.orientation.x = 0;
                    boxHandlePose.pose.orientation.y = 0;
                    boxHandlePose.pose.orientation.z = 0;
                    boxHandlePose.pose.orientation.w = 1;

                    //boxHandlePose.pose.orientation = closestTag.orientation;
                    publishTransformation(boxHandlePose.pose, "ar_tag_frame", "box_handle_frame");
                }

                // Change states
                if (state == scanTags)
                {
                    previousState = state;
                    state = rotateToTag;
                }
            }
            break;

            // Case face towards the nearest box
            case rotateToTag:
            {
                // Get the transformation from /arm_link_1 to /box_handle_frame
                tf::StampedTransform frameTransform;

                std::string fromFrame = "arm_link_1";
                std::string toFrame = "box_handle_frame";

                publishTransformation(closestTag, "base_link", "ar_tag_frame");
                publishTransformation(boxHandlePose.pose, "ar_tag_frame", "box_handle_frame");

                bool transformRecieved = false;
                ROS_INFO("[state:%s] Trying to get the transformation FROM: %s TO: %s",stateNames[state].c_str(), fromFrame.c_str(), toFrame.c_str());
                while (!transformRecieved && (state != error))
                {
                    try
                    {
                        listener.lookupTransform(fromFrame.c_str(), toFrame.c_str(), ros::Time(0), frameTransform);
                        transformRecieved = true;
                    }
                    catch (tf::TransformException ex)
                    {
                        ROS_ERROR("[state:%s] Could not get the transformation: %s",stateNames[state].c_str(), ex.what());
                        ros::Duration(1.0).sleep();
                    }
                }

                if (state != error)
                {
                    ROS_INFO("[state:%s] Got the transformation FROM: %s TO: %s",stateNames[state].c_str(), fromFrame.c_str(), toFrame.c_str());

                    // Calculate the rotation angle
                    yawDiff = atan2( frameTransform.getOrigin().y(), frameTransform.getOrigin().x() );
                    commandTurn.positions[0].value = scanCommands[scanCounter%3].positions[0].value - yawDiff;
                    ROS_INFO("[state:%s] yaw rotation = %lf",stateNames[state].c_str(), yawDiff);

                    // Turn the arm towards the tag
                    youbot_manual_operation::executeTrajectory srv;    // Service message for storing info
                    srv.request.executionType = 3;                     // 3 = execute approach
                    srv.request.joint1Changed = true;
                    srv.request.joint1Value = commandTurn.positions[0].value;

                    // Call the server until response or timeout
                    while ( (!executeTrajectoryClient.call(srv)) && (state != error))
                    {
                        ROS_ERROR("[state:%s] Failed to call service /execute_trajectory, trying again...", stateNames[state].c_str());
                    }

                    // If the server responds with some weird s*it, then dont trust it and change state to error
                    if (srv.response.executionTypeResp != 3)
                    {
                        ROS_ERROR("[state:%s] Invalid response from /execute_trajectory", stateNames[state].c_str());
                        previousState = state;
                        state = error;
                    }

                    ros::Duration(1.0).sleep();
                }

                // Change states
                if (state != error)
                {
                    previousState = state;
                    state = planPath;
                }
            }
            break;

            // Plan a cartesian path to the box handle: 1)
            case planPath:
            {
                // Set current state as the start state for planner. For some reason the actual built-in function doesn't do that.
                robot.setStartStateToCurrentState();
                ros::Duration(0.1).sleep();

                // Get current pose of the end effector
                geometry_msgs::PoseStamped current_pose = robot.getCurrentPose();

                ROS_INFO("[state:%s] frame of current_pose: %s",stateNames[state].c_str(), current_pose.header.frame_id.c_str());
                ROS_INFO("[state:%s] end effector link: %s",stateNames[state].c_str(), robot.getEndEffectorLink().c_str());

                publishTransformation(current_pose.pose, "base_footprint", "current_pose");

                // Create a target pose based on markers location. This looks like somekind of a witchcraft
                geometry_msgs::PoseStamped target_pose;
                target_pose.header.frame_id = "/base_footprint";

                // Position should be where the box handle is
                // Get the transformation from /base_footprint to /box_handle_frame
                std::string fromFrame = "base_footprint";
                std::string toFrame = "box_handle_frame";

                publishTransformation(closestTag, "base_link", "ar_tag_frame");
                publishTransformation(boxHandlePose.pose, "ar_tag_frame", "box_handle_frame");

                publishTransformation(closestTag, "base_link", "ar_tag_frame");
                publishTransformation(boxHandlePose.pose, "ar_tag_frame", "box_handle_frame");

                ros::Duration(0.5).sleep();

                tf::StampedTransform frameTransform;
                bool transformRecieved = false;
                ROS_INFO("[state:%s] Trying to get the transformation FROM: %s TO: %s",stateNames[state].c_str(), fromFrame.c_str(), toFrame.c_str());
                while (!transformRecieved && (state != error))
                {
                    try
                    {
                        listener.lookupTransform(fromFrame.c_str(), toFrame.c_str(), ros::Time(0), frameTransform);
                        transformRecieved = true;
                    }
                    catch (tf::TransformException ex)
                    {
                        ROS_ERROR("[state:%s] Could not get the transformation: %s",stateNames[state].c_str(), ex.what());
                        ros::Duration(1.0).sleep();
                    }
                }

                if (state != error)
                {
                    ROS_INFO("[state:%s] Got the transformation FROM: %s TO: %s",stateNames[state].c_str(), fromFrame.c_str(), toFrame.c_str());

                    target_pose.pose.position.x = frameTransform.getOrigin().x();
                    target_pose.pose.position.y = frameTransform.getOrigin().y();
                    target_pose.pose.position.z = frameTransform.getOrigin().z();

                    target_pose.pose.orientation = current_pose.pose.orientation;
                    publishTransformation(target_pose.pose, "base_footprint", "target_pose");

                    // There is a bug when movegroup is used for the first cycle, why? no idea. Movegroup
                    // returns previous orientation of the endeffector, rather than current one.
                    // Workaround "robot.setStartState( *(robot.getCurrentState()) )" does not help here.
                    if (!firstCycle)
                    {
                        // CARTESIAN PATH
                        std::vector<geometry_msgs::Pose> waypoints;
                        waypoints.push_back(current_pose.pose);
                        waypoints.push_back(target_pose.pose);

                        // ATM of writing the code, youbot driver ROS wrapper was unstable when executing
                        // the trajectories, the workaround was to actually listen to trajectory visualization
                        // messages with another node running in the background, which executes the
                        // trajectory by sending waypoints without using trajectory action service (when you
                        // run "computeCartesianPath", a visualization message is sent to rviz by default).

                        moveit_msgs::RobotTrajectory trajectory;
                        double fraction = robot.computeCartesianPath(waypoints, 0.0025, 0.0, trajectory);

                        ROS_INFO("[state:%s] Visualizing plan 4 (cartesian path) (%.2f%% acheived)",stateNames[state].c_str(), fraction * 100.0);
                        /* Sleep to give Rviz time to visualize the plan. */
                        //sleep(7.0);
                    }
                    else
                        firstCycle = false;
                }

                // Change states
                if (state != error)
                {
                    previousState = state;
                    state = executePlan;
                }
            }
            break;

            // Execute the cartesian path: Currently, users confirmation is needed
            case executePlan:
            {
                bool validInput = false;

                while (!validInput)
                {
                    // Get users confirmation
                    ROS_INFO("[state:%s] TRAJECTORY IS OK? possible choises:\n* 1 = traj correct\n* 2 = replan,\n* 999 = change state to error",stateNames[state].c_str());
                    int input;
                    std::cout << "Type: ";
                    std::cin >> input;

                    if (input == 1)
                    {
                        validInput = true;

                        // Send a service request
                        youbot_manual_operation::executeTrajectory srv;    // Service message for storing info
                        srv.request.executionType = 1;                     // 1 = execute box grasping
                        srv.request.joint1Changed = false;
                        srv.request.joint1Value = 0;

                        // Call the server until response or timeout
                        while ( (!executeTrajectoryClient.call(srv)) && (state != error))
                        {
                            ROS_ERROR("[state:%s] Failed to call service /execute_trajectory, trying again...", stateNames[state].c_str());
                        }

                        // If the server responds with some weird s*it, then dont trust it and change state to error
                        if (srv.response.executionTypeResp != 1)
                        {
                            ROS_ERROR("[state:%s] Invalid response from /execute_trajectory", stateNames[state].c_str());
                            previousState = state;
                            state = error;
                        }

                        // Change states
                        if (state != error)
                        {
                            previousState = state;
                            state = liftBox;
                        }
                    }
                    else if (input == 2)
                    {
                        validInput = true;
                        previousState = state;
                        state = init;
                    }
                    else if (input == 999)
                    {
                        validInput = true;
                        previousState = state;
                        state = error;
                    }
                    else
                        std::cout << "Invalid input" << std::endl;
                }
            }
            break;

            // Lift the box up
            case liftBox:
            {
                // Firstly close the gripper
                ROS_INFO("[state:%s] Closing the gripper ...",stateNames[state].c_str());
                gripperJointPositions[0].value = 0.0;
                gripperJointPositions[1].value = 0.0;

                gripperCommand.positions = gripperJointPositions;
                gripperPositionPublisher.publish(gripperCommand);
                ros::Duration(2.5).sleep();

                // Lift the box to predefined position
                ROS_INFO("[state:%s] Lifting the box ...", stateNames[state].c_str());
                commandLift.positions[0].value = scanCommands[scanCounter%3].positions[0].value - yawDiff;
                armPositionsPublisher.publish(commandLift);
                ros::Duration(1.0).sleep();

                // Change states
                if (state != error)
                {
                    previousState = state;
                    state = turnAround;
                }
            }
            break;

            // Turn towards the santa
            case turnAround:
            {
                // Send a service request
                youbot_manual_operation::executeTrajectory srv;    // Service message for storing info
                srv.request.executionType = 2;                     // 2 = execute box turning around
                srv.request.joint1Changed = false;
                srv.request.joint1Value = 0;

                // Call the server until response or timeout
                while ( (!executeTrajectoryClient.call(srv)) && (state != error))
                {
                    ROS_ERROR("[state:%s] Failed to call service /execute_trajectory, trying again...", stateNames[state].c_str());
                }

                // If the server responds with some weird s*it, then dont trust it and change state to error
                if (srv.response.executionTypeResp != 2)
                {
                    ROS_ERROR("[state:%s] Invalid response from /execute_trajectory", stateNames[state].c_str());
                    previousState = state;
                    state = error;
                }

                // Change states
                if (state != error)
                {
                    previousState = state;
                    state = givePackage;
                }
            }
            break;

            // Wait till santa grabs the box
            case givePackage:
            {
                // Open the gripper
                ROS_INFO("[state:%s] Opening the gripper ...", stateNames[state].c_str());
                gripperJointPositions[0].value = 0.0114;
                gripperJointPositions[1].value = 0.0114;

                gripperCommand.positions = gripperJointPositions;
                gripperPositionPublisher.publish(gripperCommand);

                ros::Duration(2.0).sleep();

                bool validInput = false;

                while (!validInput)
                {
                    // Get users confirmation
                    ROS_INFO("[state:%s] PACKAGE DELIVERED? possible choises:\n* 1 = package taken\n* 999 = change state to error",stateNames[state].c_str());
                    int input;
                    std::cout << "Type: ";
                    std::cin >> input;

                    if (input == 1)
                    {
                        validInput = true;
                    }
                    else if (input == 999)
                    {
                        validInput = true;
                        previousState = state;
                        state = error;
                    }
                    else
                        std::cout << "Invalid input" << std::endl;
                }

                // Change states
                if (state != error)
                {
                    previousState = state;
                    state = init;
                }
            }
            break;

            // Visualize the box in rviz
            case visualizeBox:
            {
                // Publish the marker on Rviz
                visualization_msgs::Marker marker;
                marker.header.frame_id = "ar_tag_frame";
                marker.header.stamp = ros::Time::now();
                marker.ns = "basic_shapes";
                marker.id = 0;
                marker.type = shape;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 0.065;
                marker.pose.position.y = -0.03;
                marker.pose.position.z = -0.020;

                boxHandlePose.pose.position.x = 0.065;
                boxHandlePose.pose.position.y = -0.03;
                boxHandlePose.pose.position.z = -0.020;

                marker.pose.orientation = closestTag.orientation;

                // Scale of the marker
                marker.scale.x = 0.01;
                marker.scale.y = 0.01;
                marker.scale.z = 0.01;

                // Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;

                // Publish the marker
                marker.lifetime = ros::Duration();
                marker_pub.publish(marker);

                // Change states
                if (state != error)
                {
                    previousState = state;
                    state = rotateToTag;
                }
            }
            break;

            // Error state
            case error:
            {
                ROS_ERROR("*** !WELCOME TO THE TWILIGHT ZONE! ***");
                std::cout << std::endl;

                ROS_INFO("Got an error from the state: %s.\n Choose how to proceed by changing the state manually", stateNames[previousState].c_str());
                std::cout << "States and according numbers:" << std::endl;

                for (auto& name : stateNames)
                {
                    std::cout << "* " << name.first << " - " << name.second.c_str() << std::endl;
                }
                std::cout << "* " << 999 << " - " << "exit the program" << std::endl;

                bool validInput = false;
                while (!validInput)
                {
                    // Get users confirmation
                    int input;
                    std::cout << "Enter the state number: ";
                    std::cin >> input;

                    auto search = stateNames.find(boxDemoStates(input));
                    if (search != stateNames.end())
                    {
                        ROS_INFO ("received smth");
                        validInput = true;
                        state = boxDemoStates (input);
                    }
                    else if (input == 999)
                    {
                        ROS_INFO ("Exiting");
                        validInput = true;
                        exitNode = true;
                    }
                    else
                        std::cout << "Invalid input" << std::endl;
                }
                previousState = error;
            }
            break;

            case recovery:
            {

                bool validInput = false;
                while (!validInput)
                {
                    // Firstly just open the gripper
                    ROS_INFO("[state:%s] Opening the gripper ...", stateNames[state].c_str());
                    gripperJointPositions[0].value = 0.0114;
                    gripperJointPositions[1].value = 0.0114;

                    gripperCommand.positions = gripperJointPositions;
                    gripperPositionPublisher.publish(gripperCommand);

                    // Get users confirmation
                    int input;
                    std::cout << "Proceed to init state? (1 = yes, 999 = error mode, else = open gripper): ";
                    std::cin >> input;

                    if (input == 1)
                    {
                        validInput = true;
                        state = init;
                    }
                    else if (input == 999)
                    {
                        validInput = true;
                        state = error;
                    }
                    else
                        std::cout << "Opening gripper or invalid input" << std::endl;
                }
            }
            break;
        }
    }
    return 0;
}



// Function for getting the coordinate transforms
/*
tf::StampedTransform lookupTransformation (std::string fromFrame, std::string toFrame, tf::TransformListener *listener)
{
    tf::StampedTransform frameTransform;

    bool transformRecieved = false;
    ROS_INFO("Trying to get the transformation FROM: %s TO: %s", fromFrame.c_str(), toFrame.c_str());
    while (!transformRecieved)
    {
        try
        {
            listener->lookupTransform(fromFrame, toFrame, ros::Time(0), frameTransform);
            transformRecieved = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Could not get the transformation: %s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    ROS_INFO("Got the transformation FROM: %s TO: %s", fromFrame.c_str(), toFrame.c_str());
    return frameTransform;
}
*/

/*
        double roll, pitch, yaw;
        tf::Quaternion quatIn(current_pose.pose.orientation.x,
                              current_pose.pose.orientation.y,
                              current_pose.pose.orientation.z,
                              current_pose.pose.orientation.w);

        tf::Matrix3x3 m(quatIn);
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("Before: roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll*180.0/pi, pitch*180.0/pi, yaw*180.0/pi);

        yaw += yawDiff;
        ROS_INFO("After: roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll*180.0/pi, pitch*180.0/pi, yaw*180.0/pi);

        tf::Quaternion quatOut(roll, pitch, yaw);
        quatOut.setRPY(roll, pitch, yaw);

        target_pose.pose.orientation.x = quatOut.x();
        target_pose.pose.orientation.y = quatOut.y();
        target_pose.pose.orientation.z = quatOut.z();
        target_pose.pose.orientation.w = quatOut.w();
*/
