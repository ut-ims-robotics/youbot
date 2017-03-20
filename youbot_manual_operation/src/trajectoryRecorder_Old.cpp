#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include "keyboard_reader/Key.h"
#include "boost/bind.hpp"
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include "math.h"

// Enum for keeping track on recording process
enum trajectoryRecordingStates { waiting, starting, recording };
int numberOfJoints = 5;
float pi = 3.1416;
float jointSpeedLimit = 0.5;   // 1.5707
std::string jointControlString = "arm_joint_1";

ros::Publisher pub_plan;

// Class for the recorder
class TrajectoryRecorder
{
private:

    // Action client for the joint trajectory action used to trigger the arm movement action
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_action_;

public:

    float samplingFrequency = 20.0;                             // Frequency of sampling the joint states in Hz
    float samplingPeriod = 1/samplingFrequency;
    float duration = 15;                                       // Duration of recording in second
    int numberOfPoints = (int)(samplingFrequency*duration);
    ros::Time startingTime;                                     // Starting time of the recording
    ros::Time intermediateTime;
    trajectoryRecordingStates state = waiting;                  // State of the recording process
    int pointCounter = 0;
    int activeGoal = 0;
    int activeTestGoal = 0;
    int TFSFirstPoint = 0.0;                                      // Time From Start for the first point in the trajectory
    int secondsFromRecStart = 4;

    std::vector <double> previousVelocity;
    bool removeAcceleration = false;
    bool removeVelocity = false;
    bool removePosition = false;

    std::vector <control_msgs::FollowJointTrajectoryGoal> goals;// Vector for keeping goals
    std::vector <control_msgs::FollowJointTrajectoryGoal> testGoals;

    //! Initialize the action client and wait for action server to come up
    TrajectoryRecorder() :
    trajectory_action_("/arm_1/arm_controller/follow_joint_trajectory", true)
    {
        // wait for action server to come up
        while(!trajectory_action_.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }

    //! Sends the command to start a given trajectory
    void startTrajectory()
    {
        // When to start the trajectory: 2s from now
        goals[activeGoal].trajectory.header.stamp = ros::Time::now() + ros::Duration(2.0);
        trajectory_action_.sendGoal(goals[activeGoal]);
        ROS_INFO("Sent Goal");
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
        return trajectory_action_.getState();
    }


    // Method for initializing the recorder
    void initRecorder()
    {
        state = waiting;
        pointCounter = 0;
    }

    // Method for adding a new goal
    void addNewGoal(const sensor_msgs::JointState & msg)
    {
        // reset point counter
        pointCounter = 0;
        previousVelocity.resize(numberOfJoints);

        // Create new goal
        control_msgs::FollowJointTrajectoryGoal goal;

        // We will have frequency*time waypoints in this goal trajectory
        goal.trajectory.joint_names.resize(numberOfJoints);
        goal.trajectory.points.resize(numberOfPoints);
        goal.trajectory.points[pointCounter].positions.resize(numberOfJoints);
        goal.trajectory.points[pointCounter].velocities.resize(numberOfJoints);
        goal.trajectory.points[pointCounter].accelerations.resize(numberOfJoints);

        ROS_INFO("starting recording in:");
        for (int i=0; i<secondsFromRecStart; i++)
        {
            ROS_INFO(" %d", secondsFromRecStart-i);
            ros::Duration(1.0).sleep();
        }

        // Joint names and first values
        for (int i=0; i<numberOfJoints; i++)
        {
            goal.trajectory.joint_names[i] = msg.name[i];
            goal.trajectory.points[pointCounter].positions[i] = msg.position[i];
            goal.trajectory.points[pointCounter].velocities[i] = 0.0;
            goal.trajectory.points[pointCounter].accelerations[i] = 0.0;
            //goal.trajectory.points[pointCounter].positions[i] = 0.0;
            //goal.trajectory.points[pointCounter].velocities[i] = msg.velocity[i];

            previousVelocity[i] = msg.velocity[i];
        }

        // TODO: currently it slowly goes to starting position
        goal.trajectory.points[pointCounter].time_from_start = ros::Duration(TFSFirstPoint);

        // Increment the counter, since first points were already added
        pointCounter++;

        // Get the index of the active goal and push it to the goals vector
        activeGoal = goals.size();
        goals.push_back(goal);

        // Mark the starting time
        startingTime = ros::Time::now();
        intermediateTime = ros::Time::now();

        state = recording;
        ROS_INFO("trajectoryRecorder: New goal added");
    }

    // Method for adding a new point to an active goal
    void addPointToActiveGoal(const sensor_msgs::JointState & msg)
    {
        if (pointCounter < numberOfPoints)
        {
            goals[activeGoal].trajectory.points[pointCounter].positions.resize(numberOfJoints);
            goals[activeGoal].trajectory.points[pointCounter].velocities.resize(numberOfJoints);
            goals[activeGoal].trajectory.points[pointCounter].accelerations.resize(numberOfJoints);

            for (int i=0; i<numberOfJoints; i++)
            {
                goals[activeGoal].trajectory.points[pointCounter].positions[i] = msg.position[i];
                //goals[activeGoal].trajectory.points[pointCounter].positions[i] = 0.0;
                goals[activeGoal].trajectory.points[pointCounter].velocities[i] = 0.0;
                goals[activeGoal].trajectory.points[pointCounter].accelerations[i] = 0.0;
                /*goals[activeGoal].trajectory.points[pointCounter].velocities[i] = msg.velocity[i];
                goals[activeGoal].trajectory.points[pointCounter].accelerations[i] =
                        (msg.velocity[i] - previousVelocity[i])/(ros::Time::now().toSec() - intermediateTime.toSec());
                previousVelocity[i] = msg.velocity[i];
                */
            }

            //ros::Duration diff = (ros::Time::now() - startingTime) + ros::Duration(TFSFirstPoint);
            ros::Duration diff = (ros::Time::now() - startingTime);
            goals[activeGoal].trajectory.points[pointCounter].time_from_start = diff;

            intermediateTime = ros::Time::now();
            ROS_INFO("trajectoryRecorder: Point %d added to goal %d @ dt=%f",pointCounter, activeGoal, diff.toSec() );

            pointCounter++;
        }
        else
        {
            ROS_INFO("trajectoryRecorder: goal %d filled with points",activeGoal );
            state = waiting;
        }
    }

    // Refine the trajectory
    void refineTrajectory()
    {
        if (goals.size() < 1)
        {
            ROS_ERROR("refineTrajectory: No goals in the goals vector.");
            return;
        }
        int addedPoints = 0;
        int detectedGaps = 0;
        float jointSpeedStretchFactor = 1.1;
        float timeCompensation = 0;

        ROS_INFO("refineTrajectory: Starting the interpolation process ...");

        // Create 2 new empty trajectories, one for final result and one for operations
        std::vector<trajectory_msgs::JointTrajectoryPoint> points;
        std::vector<trajectory_msgs::JointTrajectoryPoint> workingPoints;
        trajectory_msgs::JointTrajectoryPoint interpolationPoint;
        trajectory_msgs::JointTrajectoryPoint interpStartPoint;
        trajectory_msgs::JointTrajectoryPoint interpEndPoint;

        // Initialize the interpolation related points.
        interpolationPoint.positions.resize(numberOfJoints);
        interpolationPoint.velocities.resize(numberOfJoints);
        interpolationPoint.accelerations.resize(numberOfJoints);

        interpStartPoint.positions.resize(numberOfJoints);
        interpStartPoint.velocities.resize(numberOfJoints);
        interpStartPoint.accelerations.resize(numberOfJoints);

        interpEndPoint.positions.resize(numberOfJoints);
        interpEndPoint.velocities.resize(numberOfJoints);
        interpEndPoint.accelerations.resize(numberOfJoints);

        // Copy the points from current active goal to "points" vector
        for(int i=0; i<goals[activeGoal].trajectory.points.size(); i++)
        {
            points.push_back( goals[activeGoal].trajectory.points[i] );
        }
        
        // Variable for holding the shortest period between samples, initalized with an unreasonably high value.
        float shortestPeriod = 999999;

        // Find the smallest period and sections, that exceed the speed limit
        // NB! Speed limiter cannot differantiate noise from velocity jumps
        for (int i=0; i<points.size()-1; i++)
        {
            float timeDifference = points[i+1].time_from_start.toSec() - points[i].time_from_start.toSec();
            if (timeDifference < shortestPeriod)
                shortestPeriod = timeDifference;
        }

        std::cout << "shortestPeriod = " << shortestPeriod << "s, or " << 1/shortestPeriod << "Hz" << std::endl;
        
        // 
        
        int numOfInterpolationCycles = 3;
        float gapMultiplier = 2;

        // Create float vectors for storing the angular speed before and after the gap (of every joint)
        // and a vector to store the angular acceleration
        std::vector<float> vel1(numberOfJoints);
        std::vector<float> vel2(numberOfJoints);
        std::vector<float> acceleration(numberOfJoints);

        // The algorithm runs for 2 cycles in order to compsensate for speed
        for (int cycle=0; cycle<2; cycle++)
        {
            // In order to get better results, firstly start looking for larger gaps and neglect the smaller ones
            for (int k=numOfInterpolationCycles; k>0; k--)
            {
                // Inner loop that actually does the job
                for (int i=0; i<(points.size()-1); i++)
                {
                    float timeDifference = points[i+1].time_from_start.toSec() - points[i].time_from_start.toSec();
                    // std::cout << "i="<< i << ", k="<< k <<", timeDifference=" << timeDifference << std::endl;

                    // If the time difference of 2 neighboring points is bigger than (k*gapMultiplier)*shortestPeriod
                    if (timeDifference > k*gapMultiplier*shortestPeriod)
                    {
                        std::cout << "    Found a gap, that is larger than shortest period" << std::endl;
                        detectedGaps++;

                        // If the gap is between first or last two points in the trajectory, then have same velocities
                        if ( (i == 0) || (i == points.size()-2) )
                        {
                            for (int joint=0; joint<numberOfJoints; joint++)
                            {
                                vel1[joint] = (points[i+1].positions[joint] - points[i].positions[joint]) / timeDifference;
                                vel2[joint] = vel1[joint];
                            }
                        }
                        else
                        {
                            // Create variablas for storing the times (for efficiency considerations)
                            float startTimeVel1 = points[i-1].time_from_start.toSec();
                            float endTimeVel1 = points[i].time_from_start.toSec();
                            float startTimeVel2 = points[i+1].time_from_start.toSec();
                            float endTimeVel2 = points[i+2].time_from_start.toSec();

                            // Calculate velocities
                            for(int joint=0; joint<numberOfJoints; joint++)
                            {
                                vel1[joint] = (points[i].positions[joint] - points[i-1].positions[joint]) /
                                              (endTimeVel1 - startTimeVel1);
                                vel2[joint] = (points[i+2].positions[joint] - points[i+1].positions[joint]) /
                                              (endTimeVel2 - startTimeVel2);
                            }
                        }
                        // Add the starting point
                        workingPoints.push_back(points[i]);

                        // Initialize the variables that are used in the interpolation
                        int steps = int(timeDifference/shortestPeriod);
                        float dt = timeDifference/steps;
                        float startTime = points[i].time_from_start.toSec();
                        interpStartPoint = points[i];
                        interpEndPoint = points[i+1];

                        // Calculate the acceleration of each joint
                        for (int joint=0; joint<numberOfJoints; joint++)
                        {
                            acceleration[joint] = (vel2[joint] - vel1[joint]) / (timeDifference*2);
                        }

                        // Start the interpolation
                        for (int j=1; j<steps; j++)
                        {
                            std::cout << "    interp j=" << j << std::endl;
                            // Calculate the trajectory bending factor, cosine blend gives smoother transitions than linear
                            float trajectoryBlend = (1 + cos(pi*j*dt/timeDifference)) / 2;

                            // Calculate the trajectories
                            for (int joint=0; joint<numberOfJoints; joint++)
                            {
                                float traj1 = interpStartPoint.positions[joint] + vel1[joint]*j*dt + acceleration[joint]*(pow(j*dt, 2));
                                float traj2 = interpEndPoint.positions[joint] - vel2[joint]*(steps - j)*dt + acceleration[joint]*(pow((steps - j)*dt, 2));
                                interpolationPoint.positions[joint] = trajectoryBlend*traj1 + (1 - trajectoryBlend)*traj2;
                            }
                            // Calculate the timedifference and push the interpolation point into workingPoints trajectory
                            interpolationPoint.time_from_start = ros::Duration(startTime + dt*j);
                            workingPoints.push_back(interpolationPoint);
                            addedPoints++;
                        }
                    }
                    else
                    {
                        workingPoints.push_back(points[i]);
                    }
                }                        // Interpolation loop end
                // Push in the last point
                workingPoints.push_back(points[points.size() - 1]);
                points.clear();

                //Copy the points from current active goal to points vector
                for(int i=0; i<workingPoints.size(); i++)
                {
                    points.push_back( workingPoints[i] );
                }
                workingPoints.clear();

            // Gap size inrement loop end
            }

            // Apply gaussian filtering
            // float gaussKernel[5] = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};
            float gaussKernel[9] = {0.000229, 0.005977, 0.060598, 0.241732, 0.382928, 0.241732, 0.060598, 0.005977, 0.000229};
            int gaussKernelSize = 9;
            int halfKernelSize = gaussKernelSize/2;
            int sizeOfPoints = points.size();
            std::vector<trajectory_msgs::JointTrajectoryPoint> pointsGauss;

            // Add first "halfKernelSize" points to filtered vector
            for (int i=0; i<halfKernelSize; i++)
            {
                pointsGauss.push_back(points[i]);
            }

            // Filter loop
            for (int i=halfKernelSize; i<(sizeOfPoints-halfKernelSize); i++)
            {
                // Loop over joints
                for (int joint=0; joint<numberOfJoints; joint++)
                {
                    float filtered = 0;

                    // Loop over kernel
                    for (int j=0; j<gaussKernelSize; j++)
                    {
                        filtered += gaussKernel[j] * points[i - halfKernelSize + j].positions[joint];
                    }
                    interpolationPoint.positions[joint] = filtered;
                }
                interpolationPoint.time_from_start = points[i].time_from_start;
                pointsGauss.push_back(interpolationPoint);
            }

            // Add last "halfKernelSize" points to filtered vector
            for (int i=(sizeOfPoints-halfKernelSize); i<sizeOfPoints; i++)
            {
                pointsGauss.push_back(points[i]);
            }

            points.clear();


            // Copy the points from "gauss" to "points" vector
            for(int i=0; i<pointsGauss.size(); i++)
            {
                points.push_back( pointsGauss[i] );
            }

            std::cout << "speedcheck" << std::endl;
            // Find the sections that exceed the joint speed limit
            if (cycle == 0)
            {
                workingPoints.push_back(points[0]);

                for (int i=0; i<points.size()-1; i++)
                {
                    float timeDifference = points[i+1].time_from_start.toSec() - points[i].time_from_start.toSec();

                    // Find check all the joints in the same timeframe
                    float velocity = 0;
                    for (int joint=0; joint<numberOfJoints; joint++)
                    {
                        interpolationPoint.positions[joint] = points[i+1].positions[joint];
                        float tempVelocity = abs( (points[i+1].positions[joint] - points[i].positions[joint]) / timeDifference );

                        // find the highest one
                        if (tempVelocity > velocity)
                            velocity = tempVelocity;
                    }

                    // Does it exceed the speed limit?
                    if (velocity >= jointSpeedLimit)
                    {
                        ROS_ERROR("Velocity exceeded at point i=%d", i);
                        float newTimeDifference = (velocity/jointSpeedLimit) * jointSpeedStretchFactor * timeDifference;
                        timeCompensation += (newTimeDifference - timeDifference);
                    }
                    interpolationPoint.time_from_start = points[i+1].time_from_start + ros::Duration(timeCompensation);
                    workingPoints.push_back(interpolationPoint);
                }

                points.clear();
                for(int i=0; i<workingPoints.size(); i++)
                {
                    points.push_back( workingPoints[i] );
                }
                workingPoints.clear();
            }
            std::cout << "speedcheck end" << std::endl;
        // Algoithm loop end
        }

        // Create a new goal with the smoothened trajectory
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = goals[activeGoal].trajectory.joint_names;
        // goal.trajectory.points = points;
        goal.trajectory.points = points;

        activeGoal = goals.size();
        goals.push_back(goal);
        //activeTestGoal = testGoals.size();
        //testGoals.push_back(goal);

        ROS_INFO("refineTrajectory: Finished the interpolation process. Detected %d gaps, added %d new points", detectedGaps, addedPoints);
    }

    // Method for printing the trajectory to the terminal window
    void printTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {
        ROS_INFO("Printing the trajectory of the active goal.");

        // Print out the names of the joints
        std::cout << std::endl;
        for (int k=0; k<numberOfJoints; k++)
        {
            std::cout << " [" << goal.trajectory.joint_names[k] << "] ";
        }
        std::cout << std::endl;

        // Print out the recorded values
        for (int i=0; i<goal.trajectory.points.size(); i++)
        {
            float timeDifference = 0;

            // Print positions
            std::cout << i << ": t=" << goal.trajectory.points[i].time_from_start.toSec();
            if (i > 0)
            {
                timeDifference = goal.trajectory.points[i].time_from_start.toSec() - goal.trajectory.points[i-1].time_from_start.toSec();
                std::cout << "  dt=" << timeDifference << " s  freq=" << 1/timeDifference << " Hz";
            }
            std::cout << std::endl;

            std::cout << "  POS: ";
            for (int j=0; j<numberOfJoints; j++)
            {
                std::cout << " [" << goal.trajectory.points[i].positions[j] << "] ";
            }
            std::cout << std::endl;

            // Print velocities
            std::cout << "  VEL: ";
            if (i > 0)
            {
                for (int j=0; j<numberOfJoints; j++)
                {
                    float vel = goal.trajectory.points[i].positions[j] - goal.trajectory.points[i-1].positions[j];
                    vel /= timeDifference;
                    std::cout << " [" << vel << "] ";
                    if (vel >= 1.5707)
                        ROS_ERROR("joint %d exceedes the speedlimit", j);
                }
            }
            std::cout << std::endl;
/*
            // Print accelerations
            std::cout << "  ACC: ";
            for (int j=0; j<numberOfJoints; j++)
            {
                std::cout << " [" << goal.trajectory.points[i].accelerations[j] << "] ";
            }
            std::cout << std::endl;
*/
        }
        std::cout << std::endl;
    }

    // Method for publishing moveit_msgs/DisplayTrajectory messages for visualizing the robot in rviz
    void displayTrajectory( trajectory_msgs::JointTrajectory traj)
    {
        // Create trajectory message and initialize it
        moveit_msgs::RobotTrajectory robot_trajectory;
        robot_trajectory.joint_trajectory = traj;

        // Create initial position message
        moveit_msgs::RobotState robot_state_msg;
        robot_state_msg.joint_state.name.resize(numberOfJoints);
        robot_state_msg.joint_state.position.resize(numberOfJoints);

        for (int i=0; i<numberOfJoints; i++)
        {
            robot_state_msg.joint_state.name[i] = traj.joint_names[i];
            robot_state_msg.joint_state.position[i] = traj.points[0].positions[i];
        }

        // Do some sh*t
        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.model_id = "youbot";
        display_trajectory.trajectory_start = robot_state_msg;
        display_trajectory.trajectory.clear();
        display_trajectory.trajectory.push_back(robot_trajectory);

        ROS_INFO("Displaying the trajectory in Rviz.");
        pub_plan.publish(display_trajectory);
    }

    void trajectoryCheck(control_msgs::FollowJointTrajectoryGoal goal)
    {
        //
        for (int i=1; i<goal.trajectory.points.size(); i++)
        {
            float timeDifference = goal.trajectory.points[i].time_from_start.toSec() - goal.trajectory.points[i-1].time_from_start.toSec();
            float time = goal.trajectory.points[i].time_from_start.toSec();
            std::cout << "i="<< i << ", time=" << time << ", diff=" << timeDifference << std::endl;
        }
    }

    // Callback function for enabling or disabling the motors
    void motorSwitchCallback(bool Mstate)
    {
        if (Mstate == true)
        {
            ROS_INFO("Enabling the motors ...");
            //kill motors
            std_srvs::Empty empty;
            ros::service::call("arm_1/switchOnMotors", empty);
            ros::service::call("base/switchOnMotors", empty);
        }

        else if (Mstate == false)
        {
            ROS_INFO("Disabling the motors ...");
            //kill motors
            std_srvs::Empty empty;
            ros::service::call("arm_1/switchOffMotors", empty);
            ros::service::call("base/switchOffMotors", empty);
        }
    }

    // Callback for processing joint states
    void jointCallback(const sensor_msgs::JointState & msg)
    {
        if (msg.name[0].compare(jointControlString) == 0)
        {
            switch(state)
            {
                // Case starting
                case waiting:
                {
                    // TODO
                }
                break;

                // Cas starting
                case starting:
                {
                    addNewGoal(msg);
                }
                break;

                // Case recording
                case recording:
                {
                    double timeDiffSec = ros::Time::now().toSec() - intermediateTime.toSec();
                    if ( timeDiffSec >= samplingPeriod )
                    {
                        addPointToActiveGoal(msg);
                    }
                }
                break;
            }
        }
    }

    void trajectoryCallback(control_msgs::FollowJointTrajectoryActionGoal msg)
    {
        ROS_INFO("Recieved a goal! Adding it to goals list.");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal = msg.goal;

        // Make some stuff ziro
        if (removeAcceleration)
        {
            for (int i=0; i<goal.trajectory.points.size(); i++)
            {
                for (int j=0; j<numberOfJoints; j++)
                {
                    goal.trajectory.points[i].accelerations[j] = 0.0;
                }
            }
        }

        // Make some stuff ziro
        if (removeVelocity)
        {
            for (int i=0; i<goal.trajectory.points.size(); i++)
            {
                for (int j=0; j<numberOfJoints; j++)
                {
                    goal.trajectory.points[i].velocities[j] = 0.0;
                }
            }
        }

        // Make some stuff ziro
        if (removePosition)
        {
            for (int i=0; i<goal.trajectory.points.size(); i++)
            {
                for (int j=0; j<numberOfJoints; j++)
                {
                    goal.trajectory.points[i].positions[j] = 0.0;
                }
            }
        }
        activeGoal = goals.size();
        goals.push_back(goal);
    }

    // Callback for processing keyboard events
    void keyboardCallback(keyboard_reader::Key kbCommand)
    {
        if (kbCommand.key_pressed == true)
        {
            // Record joint states: "q" key
            if (kbCommand.key_code == 0x0010 )
            {
                if (state == waiting)
                {
                    ROS_INFO("Starting to record the trajectory ...");
                    state = starting;
                }
            }

            // eXecute the recorded state (move): "z" key
            else if (kbCommand.key_code == 0x002c)
            {
                ROS_INFO("Calling trajectory execution ...");
                startTrajectory();
            }

            // Print the trajectory of the active goal: "w" key
            else if (kbCommand.key_code == 0x0011)
            {
                printTrajectory(goals[activeGoal]);
            }
            // Print the trajectory of the active testGoal: "2" key
            else if (kbCommand.key_code == 0x0003)
            {
                printTrajectory(testGoals[activeTestGoal]);
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
            // Publish active goal to displayTrajectory: "t" key
            else if (kbCommand.key_code == 0x0014)
            {
                displayTrajectory(goals[activeGoal].trajectory);
            }
            // Publish interpolated goal to displayTrajectory: "i" key
            else if (kbCommand.key_code == 0x0017)
            {
                displayTrajectory(testGoals[activeTestGoal].trajectory);
            }

            // Publish interpolated goal to displayTrajectory: "1" key
            else if (kbCommand.key_code == 0x0002)
            {
                trajectoryCheck(testGoals[activeTestGoal]);
            }

            // Publish interpolated goal to displayTrajectory: "f" key
            else if (kbCommand.key_code == 0x0021)
            {
                refineTrajectory();
            }
            // remove acceleration: "a" key
            else if (kbCommand.key_code == 0x001e)
            {
                if (removeAcceleration)
                {
                    ROS_INFO("removeAcceleration = FALSE");
                    removeAcceleration = false;
                }
                else
                {
                    ROS_INFO("removeAcceleration = TRUE");
                    removeAcceleration = true;
                }
            }
            // Toggle velocity: "v" key
            else if (kbCommand.key_code == 0x002f)
            {
                if (removeVelocity)
                {
                    ROS_INFO("removeVelocity = FALSE");
                    removeVelocity = false;
                }
                else
                {
                    ROS_INFO("removeVelocity = TRUE");
                    removeVelocity = true;
                }
            }

            // Toggle position: "l" key
            else if (kbCommand.key_code == 0x0026)
            {
                if (removePosition)
                {
                    ROS_INFO("removePosition = FALSE");
                    removePosition = false;
                }
                else
                {
                    ROS_INFO("removePosition = TRUE");
                    removePosition = true;
                }
            }
        }
    }

};




int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "trajectory_recorder");
    ros::NodeHandle n;

    // Publisher for publishing the DisplayTrajectory
    pub_plan = n.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 100);

    // Create the instance of trajectory recorder
    TrajectoryRecorder recorderT;

    // Listen to joint states and keyboard events
    ros::Subscriber sub = n.subscribe("/joint_states", 10, &TrajectoryRecorder::jointCallback, &recorderT);
    ros::Subscriber sub_kb_event = n.subscribe<keyboard_reader::Key>("keyboard", 1, &TrajectoryRecorder::keyboardCallback, &recorderT);
    ros::Subscriber sub_traj_goal = n.subscribe("/arm_1/arm_controller/follow_joint_trajectory/goalBag", 1, &TrajectoryRecorder::trajectoryCallback, &recorderT);

    ROS_INFO("STILL ALIVE ...");
    /*
    ROS_INFO("Simple Trajectory");
    RobotArm arm;
    // Start the trajectory
    arm.startTrajectory(arm.armExtensionTrajectory());
    // Wait for trajectory completion
    while(!arm.getState().isDone() && ros::ok())
    {
    usleep(50000);
    }
*/
    ros::spin();
    ROS_INFO("STILL not ALIVE ...");
    return 0;
}
