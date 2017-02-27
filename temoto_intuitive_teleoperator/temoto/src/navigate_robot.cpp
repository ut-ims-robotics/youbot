// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file navigate_robot.cpp
 * 
 *  @brief ROS server that acts as an interface for ROS navigation stack.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/navigate_robot.h"

/** This method is executed when temoto/navigate_robot_service service is called.
 *  It updates navigation_goal_stamped_ based on the client's request and sets the new_navgoal_requested_ flag to let main() know that moving of the robot has been requested.
 *  When ABORT has been requested, it sets stop_navigation_ TRUE.
 *  @param req temoto::Goal service request.
 *  @param res temoto::Goal service response.
 *  @return always true.
 */
bool NavigateRobotInterface::serviceUpdate(temoto::Goal::Request  &req,
					   temoto::Goal::Response &res)
{
//  ROS_INFO("[temoto/navigate_robot] New service update requested.");
  
  // check first if abort navigation has been requested.
  if (req.action_type == temoto::GoalRequest::ABORT)
  {
    stop_navigation_ = true;
    return true;
  }
  // get goal pose and set new_navgoal_requested_ TRUE
  else
  {
    navigation_goal_stamped_ = req.goal;
    new_navgoal_requested_ = true;
    return true;
  }
} // end serviceUpdate

/** Plans and executes the navigation of the robot to the pose stored in navigation_goal_stamped_. This function is blocking. */
void NavigateRobotInterface::requestNavigation()
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose = navigation_goal_stamped_;
  
  ROS_INFO("[temoto/navigate_robot] Sending navigation goal");
  move_base_aclient_.sendGoal(goal);

  move_base_aclient_.waitForResult();

  if(move_base_aclient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("[temoto/navigate_robot] Navigation goal achieved");
  else
    ROS_INFO("[temoto/navigate_robot] Navigation goal failed for some reason");
  return;
}


/** Send navigation_goal_stamped_ to move_base action server. Non-blocking. */
void NavigateRobotInterface::sendNavigationGoal()
{
  move_base_msgs::MoveBaseGoal mb_goal;

  mb_goal.target_pose = navigation_goal_stamped_;
  
  ROS_INFO("[temoto/navigate_robot] Sending navigation goal");
  move_base_aclient_.sendGoal(mb_goal);

  return;
}

/** Sends a action request to cancel goal. */
void NavigateRobotInterface::abortNavigation()
{
  move_base_aclient_.cancelGoal();
  return;
}

/** Asks for status from action server. */
void NavigateRobotInterface::checkNavigationStatus()
{
  move_base_aclient_.getState();
  return;
}

/** Main method. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigate_robot");
  ros::NodeHandle n;
  ros::Rate node_rate(1000);
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

  // Instance of NavigateRobotInterface
  NavigateRobotInterface navigateIF("move_base");

  // Wait for the action server to come up
  while( !navigateIF.move_base_aclient_.waitForServer( ros::Duration(5.0) ) )
  {
    ROS_INFO("[temoto/navigate_robot] Waiting for the move_base action server to come up");
  }
  
  // Set up service for navigate_robot_srv; if there's a service request, call serviceUpdate() function
  ros::ServiceServer service = n.advertiseService("temoto/navigate_robot_srv", &NavigateRobotInterface::serviceUpdate, &navigateIF);
  
  while ( ros::ok() )
  {
    if (navigateIF.stop_navigation_)
    {
      navigateIF.abortNavigation();
      navigateIF.stop_navigation_ = false;
    }
    // if new navigation goal has been requested
    else if (navigateIF.new_navgoal_requested_)
    {
      // navigate the robot to requested goal
      navigateIF.sendNavigationGoal();
//       navigateIF.requestNavigation();
      navigateIF.new_navgoal_requested_ = false;
    }
      
    ros::spinOnce();
    node_rate.sleep();
  } // while



  return 0;
}