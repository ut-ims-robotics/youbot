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
 *  @brief Interface for ROS navigation stack.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

// temoto includes
#include "temoto/temoto_common.h"

#ifndef NAVIGATE_ROBOT_H
#define NAVIGATE_ROBOT_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigateRobotInterface {
public:
  /** Constructor */
  NavigateRobotInterface(std::string mbase_name) :	// I use something called initializer list here, Alex taught me that
	  move_base_aclient_(mbase_name, "true")	// Initialize the action client and tell it that we want to spin a thread by default
  {
    new_navgoal_requested_ = false;
    stop_navigation_ = false;
  };
   
  /** Simple Action Client for move_base */
  MoveBaseClient move_base_aclient_;
  
  /** Callback function for navigation service request */
  bool serviceUpdate(temoto::Goal::Request  &req, temoto::Goal::Response &res);
  
  // Functions that actually communicates with move_base action server
  
  /** Plans and executes the navigation of the robot to the pose stored in navigation_goal_stamped_. This function is blocking. */
  void requestNavigation();
  
  /** Sends navigation_goal_stamped_ to move_base action server. Non-blocking. */
  void sendNavigationGoal();

  /** Sends a action request to cancel goal. */
  void abortNavigation();

  /** Asks for status from action server. */
  void checkNavigationStatus();


  /** Target navigation pose for the robot */
  geometry_msgs::PoseStamped navigation_goal_stamped_;
   
  // Public variables describing the state of MoveRobotInterface
  
  /** If new move has been requested by a client, it is set to TRUE; after calling sendNavigationGoal(), it is set to FALSE. */
  bool new_navgoal_requested_;
  
  /** TRUE when 'cancel goal' has been requested; FALSE otherwise */
  bool stop_navigation_;
};

#endif