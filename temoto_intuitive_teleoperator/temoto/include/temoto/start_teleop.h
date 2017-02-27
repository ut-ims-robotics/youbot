// Copyright (c) 2015-2016, The University of Texas at Austin
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

/** @file start_teleop.h
 * 
 *  @brief Central node for TEMOTO teleoperator. Subscribes to relevant messages, calls move and navigation interfaces, and publishes system status.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

// temoto includes
#include "temoto/temoto_common.h"
#include "leap_motion_controller/Set.h"
#include "griffin_powermate/PowermateEvent.h"

// Other includes
#include <cstdlib>
#include <iostream>
#include <algorithm>    // std::reverse

#ifndef START_TELEOP_H
#define START_TELEOP_H

class Teleoperator
{
public:
  // Constructor
  
  Teleoperator(std::string primary_hand, bool navigate, bool manipulate);
  
  // Callout functions
  
  void callRobotMotionInterface(uint8_t action_type);
  
  void callRobotMotionInterfaceWithNamedTarget(uint8_t action_type, std::string named_target);
  
  void computeCartesian(std::string frame_id);

  // Helper functions
   
  std::vector<geometry_msgs::Pose> wayposesInFixedFrame(std::vector<geometry_msgs::Pose> wayposes_leapmotion);
  
  geometry_msgs::Quaternion extractOnlyPitch(geometry_msgs::Quaternion msg);
  
  geometry_msgs::Quaternion oneEightyAroundOperatorUp(geometry_msgs::Quaternion operator_input_quaternion_msg);
  
  temoto::Status getStatus();
  
  //Callback functions
  
  void processLeap(leap_motion_controller::Set leap_data);			// TODO rename to more general case, e.g. processHumanInput
  
  void processPowermate(griffin_powermate::PowermateEvent powermate);		// TODO rename to more general case, e.g. processScaleFactor
  
  void updateEndEffectorPose(geometry_msgs::PoseStamped end_effector_pose);
  
  void processVoiceCommand(temoto::Command voice_command);
  
  // Public members
  ros::ServiceClient move_robot_client_;		///< Service client for temoto/move_robot_service is global.
  ros::ServiceClient navigate_robot_client_;		///< Service client for temoto/navigate_robot_srv is global.
  ros::ServiceClient tf_change_client_;			///< Service client for requesting changes of control mode, i.e., change of orientation for leap_motion frame.
  
//   NavigateRobotInterface navigate_robot_ifclient_;
  
  // Temporary flags
  std_msgs::Bool okay_robot_execute;

private:
  /// Local TransformListener for transforming poses
  tf::TransformListener transform_listener_;

  /// Scaling factor. Normalized to 1.
  double scale_by_;
  
  /// Amplification of input hand motion. (Scaling factor scales the amplification.)
  int8_t AMP_HAND_MOTION_;
  
  /// Offsett zero position of the Leap Motion Controller.
  const double OFFSET_X_ = 0;
  const double OFFSET_Y_ = 0.2;		// Height of zero
  const double OFFSET_Z_ = 0;

  /// Latest pose value received for the end effector.
  geometry_msgs::PoseStamped current_pose_;
  
  /// Properly scaled target pose in reference to the hand pose input (e.g. leap_motion) frame.
  geometry_msgs::PoseStamped live_pose_;
  
  /// Vector of desired wayposes for a Cartesian move of the end-effector. Specified in the hand pose input (e.g. leap_motion) frame.
  std::vector<geometry_msgs::Pose> wayposes_;
  
  /// Vector of desired wayposes for a Cartesian move of the end-effector. Specified in a fixed (e.g. base_link) frame. Current pose included in the beginning.
  std::vector<geometry_msgs::Pose> wayposes_fixed_in_baselink_;

  // ~*~ VARIABLES DESCRIBING THE STATE ~*~
  // NATURAL control: robot and human are oriented the same way, i.e., the first person perspective
  // INVERTED control: the human operator is facing the robot so that left and right are inverted.
  bool using_natural_control_;		///< Mode of intepration for hand motion: 'true' - natural, i.e., human and robot arms are the same; 'false' - inverted.
  bool orientation_locked_;		///< Hand orientation info is to be ignored if TRUE.
  bool position_limited_;		///< Hand position is restricted to a specific direction/plane if TRUE.
  bool position_fwd_only_;		///< TRUE when hand position is restricted to back and forward motion. Is only relevant when position_limited is 'true'.
  bool secondary_hand_before_;		///< Presence of secondary hand during the previous iteration of Leap Motion's callback processLeap(..).
  bool navigate_to_goal_;		///< TRUE: interpret live_pose_ as 2D navigation goal; FALSE: live_pose_ is the motion planning target for robot EEF.
  bool primary_hand_is_left_;		///< TRUE unless user specified right hand as the primary hand.
  uint8_t control_state_;		///< 1 -> manipulate only; 2 -> navigate only; 3 -> manipulate&navigate
};
#endif