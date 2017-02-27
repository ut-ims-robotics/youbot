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

/** @file start_teleop.cpp
 * 
 *  @brief Central node for TEMOTO teleoperator. Subscribes to relevant messages, calls move and navigation interfaces, and publishes system status.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/start_teleop.h"

/** Constructor for Teleoperator.
 *  @param primary_hand a std::string that determines the primary hand used during teleoperation. Unless specified as "right", primary hand will be left.
 *  @param navigate enables teleoperation for navigation interface.
 *  @param manipulate enables teleoperation for move interface.
 */
Teleoperator::Teleoperator(std::string primary_hand, bool navigate, bool manipulate)
{
  scale_by_ = 1;
  using_natural_control_ = true;	// always start in natural control perspective
  orientation_locked_ = false;
  position_limited_ = true;
  position_fwd_only_ = false;
  secondary_hand_before_ = false;
    
  // Setting up control_state, i.e., whether teleoperator is controlling navigation, manipulation, or both.
  if (manipulate && navigate)
  {
    control_state_ = 3;
    navigate_to_goal_ = true;		// if navigation AND manipulation are enable, start out in navigation mode.
    AMP_HAND_MOTION_ = 100;		// 100 for navigation
  }
  else if (navigate && !manipulate)
  {
    control_state_ = 2;
    navigate_to_goal_ = true;		// if only navigation is enabled, navigate_to_goal_ is TRUE
    AMP_HAND_MOTION_ = 100;		// 100 for navigation
  }
  else if (manipulate && !navigate)
  {
    control_state_ = 1;
    navigate_to_goal_ = false;		// if only manipulation is enabled, navigate_to_goal_ is FALSE
    AMP_HAND_MOTION_ = 10;		// 10 for manipulation
  }
  else
  {
    ROS_WARN("[start_teleop/Teleoperator] Control state not specified.");
  }

  // Set up primary hand
  if (primary_hand == "right") {
    primary_hand_is_left_ = false;
  }
  else
  {
    primary_hand_is_left_ = true;
  }
  
  okay_robot_execute.data = false;
}

/** Function that actually makes the service call to appropriate robot motion intefrace.
 *  Currently, there are MoveRobotInterface and NavigateRobotInterface.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03). 
 */
// void Teleoperator::callPlanAndMove(uint8_t action_type)
void Teleoperator::callRobotMotionInterface(uint8_t action_type)
{
  // Create a service request
  temoto::Goal motion;	
  motion.request.goal = live_pose_;		// set live_pose_ as the requested pose for the motion planner
  motion.request.action_type = action_type;	// set action_type

  // =================================================
  // === Calling NavigateRobotInterface ==============
  // =================================================
  if (navigate_to_goal_) // if in NAVIGATION mode
  {
    // If operator requested ABORT
    if (action_type == temoto::GoalRequest::ABORT)
    {
      ROS_INFO("[start_teleop/callRobotMotionInterface] Requesting robot to stop navigation.");
      // make a service request to stop the robot
      if ( navigate_robot_client_.call( motion ) )
      {
	ROS_INFO("[start_teleop/callRobotMotionInterface] Successfully called temoto/navigate_robot_srv");
      }
      else
      {
	ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call temoto/navigate_robot_srv");
      }
      return;
    } // if (action_type == temoto::GoalRequest::ABORT)
    // If operator requested the robot to move to a goal pose
    else if (action_type == temoto::GoalRequest::GO)
    {  
      // ------------------------- DEBUG START {
      double lm_roll, lm_pitch, lm_yaw;
      ROS_INFO("[start_teleop/callRobotMotionInterface] Target QUAT in leap_motion: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
	      motion.request.goal.pose.orientation.x, motion.request.goal.pose.orientation.y, motion.request.goal.pose.orientation.z, motion.request.goal.pose.orientation.w);
      tf::Quaternion quat_leap_motion;
      tf::quaternionMsgToTF(motion.request.goal.pose.orientation, quat_leap_motion);
      tf::Matrix3x3(quat_leap_motion).getRPY(lm_roll, lm_pitch, lm_yaw);
      ROS_INFO("[start_teleop/callRobotMotionInterface] Target RPY in leap_motion: ROLL=%.3f, PITCH=%.3f, YAW=%.3f", lm_roll, lm_pitch, lm_yaw);
      // } ----------------------- END DEBUG
      
      // Translate leap_motion pose to base_link
      geometry_msgs::PoseStamped goal_in_baselink;
      // live_pose_ is given in leap_motion frame and shall be transformed into base_link
      transform_listener_.transformPose("base_link", motion.request.goal, goal_in_baselink);
    
      // ------------------------- DEBUG START {
      double bl_roll, bl_pitch, bl_yaw;
      ROS_INFO("[start_teleop/callRobotMotionInterface] Target QUAT in base_link: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
	      goal_in_baselink.pose.orientation.x, goal_in_baselink.pose.orientation.y, goal_in_baselink.pose.orientation.z, goal_in_baselink.pose.orientation.w);

      // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
      tf::Quaternion quat_base_link;
      tf::quaternionMsgToTF(goal_in_baselink.pose.orientation, quat_base_link);

      // the tf::Quaternion has a method to acess roll pitch and yaw
      tf::Matrix3x3(quat_base_link).getRPY(bl_roll, bl_pitch, bl_yaw);  
      ROS_INFO("[start_teleop/callRobotMotionInterface] Target RPY in base_link Roll=%.3f, Pitch=%.3f, Yaw=%.3f", bl_roll, bl_pitch, bl_yaw);
      // } ----------------------- END DEBUG 
      
      // TODO Figure it out!!
      // I still don't understand why the transformPose() is not doing this but ...
      // ... I set hand pitch from leap_motion frame as the yaw in base_link frame,
      // i.e., set rotation around UP in leap_motion as rotation around UP in base_link.
      ROS_INFO("[start_teleop/callRobotMotionInterface] Setting rotation around UP directly");
      quat_base_link.setRPY(0, 0, lm_pitch);
      quat_base_link.normalize();
      tf::quaternionTFToMsg(quat_base_link, goal_in_baselink.pose.orientation);
      ROS_INFO("[start_teleop/callRobotMotionInterface] Target QUAT in base_link: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
	      goal_in_baselink.pose.orientation.x, goal_in_baselink.pose.orientation.y, goal_in_baselink.pose.orientation.z, goal_in_baselink.pose.orientation.w);

      // Set goal_in_baselink as the target goal
      motion.request.goal = goal_in_baselink;
      
      // make a service request to navigate_robot_srv
      if ( navigate_robot_client_.call( motion ) )
      {
	ROS_INFO("[start_teleop/callRobotMotionInterface] Successfully called temoto/navigate_robot_srv");
      }
      else
      {
	ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call temoto/navigate_robot_srv");
      }
      return;
    } // else if (action_type == temoto::GoalRequest::GO)
  } // if (navigate_to_goal_)
  // =================================================
  // === Calling MoveRobotInterface ==================
  // =================================================
  else if ( !navigate_to_goal_ )						// If Teleoperator is in MANIPULATION (MoveIt!) mode
  {  
    // Adjust orientation for inverted control mode, i.e. translate leap_motion to leap_motion_on_robot
    if ( !using_natural_control_)
    {
      // rotate orientation 180 around UP-vector
      motion.request.goal.pose.orientation = oneEightyAroundOperatorUp( motion.request.goal.pose.orientation ) ;
    }
       
    // Call temoto/move_robot_service
    if ( move_robot_client_.call( motion ) )
    {
      ROS_INFO("[start_teleop/callRobotMotionInterface] Successfully called temoto/move_robot_service");
    }
    else
    {
      ROS_ERROR("[start_teleop/callRobotMotionInterface] Failed to call temoto/move_robot_service");
    }
    return;
  }
  // =================================================
  // === Unexpected case =============================
  // =================================================
  else
  {
    ROS_INFO("[start_teleop/callRobotMotionInterface] Request unavailable for current list of robot interfaces.");
  }
  return;
} // end callRobotMotionInterface

/** Function that actually makes the service call to /temoto/move_robot_service.
 *  @param action_type determines what is requested from MoveGroup, i.e. PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03).
 *  @param named_target uses this named target for target pose.
 */
// void Teleoperator::callPlanAndMoveNamedTarget(uint8_t action_type, std::string named_target)
void Teleoperator::callRobotMotionInterfaceWithNamedTarget(uint8_t action_type, std::string named_target)
{
  if ( !navigate_to_goal_ )			// currenly implemented for only MoveRobotInterface
  {
    temoto::Goal motion;			// create a service request
    motion.request.action_type = action_type;	// set action_type
    motion.request.named_target = named_target;	// set named_target as the goal
    
    // call for the service to move robot group
    if ( move_robot_client_.call( motion ) )
    {
      ROS_INFO("Successfully called temoto/move_robot_service");
    }
    else
    {
      ROS_ERROR("Failed to call temoto/move_robot_service");
    }
  }
  else
  {
    ROS_INFO("[start_teleop/callRobotMotionInterfaceWithNamedTarget] Not available for NavigateRobotInterface.");
  }
  return;
} // end callRobotMotionInterfaceWithNamedTarget

/** Function that makes the service call to /temoto/move_robot service.
 *  It requests computation/planning of cartesian path based on existing waypoints.
 *  If a complete path cannot be calculated, it removes the last waypose and replans the cartesian path.
 * 
 */
void Teleoperator::computeCartesian(std::string frame_id)
{
  if (!navigate_to_goal_)			// currenly implemented for only MoveRobotInterface
  {
    temoto::Goal move;				// create a service request
    move.request.action_type = 0x04;		// set action_type
    move.request.cartesian_wayposes = wayposes_;
    move.request.cartesian_frame = frame_id;
    if (move_robot_client_.call(move))		// call for the service to move robot group
    {
      ROS_INFO("[start_teleop] Successfully called temoto/move_robot_service for Cartesian move. Fraction of computed path is %f", move.response.cartesian_fraction);
      if (move.response.cartesian_fraction < 1 && !wayposes_.empty())
      {
	ROS_INFO("[start_teleop] Failed to compute the entire Cartesian path. Removing the last waypose and retrying...");
	wayposes_.pop_back();
	computeCartesian(frame_id);		// going recursive
      }
      wayposes_fixed_in_baselink_ = wayposesInFixedFrame(wayposes_);
    }
    else
    {
      ROS_ERROR("[start_teleop] Failed to call temoto/move_robot_service for Cartesian move.");
    }
  }
  else
  {
    ROS_INFO("[start_teleop/computeCartesian] Not available for NavigateRobotInterface.");
  }
  
  return;
}

/** Returns a vector of wayposes that have been tranformed from "leap_motion" frame to "base_link".
 * 
 *  @param wayposes_leapmotion vector of wayposes defined in "leap_motion" frame.
 *  @param tf_listener a TransformListener that has been around long enough to know a transform from "leap_motion" to "base_link".
 *  @return a vector where all the poses are defined in "base_link" and current_pose_ is inserted as the first element.
 */
std::vector<geometry_msgs::Pose> Teleoperator::wayposesInFixedFrame(std::vector<geometry_msgs::Pose> wayposes_leapmotion)
{
  std::vector<geometry_msgs::Pose> wayposes_baselink;
  wayposes_baselink.push_back(current_pose_.pose);			// Start the vector with current end-effector pose
  std::reverse(wayposes_leapmotion.begin(),wayposes_leapmotion.end());	// reversed the order of elements in wayposes_leapmotion
  geometry_msgs::PoseStamped stamped_pose_msg_lm, stamped_pose_msg_bl;	// temp stamped poses for leap_motion and base_link
  stamped_pose_msg_lm.header.frame_id = "leap_motion";			// stamp leap_motion poses with appropriate data
  stamped_pose_msg_lm.header.stamp = ros::Time(0);			// ros::Time(0) is different from ros::Time::now() and is better in this case, idky
  while ( !wayposes_leapmotion.empty() )
  {
    stamped_pose_msg_lm.pose = wayposes_leapmotion.back();		// access the last element of reversed wayposes_
    wayposes_leapmotion.pop_back();					// delete the last element
    tf::Stamped<tf::Pose> tf_pose_lm;					// tf equivalent for PoseStamped
    tf::Stamped<tf::Pose> tf_pose_bl;
    tf::poseStampedMsgToTF(stamped_pose_msg_lm, tf_pose_lm);		// convert geometry_msgs/PoseStamped to TF Stamped<Pose>
    transform_listener_.transformPose("base_link", tf_pose_lm, tf_pose_bl);	// tranform pose to base_link frame
    tf::poseStampedTFToMsg(tf_pose_bl, stamped_pose_msg_bl);		// convert TF Stamped<Pose> to geometry_msgs/PoseStamped 
    wayposes_baselink.push_back(stamped_pose_msg_bl.pose);		// push to the end of return vector
  }
  return wayposes_baselink;
}

/** Alters a quaternion so that the pitch is preserved while roll and yaw are set to zero.
 * 
 *  @param quaternion_msg incoming quaternion as a geometry_msgs.
 *  @return normalized quaternion as a geometry_msgs that contains only pitch information.
 */
geometry_msgs::Quaternion Teleoperator::extractOnlyPitch(geometry_msgs::Quaternion quaternion_msg)
{
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(quaternion_msg, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  
  // modify quaternion by setting roll and yaw to zero
//   ROS_INFO("[start_teleop/extractOnlyPitch] PITCH: %.2f", pitch);
  quat.setRPY(0, pitch, 0);
  quat.normalize();
  
  // the modified quaternion is converted to a geometry_msgs::Quaternion
  geometry_msgs::Quaternion outbound_msg;
  tf::quaternionTFToMsg(quat, outbound_msg);
  return outbound_msg;
}

/** Callback function for /leapmotion_general subscriber.
 *  It updates target pose based on latest left palm pose (any scaling and/or relevant limitations are also being applied).
 *  Presence of the right hand is used to lock and unlock forward motion.
 *  KEY_TAP gesture detection is currenly unimplemented.
 *  @param leap_data temoto::Leapmsg published by leap_motion node
 */
void Teleoperator::processLeap(leap_motion_controller::Set leap_data)
{
  // First, set up primary and secondary hand.
  geometry_msgs::Pose primary_hand;
  geometry_msgs::Pose secondary_hand;
  bool primary_hand_is_present;
  bool secondary_hand_is_present;
  if ( primary_hand_is_left_ )
  {
    // copy left hand data to primary_hand and right hand data to secondary_hand
    primary_hand = leap_data.left_hand.palm_pose.pose;
    secondary_hand = leap_data.right_hand.palm_pose.pose;
    primary_hand_is_present = leap_data.left_hand.is_present;
    secondary_hand_is_present = leap_data.right_hand.is_present;
  }
  else
  {
    // copy right hand data to primary_hand and left hand data to secondary_hand
    primary_hand = leap_data.right_hand.palm_pose.pose;
    secondary_hand = leap_data.left_hand.palm_pose.pose;
    primary_hand_is_present = leap_data.right_hand.is_present;
    secondary_hand_is_present = leap_data.left_hand.is_present;
  }
  
  // If position data is to be limited AND secondary hand is detected AND secondary hand wasn't there before, toggle position_fwd_only_.
  if (position_limited_ && secondary_hand_is_present && !secondary_hand_before_)
  {
    (position_fwd_only_) ? position_fwd_only_ = false : position_fwd_only_ = true;	// toggles position_fwd_only value between TRUE and FALSE
    secondary_hand_before_ = true;							// sets secondary_hand_before_ TRUE;
    ROS_INFO("SECONDARY HAND DETECTED, position_fwd_only_ is now %d", position_fwd_only_);
  }
  else if (secondary_hand_is_present == false) secondary_hand_before_ = false;		// if secondary hand is not detected, set secondary_hand_before_ FALSE;
  

  // Leap Motion Controller uses different coordinate orientation from the ROS standard for SIA5:
  //				SIA5	LEAP
  //		forward 	x	z
  //		right		y	x
  //		down		z	y

  // *** Following calculations are done in "leap_motion" frame, i.e. incoming leap_data pose is relative to leap_motion frame.

  // Working variable for potential target pole calculated from the hand pose coming from Leap Motion Controller (i.e. stamped to leap_motion frame).
  geometry_msgs::PoseStamped scaled_pose;
  // Header is copied without a change.
  scaled_pose.header = leap_data.header;
  // Reads the position of primary palm in meters, amplifies to translate default motion; scales to dynamically adjust range; offsets for better usability.
  scaled_pose.pose.position.x = scale_by_*AMP_HAND_MOTION_*(primary_hand.position.x - OFFSET_X_);
  scaled_pose.pose.position.y = scale_by_*AMP_HAND_MOTION_*(primary_hand.position.y - OFFSET_Y_);
  scaled_pose.pose.position.z = scale_by_*AMP_HAND_MOTION_*(primary_hand.position.z - OFFSET_Z_);
  
  // Orientation of primary palm is copied unaltered, i.e., is not scaled
  scaled_pose.pose.orientation = primary_hand.orientation;
  
  // Applying relevant limitations to direction and/or orientation
  if (navigate_to_goal_)				// if in navigation mode, UP-DOWN motion of the hand is to be ignored
  {
    scaled_pose.pose.position.y = 0;
    // preserve only pitch (rotation around UP vector) of hand orientation
    scaled_pose.pose.orientation = extractOnlyPitch( scaled_pose.pose.orientation );
  }
  else if (position_limited_ && position_fwd_only_) 	// if position is limited and position_fwd_only_ is true
  {
    scaled_pose.pose.position.x = 0;
    scaled_pose.pose.position.y = 0;
  }
  else if (position_limited_ && !position_fwd_only_)	// if position is limited and position_fwd_only_ is false, i.e. consider only sideways position change
  {
    scaled_pose.pose.position.z = 0;
  }

  if (orientation_locked_)				// if palm orientation is to be ignored 
  {
    // Overwrite orientation with identity quaternion.
    scaled_pose.pose.orientation.w = 1;
    scaled_pose.pose.orientation.x = 0;
    scaled_pose.pose.orientation.y = 0;
    scaled_pose.pose.orientation.z = 0;
  }
  
    // == Additional explanation of the above coordinates and origins ==
    // While the motion planner often uses base_link as origin, it is not convenient for the operator as s/he likely wishes to see the motion relative to end effector.
    // leap_motion frame has been attached to the end effector and, thus, hand motion is visualized relative to the pose of end effector.
    // If hand is moved to the origin of hand motion system, the target position is at current position. This makes the UI more intuitive for the operator, imho.
    // Scaling down drags the marker and the corresponding target position towards the actual origin of hand motion systems (i.e. Leap Motion Controller).
    // Leap Motion Controller has forward and left-right origins in the middle of the sensor display, which is OK, but up-down origin is on the keyboard.
    // Subtracting HEIGHT_OF_ZERO sets the up-down origin at HEIGHT_OF_ZERO above the keyboard and allows negative values which represent downward motion relative to end effector.

  // Setting properly scaled and limited pose as the live_pose_
  live_pose_.pose = scaled_pose.pose;
  live_pose_.header.frame_id = leap_data.header.frame_id;
  live_pose_.header.stamp = ros::Time(0);//ros::Time::now();

  // Print position info to terminal
  printf("Scale motion by %f; move robot end-effector by (x=%f, y=%f, z=%f) mm; position_fwd_only_=%d\n",
	   AMP_HAND_MOTION_*scale_by_, live_pose_.pose.position.x*1000, live_pose_.pose.position.y*1000, live_pose_.pose.position.z*1000, position_fwd_only_);

  return;
} // end processLeap

/** Callback function for Griffin Powermate events subscriber.
 *  It either reacts to push button being pressed or it updates the scaling factor.
 *  @param powermate temoto::Dial message published by griffin_powermate node.
 */
void Teleoperator::processPowermate(griffin_powermate::PowermateEvent powermate)
{
  if (powermate.push_state_changed)		// if push event (i.e. press or depress) has occured_occured
  {
    if (powermate.is_pressed)			// if the push button has been pressed
    {
      ROS_INFO("[start_teleop] Griffin Powermate knob has been pressed");
      callRobotMotionInterface(0x03);		// makes the service request to move the robot; requests plan&execute
    }
    else					// if the push button has been depressed
    {
      ROS_INFO("[start_teleop] Griffin Powermate knob has been depressed");
    }
    return;
  }
  else						// if push event did not occur, it had to be turning
  {
    // Calculate step size depening on the current scale_by_ value. Negating direction means that clock-wise is zoom-in/scale-down and ccw is zoom-out/scale-up
    // The smaller the scale_by_, the smaller the step. log10 gives the order of magnitude
    double step = (-powermate.direction)*pow( 10,  floor( log10( scale_by_ ) ) - 1 );
    if (step < -0.09) step = -0.01;		// special case for when scale_by is 1; to ensure that step is never larger than 0.01
    scale_by_ = scale_by_ + step;		// increase/decrease scale_by
    if (scale_by_ > 1) scale_by_ = 1;		// to ensure that scale_by is never larger than 1
    // Print position info to terminal
    printf("Scale motion by %f; move SIA5 by (x=%f, y=%f, z=%f) mm; position_fwd_only_=%d\n",
	    AMP_HAND_MOTION_*scale_by_, live_pose_.pose.position.x*1000, live_pose_.pose.position.y*1000, live_pose_.pose.position.z*1000, position_fwd_only_);
    return;
  } // else
} // end processPowermate()

/** Callback function for /temoto/end_effector_pose.
 *  Sets the received pose of an end effector as current_pose_.
 *  @param end_eff_pose geometry_msgs::PoseStamped for end effector.
 */
void Teleoperator::updateEndEffectorPose(geometry_msgs::PoseStamped end_effector_pose)
{
  current_pose_ = end_effector_pose;		// sets the position of end effector as current pose
  return;
} // end processEndeffector()


/** This function fixes the quaternion of a pose input in 'inverted control mode'.
 *  @param operator_input_quaternion_msg is the original quaternion during 'inverted control mode'.
 *  @return geometry_msgs::Quaternion quaternion_msg_out is the proper quaternion leap_motion frame.
 */
geometry_msgs::Quaternion Teleoperator::oneEightyAroundOperatorUp(geometry_msgs::Quaternion operator_input_quaternion_msg)
{
  geometry_msgs::Quaternion quaternion_msg_out;
  // Adjust orientation for inverted control mode, i.e. translate leap_motion to leap_motion_on_robot
  tf::Quaternion invert_rotation(0, 1, 0, 0);				// 180° turn around Y axis, UP in leap_motion frame
  tf::Quaternion operator_input;					// incoming operator's palm orientation
  tf::quaternionMsgToTF(operator_input_quaternion_msg, operator_input);	// convert incoming quaternion msg to tf quaternion
  tf::Quaternion final_rotation = operator_input * invert_rotation;	// apply invert_rotation to incoming palm rotation
  final_rotation.normalize();						// normalize resulting quaternion
  tf::quaternionTFToMsg(final_rotation, quaternion_msg_out);		// convert tf quaternion to quaternion msg
  return quaternion_msg_out;
}

/** Callback function for /temoto/voice_commands.
 *  Executes published voice command.
 *  @param voice_command contains the specific command as an unsigned integer.
 */
void Teleoperator::processVoiceCommand(temoto::Command voice_command)
{
  // TODO check for namespace
  if (voice_command.code == 0xff)
  {
    ROS_INFO("Voice command received! Aborting ...");
    // TODO
  }
  else if (voice_command.code == 0x00)
  {
    ROS_INFO("Voice command received! Stopping ...");
    // Request abort motion
    callRobotMotionInterface(0xff);
    
    // stop the test task
    okay_robot_execute.data = false;
  }
  else if (voice_command.code == 0x01)
  {
    ROS_INFO("Voice command received! Planning ...");
    callRobotMotionInterface(0x01);
  }
  else if (voice_command.code == 0x02)
  {
    ROS_INFO("Voice command received! Executing last plan ...");
    callRobotMotionInterface(0x02);
  }
  else if (voice_command.code == 0x03)
  {
    ROS_INFO("Voice command received! Planning and moving ...");
    callRobotMotionInterface(0x03);
  }
  else if (voice_command.code == 0xf1)
  {
    ROS_INFO("Voice command received! Planning to home ...");
    callRobotMotionInterfaceWithNamedTarget(0x01, "home_pose");
  }
  else if (voice_command.code == 0xf3)
  {
    ROS_INFO("Voice command received! Planning and moving to home ...");
    callRobotMotionInterfaceWithNamedTarget(0x03, "home_pose");
  }
  else if (voice_command.code == 0x10)
  {
    ROS_INFO("Voice command received! Using natural control perspective ...");
    temoto::ChangeTf switch_human2robot_tf;
    switch_human2robot_tf.request.first_person_perspective = true;	// request a change of control perspective
    switch_human2robot_tf.request.navigate = navigate_to_goal_;	// preserve current navigation/manipulation mode
    // if service request successful, change the value of control perspective in this node
    if ( tf_change_client_.call( switch_human2robot_tf ) ) using_natural_control_ = true;
  }
  else if (voice_command.code == 0x11)
  {
    ROS_INFO("Voice command received! Using inverted control perspective ...");
    temoto::ChangeTf switch_human2robot_tf;
    switch_human2robot_tf.request.first_person_perspective = false;	// request a change of control perspective
    switch_human2robot_tf.request.navigate = navigate_to_goal_;	// preserve current navigation/manipulation mode
    // if service request successful, change the value of control perspective in this node
    if ( tf_change_client_.call( switch_human2robot_tf ) ) using_natural_control_ = false;
  }
  else if (voice_command.code == 0x20)
  {
    ROS_INFO("Voice command received! Acknowledging 'Free directions' ...");
    position_limited_ = false;
    position_fwd_only_ = false;				// once input position is not limited, there cannot be "forward only" mode
  }
  else if (voice_command.code == 0x21)
  {
    ROS_INFO("Voice command received! Acknowledging 'Limit directions' ...");
    position_limited_ = true;
  }
  else if (voice_command.code == 0x22)
  {
    ROS_INFO("Voice command received! Considering hand rotation/orientation ...");
    orientation_locked_ = false;
  }
  else if (voice_command.code == 0x23)
  {
    ROS_INFO("Voice command received! Ignoring hand rotation/orientation ...");
    orientation_locked_ = true;
  }
  else if (voice_command.code == 0x34)			// Restart (delete all and add new) Cartesian wayposes
  {
    ROS_INFO("Voice command received! Started defining new Cartesian path ...");
    wayposes_.clear();					// Clear existing wayposes
    geometry_msgs::Pose new_waypose = live_pose_.pose;	// Set live_pose_ as a new waypose
    if (!using_natural_control_)			// Fix orientation if in inverted control perspective
    {
      new_waypose.orientation = oneEightyAroundOperatorUp(new_waypose.orientation);
    }
    wayposes_.push_back(new_waypose);			// Add a waypose
    computeCartesian(live_pose_.header.frame_id.c_str());// Try to compute Cartesian path
  }
  else if (voice_command.code == 0x35)			// Add a Cartesian waypose to the end existing Cartesian path
  {
    ROS_INFO("Voice command received! Adding a pose to Cartesian path ...");
    geometry_msgs::Pose new_waypose = live_pose_.pose;	// Set live_pose_ as a new waypose
    if (!using_natural_control_)			// Fix orientation if in inverted control perspective
    {
      new_waypose.orientation = oneEightyAroundOperatorUp(new_waypose.orientation);
    }
    wayposes_.push_back(new_waypose);			// Add a waypose
    computeCartesian(live_pose_.header.frame_id.c_str());// Try to compute Cartesian path
  }
  else if (voice_command.code == 0x36)			// Remove the last Cartesian waypose
  {
    ROS_INFO("Voice command received! Removing the last Cartesian waypose  ...");
    wayposes_.pop_back();				// Remove last waypose
    computeCartesian(live_pose_.header.frame_id.c_str());// Try to compute Cartesian path
  }
  else if (voice_command.code == 0x37)			// Remove the last Cartesian waypose
  { 
    ROS_INFO("Voice command received! Removing all Cartesian wayposes  ...");
    wayposes_.clear();					// Clear all wayposes
    wayposes_fixed_in_baselink_.clear();		// Clear wayposes defined in base_link
  }
  else if (voice_command.code == 0x40 && control_state_ != 2)	// Switch over to manipulation (MoveIt!) mode
  { 
    ROS_INFO("Voice command received! Going into MANIPULATION mode  ...");
    AMP_HAND_MOTION_ = 10;
    temoto::ChangeTf switch_human2robot_tf;
    switch_human2robot_tf.request.navigate = false;	// request a change of control mode
    switch_human2robot_tf.request.first_person_perspective = using_natural_control_;	// preserve current control perspective
    // if service request successful, change the value of control mode in this node
    if ( tf_change_client_.call( switch_human2robot_tf ) ) navigate_to_goal_ = false;
  }
  else if (voice_command.code == 0x41 && control_state_ != 1)	// Switch over to navigation mode
  { 
    ROS_INFO("Voice command received! Going into NAVIGATION mode  ...");
    AMP_HAND_MOTION_ = 100;
    temoto::ChangeTf switch_human2robot_tf;
    switch_human2robot_tf.request.navigate = true;	// request a change of control mode
    switch_human2robot_tf.request.first_person_perspective = using_natural_control_;	// preserve current control perspective
    // if service request successful, change the value of control mode in this node
    if ( tf_change_client_.call( switch_human2robot_tf ) ) navigate_to_goal_ = true;
  }
  else if (voice_command.code == 0x66)			// test command for some subtask
  {
    ROS_INFO("Executing subtask. Setting okay_robot_execute to TRUE.");
    okay_robot_execute.data = true;
  }
  
  else
  {
    ROS_INFO("Voice command received! Unknown voice command.");
  }
  return;
}

/** Puts all the latest private variable values into temoto/status message.
 *  @return temoto::Status message.
 */
temoto::Status Teleoperator::getStatus()
{
  temoto::Status status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = "teleoperator";
  status.scale_by = scale_by_;					// Latest scale_by_ value
  status.live_hand_pose = live_pose_;				// Latest hand pose stamped
  status.cartesian_wayposes = wayposes_fixed_in_baselink_;	// Latest cartesian wayposes in base_link frame
  status.in_natural_control_mode = using_natural_control_;
  status.orientation_free = !orientation_locked_;
  status.position_unlimited = !position_limited_;
  status.end_effector_pose = current_pose_;			// latest known end effector pose
  status.position_forward_only = position_fwd_only_;
  status.in_navigation_mode = navigate_to_goal_;

  return status;
} // end getStatus()

/** Main method. */
int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "start_teleop");

  // NodeHandle for regular stuff
  ros::NodeHandle n;

  // NodeHandle for accessing private parameters
  ros::NodeHandle pn("~");

  // Setting work rate to 1 kHz
  ros::Rate node_rate(500);

  // Getting user-specified primary hand from ROS parameter server
  std::string primary_hand_name;
  pn.param<std::string>("primary_hand", primary_hand_name, "left");

  // Getting parameters that specify the types of teleoperation
  // By default navigation is turned OFF
  bool navigate;
  pn.param<bool>("navigate", navigate, false);
  // By default manipulation is turned ON
  bool manipulate;
  pn.param<bool>("manipulate", manipulate, true);
  
  // Instance of Teleoperator
  Teleoperator temoto_teleop(primary_hand_name, navigate, manipulate);
 
  // Setup Teleoperator ROS clients
  // ROS client for /temoto/move_robot_service
  temoto_teleop.move_robot_client_ = n.serviceClient<temoto::Goal>("temoto/move_robot_service");
  // ROS client for /temoto/navigate_robot_srv
  temoto_teleop.navigate_robot_client_ = n.serviceClient<temoto::Goal>("temoto/navigate_robot_srv");
  // ROS client for requesting change of transform between operator's hand frame and the robot's tool/planning frame
  temoto_teleop.tf_change_client_ = n.serviceClient<temoto::ChangeTf>("temoto/change_human2robot_tf");

  // Wait for human_frame_broadcaster to come on-line.
  while( !temoto_teleop.tf_change_client_.waitForExistence(ros::Duration(1)) )
  {
      ROS_INFO("[start_teleop] Waiting for human_frame_broadcaster.");
  }
  // Make a request for initial human2robot TF set-up
  temoto::ChangeTf initial_human2robot_tf;
  initial_human2robot_tf.request.navigate = temoto_teleop.getStatus().in_navigation_mode;
  initial_human2robot_tf.request.first_person_perspective = temoto_teleop.getStatus().in_natural_control_mode;
  temoto_teleop.tf_change_client_.call( initial_human2robot_tf );

  // Setup ROS subscribers
  // ROS subscriber on /griffin_powermate
  ros::Subscriber sub_scaling_factor = n.subscribe<griffin_powermate::PowermateEvent>("/griffin_powermate/events", 10, &Teleoperator::processPowermate, &temoto_teleop);
  // ROS subscriber on /leapmotion_general
  ros::Subscriber sub_operator_hand = n.subscribe("leap_motion_output", 10,  &Teleoperator::processLeap, &temoto_teleop);
  // ROS subscriber on /temoto/voice_commands
  ros::Subscriber sub_voice_commands = n.subscribe("temoto/voice_commands", 1, &Teleoperator::processVoiceCommand, &temoto_teleop);
  // ROS subscriber on /temoto/end_effector_pose
  ros::Subscriber sub_end_effector = n.subscribe("temoto/end_effector_pose", 0, &Teleoperator::updateEndEffectorPose, &temoto_teleop);
  
  // Setup ROS publisher for Teleoperator::getStatus()
  ros::Publisher pub_status = n.advertise<temoto::Status>("temoto/status", 3);
  
//   // ==== This code is required to trigger contact task demo. ==============================
//   // ROS publisher for triggering compliant contact task demo.
//   ros::Publisher pub_cc_demo_trigger = n.advertise<std_msgs::Bool>("enable_compliance", 1);
//   bool compliance_is_on = false;
//   // =======================================================================================
  
  ROS_INFO("Starting teleoperation ...");
  
  
  while ( ros::ok() )
  {
    // publish status current
    pub_status.publish( temoto_teleop.getStatus() );
    
//     // ==== This code is required to trigger contact task demo. ==============================
//     if (compliance_is_on != temoto_teleop.okay_robot_execute.data)
//     {
//       // publish the current value for okay_robot_execute
//       pub_cc_demo_trigger.publish( temoto_teleop.okay_robot_execute );
//       compliance_is_on = temoto_teleop.okay_robot_execute.data;
//     }
//     // =======================================================================================
    
    // spins once to update subscribers or something like that
    ros::spinOnce();
    
    // sleep to meet rate
    node_rate.sleep();
  } // end while
  
  return 0;
} // end main