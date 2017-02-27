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

/** @file move_robot.cpp
 * 
 *  @brief ROS server that acts as an interface for ROS MoveIt! and publishes end-effector pose.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/move_robot.h"

/** This method is executed when temoto/move_robot_service service is called.
 *  It updates target_pose_stamped based on the client's request and sets the new_move_req flag to let main() know that moving of the robot has been requested.
 *  @param req temoto::Goal service request.
 *  @param res temoto::Goal service response.
 *  @return always true.
 */
bool MoveRobotInterface::serviceUpdate(temoto::Goal::Request  &req,
				       temoto::Goal::Response &res)
{
//  ROS_INFO("New service update requested.");
  if (req.action_type == temoto::GoalRequest::CARTESIAN_COMPUTE)
  {
    // Set current state as the start state for planner. For some reason the actual built-in function doesn't do that.
    movegroup_.setStartState( *(movegroup_.getCurrentState()) );
    
    // waypoints are interpreted in the "leap_motion" ref.frame
    movegroup_.setPoseReferenceFrame(req.cartesian_frame);
    
    // Computed Cartesian trajectory.
    moveit_msgs::RobotTrajectory trajectory;
    res.cartesian_fraction = movegroup_.computeCartesianPath(req.cartesian_wayposes,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);
    ROS_INFO("[MoveRobotInterface::serviceUpdate] Cartesian path %.2f%% acheived", res.cartesian_fraction * 100.0);
  
    latest_plan_.trajectory_ = trajectory;
    new_plan_available_ = true;
  }
  else
  {
    // sets the action associated to target pose
    req_action_type_ = req.action_type;
    
    // sets target requested pose
    target_pose_stamped_ = req.goal;
    
    // check for named target
    use_named_target_ = false;					// by default do not use named_target.
    named_target_ = req.named_target;				// get named_target from the service request.
    ROS_INFO("[MoveRobotInterface::serviceUpdate] named_target_='%s'", named_target_.c_str());
    if (!named_target_.empty()) use_named_target_ = true;	// if a named target was specified, set use_named_target_ to true.
    if (use_named_target_) ROS_INFO("[MoveRobotInterface::serviceUpdate] use_named_target is set TRUE");
    
    // Set new_move_requested_ TRUE for main() to see it
    new_move_requested_ = true;
    ROS_INFO("[MoveRobotInterface::serviceUpdate] new_move_requested_ is set TRUE");
  }
  return true;
} // end serviceUpdate

/** Plans and/or executes the motion of the robot.
 *  @param robot MoveGroup object of the robot.
 */
void MoveRobotInterface::requestMove()
{

  // Set current state as the start state for planner. For some reason the actual built-in function doesn't do that.
  movegroup_.setStartState( *(movegroup_.getCurrentState()) );
  
  // Get and print current position of the end effector
  geometry_msgs::PoseStamped current_pose = movegroup_.getCurrentPose();
  ROS_INFO("[move_robot/requestMove] === CURRENT POSE ( as given by MoveGroup::getCurrentPose() ) ===");
  ROS_INFO("[move_robot/requestMove] Current pose frame: %s", current_pose.header.frame_id.c_str());
  ROS_INFO("[move_robot/requestMove] Current pose (posit x, y, z): (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  ROS_INFO("[move_robot/requestMove] Current pose (orien x, y, z, w): (%f, %f, %f, %f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
 
  // Use either named or pose target (named target takes the priority over regular pose target).
  if (use_named_target_)						// if use_named_target is true
  {
    bool set_target_ok = movegroup_.setNamedTarget(named_target_);	// set target pose as a named target
    if (!set_target_ok)							// check if setting named target was successful
    {
      ROS_INFO("[move_robot/requestMove] Failed to set named target. Please retry.");
      return;								// return if setNamedTarget failed
    }
    else
      ROS_INFO("[move_robot/requestMove] Using NAMED TARGET for planning and/or moving.");
  } // end if (use_named_target_)
  else
  {
//     movegroup_.setEndEffectorLink( "temoto_end_effector" );
    ROS_INFO("[move_robot/requestMove] Found end effector link: %s", movegroup_.getEndEffectorLink().c_str());
    // use the stamped target pose to set the target pose for robot
    bool set_target_ok = movegroup_.setPoseTarget(target_pose_stamped_ /*, "temoto_end_effector"*/);
    if (!set_target_ok)							// check if set target pose failed
    {
      ROS_INFO("[move_robot/requestMove] Failed to set pose target. Please retry.");
      return;								// return if setPoseTarget failed
    }
    else
      ROS_INFO("[move_robot/requestMove] Using POSE TARGET for planning and/or moving.");
  }

  // TODO
//   // Set goal tolerance based on the actual shift from current positio to target position
//   std::vector <geometry_msgs::Point> current_and_target;		// Vector that contains current and target points
//   current_and_target.push_back(current_pose.pose.position);		// Add current position
//   current_and_target.push_back(target_pose.position);			// Add target position
//   double shift = calculateDistance(current_and_target);			// Calculate the linear distance between the two positions
//   // CALCULATE DISTNACE BETWEEN CURRENT AND TARGET
//   // CALCULATE TOLERANCE AS A PERCENTAGE OF DISTANCE && CONSIDER SOME MIN/MAX LIMITS
//   // SET TOLERANCE FOR PLANNING
  
  // Just checking what is the target pose
  geometry_msgs::PoseStamped current_target = movegroup_.getPoseTarget();
  ROS_INFO("[move_robot/requestMove] Target pose frame: %s", current_target.header.frame_id.c_str());
  ROS_INFO("[move_robot/requestMove] Target pose (posit x, y, z): (%f, %f, %f)", current_target.pose.position.x, current_target.pose.position.y, current_target.pose.position.z);
  ROS_INFO("[move_robot/requestMove] Target pose (orien x, y, z, w): (%f, %f, %f, %f)", current_target.pose.orientation.x, current_target.pose.orientation.y, current_target.pose.orientation.z, current_target.pose.orientation.w);

  // Based on action type: PLAN (0x01), EXECUTE PLAN (0x02), or PLAN&EXECUTE (0x03)
  if ( req_action_type_ == temoto::GoalRequest::PLAN )
  {
    ROS_INFO("[move_robot/requestMove] Starting to plan ...");
    ROS_INFO("[move_robot/requestMove] Planning frame: %s", movegroup_.getPlanningFrame().c_str());
    movegroup_.plan( latest_plan_ );				// Calculate plan and store it in latest_plan_.
    new_plan_available_ = true;					// Set new_plan_available_ to TRUE.
    ROS_INFO("[move_robot/requestMove] DONE planning.");
  }
  else if ( req_action_type_ == temoto::GoalRequest::EXECUTE )
  {
    ROS_INFO("[move_robot/requestMove] Starting to execute last plan ...");
    if ( new_plan_available_ ) movegroup_.execute( latest_plan_ );	// If there is a new plan, execute latest_plan_.
    else ROS_INFO("[move_robot/requestMove] No plan to execute.");	// Else do nothing but printout "no plan"
    new_plan_available_ = false;					// Either case, set new_plan_available_ to FALSE.
    ROS_INFO("[move_robot/requestMove] DONE executing the plan");
  }
  else if ( req_action_type_ == temoto::GoalRequest::GO )
  {
    ROS_INFO("[move_robot/requestMove] Starting to move (i.e. plan & execute) ...");
//     robot.move();						// Plan and execute.
    // Since move() has a bug of start state not being current state, I am going to plan and execute sequentally.
    moveit::planning_interface::MoveGroup::Plan move_plan;
    printf("[move_robot/requestMove] Planning ...");
    movegroup_.plan( move_plan );
    printf("[DONE] \n[move_robot/requestMove] and Executing ...\n");
    movegroup_.execute( move_plan );
    ROS_INFO("[move_robot/requestMove] DONE moving.");
    new_plan_available_ = false;					// As any previous plan has become invalid, set new_plan_available_ to FALSE.
  }

  return;
} // end requestMove

/** Main method. */
int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "move_robot");
  // ROS Nodehandle for public namespace
  ros::NodeHandle n;
  // ROS NodeHandle for accessing private parameters
  ros::NodeHandle pn("~");
  
  // Rate used for this node. If I only use the following async spinenr, the node also works but takes over 130%CPU.
  ros::Rate node_rate(200);

  // Using an async spinner. It is needed for moveit's MoveGroup::plan(), which would get stuck otherwise. Might be a bug.
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  // get user-specified name for the movegroup as a private parameter
  std::string move_group_name;
  pn.getParam("movegroup", move_group_name);
  if ( move_group_name.empty() )
  {
    ROS_ERROR("[move_robot/main] No movegroup name was specified. Aborting.");
    return -1;
  }
  ROS_INFO("[move_robot/main] Retrieved '%s' from parameter server as a movegroup name.", move_group_name.c_str());
  
  // Create MoveRobotInterface for user-specified movegroup
  MoveRobotInterface moveIF(move_group_name);
  // Using RRTConnectkConfigDefault planner for motion planning
  moveIF.movegroup_.setPlannerId("RRTConnectkConfigDefault");
  
  // FYI
  ROS_INFO("[move_robot/main] Planning frame: %s", moveIF.movegroup_.getPlanningFrame().c_str());
  ROS_INFO("[move_robot/main] End effector link: %s", moveIF.movegroup_.getEndEffectorLink().c_str());
  ROS_INFO("[move_robot/main] End effector: %s", moveIF.movegroup_.getEndEffector().c_str());
  ROS_INFO("[move_robot/main] Goal position tolerance is: %.6f", moveIF.movegroup_.getGoalPositionTolerance());
  ROS_INFO("[move_robot/main] Goal orientation tolerance is: %.6f", moveIF.movegroup_.getGoalOrientationTolerance());
  ROS_INFO("[move_robot/main] Goal joint tolerance is: %.6f", moveIF.movegroup_.getGoalJointTolerance());
  
  // Set up service for move_robot_service; if there's a service request, executes serviceUpdate() function
  ros::ServiceServer service = n.advertiseService("temoto/move_robot_service", &MoveRobotInterface::serviceUpdate, &moveIF);
  ROS_INFO("[move_robot/main] Service 'temoto/move_robot_service' up and going. Ready to send move commands to %s.", move_group_name.c_str());
  
  // Set up publisher for the end effector location
  ros::Publisher pub_end_effector = n.advertise<geometry_msgs::PoseStamped>( "temoto/end_effector_pose", 1 );
  
  // ROS client for /temoto/adjust_rviz_camera
  ros::ServiceClient client_visual = n.serviceClient<std_srvs::Empty>("temoto/adjust_rviz_camera");
  std_srvs::Empty empty_srv;			// Empty service.

  while ( ros::ok() )
  {
    // check if there has been a service request for a new move
    if ( moveIF.new_move_requested_ && moveIF.req_action_type_ < 0x04 )
    {
      moveIF.requestMove();			// plan and execute move using move_group
      moveIF.new_move_requested_ = false;	// set request flag to false
      moveIF.new_end_effector_pose_ = true;	// assumes that request move resulted in new pose for end effector and sets the corresponding flag
    }
    
    // get and publish current end effector pose;
    pub_end_effector.publish( moveIF.movegroup_.getCurrentPose() );

    // If pose of the end effector has changed. Update camera position if the end effector has moved since the last time camera was positioned.
    if ( moveIF.new_end_effector_pose_ )
    {
      client_visual.call(empty_srv);
      moveIF.new_end_effector_pose_ = false;	// set new_end_effector_pose to zero
    } // end if
    
    ros::spinOnce();
    // sleep to meet the node_rate frequency
    node_rate.sleep();
    
  } // end while

  return 0;
} // end main
