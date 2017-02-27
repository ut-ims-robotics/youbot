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

/** @file human_frame_broadcaster.cpp
 * 
 *  @brief ROS server that broadcasts the transform to the frame the human input is interpreted in.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

// temoto includes
#include "temoto/temoto_common.h"

bool g_natural_perspective = true;		///< Is TRUE for natural interpretation of human input; FALSE for inverted perspective.
bool g_navigation_control = true;		///< Is TRUE when human input is to be interpred as a navigation goal in base_link frame.

/** This method is executed when change_human2robot_tf service is called.
 *  It updates leap_motion frame based on the client's request.
 *  @param req temoto::ChangeTf service request.
 *  @param res temoto::ChangeTf service response.
 *  @return always true.
 */
bool service_change_tf(	temoto::ChangeTf::Request  &req,
			temoto::ChangeTf::Response &res)
{
  // Get the value from the request
  g_natural_perspective = req.first_person_perspective;
  g_navigation_control = req.navigate;
  ROS_INFO("[human_frame_broadcaster] New service update requested. Now g_natural_perspective = %d; g_navigation_control = %d", g_natural_perspective, g_navigation_control);
  
  return true;
} // end service_change_tf

/** Main method. */
int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "human_frame_broadcaster");
  // ROS node handle
  ros::NodeHandle nh;
  // Set ROS rate to 10 Hz
  ros::Rate r(10);
  
  // Human input frame.
  std::string human_frame;
  // Motion-planning control frame.
//   std::string eef_frame;
  // Navigation control frame.
  std::string mobile_frame;
  
  // Get proper frame names from parameter servers
  ROS_INFO("[human_frame_broadcaster] Getting frame names from parameter server.");
  nh.param<std::string>("/temoto_frames/human_input", human_frame, "leap_motion");
  ROS_INFO("[human_frame_broadcaster] Human frame is: %s", human_frame.c_str());
//   nh.param<std::string>("/temoto_frames/end_effector", eef_frame, "temoto_end_effector");
//   ROS_INFO("[human_frame_broadcaster] End-effector frame is: %s", eef_frame.c_str());
  nh.param<std::string>("/temoto_frames/mobile_base", mobile_frame, "base_link");
  ROS_INFO("[human_frame_broadcaster] Mobile base frame is: %s", mobile_frame.c_str());
  
  // Create a tranform broadcaster.
  static tf::TransformBroadcaster tf_broadcaster;
  
  // Advertise a service for switchig between tranfrom rotations.
  ros::ServiceServer service = nh.advertiseService("temoto/change_human2robot_tf", service_change_tf);
  ROS_INFO("[human_frame_broadcaster] Service 'temoto/change_human2robot_tf' up and going.");
  
  // A tranform between leap_motion (child) frame and parent robot frame. 
  tf::Transform hand_frame_to_robot;

  while ( ros::ok() )
  {  
    // IF human input is to be used for navigating robot base
    if ( g_navigation_control )
    {
      hand_frame_to_robot.setOrigin( tf::Vector3(0, 0, 0) );
      // in navigation mode, leap_motion rotated RPY=(90, 0, -90) in base_link
      hand_frame_to_robot.setRotation( tf::Quaternion(0.5, -0.5, -0.5, 0.5) );// set leap_motion about base_link
      
      // Broadcast a transform that attaches leap_motion to base_link using the hand_frame_to_robot transform.
      tf_broadcaster.sendTransform( tf::StampedTransform(hand_frame_to_robot, ros::Time::now(), mobile_frame, human_frame) );
    }
    // ELSE: human input is used for moving end effector
    else
    {
      // Set appropriate rotation for how leap_motion data is interpreted.
      if ( g_natural_perspective )						// "natural" means robot arm is direct extension of human hand
      {
	// in natural mode leap_motion and leap_motion_on_robot are oriented exactly the same
	hand_frame_to_robot.setRotation ( tf::Quaternion(0, 0, 0, 1) );	// identity quaternion, i.e., no rotation
      }
      else 									// not "natural" means human is facing the robot, i.e. left and right are inverted.
      {
	// in inverted mode leap_motion is rotated 180° around the y(UP)-axis of leap_motion_on_robot
	hand_frame_to_robot.setRotation( tf::Quaternion(0, 1, 0, 0) );	// 180 around y-axis
      }

      // Broadcast a transform that attaches leap_motion to leap_motion_on_robot using the hand_frame_to_robot tranform. TODO: "leap_motion_on_robot" should not be hard-coded
      tf_broadcaster.sendTransform( tf::StampedTransform(hand_frame_to_robot, ros::Time::now(), "leap_motion_on_robot", human_frame) );
    }

    ros::spinOnce();
    r.sleep();
  }
  
  return 1;
}