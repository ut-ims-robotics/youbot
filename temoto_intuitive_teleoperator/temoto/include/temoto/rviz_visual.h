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

/** @file rviz_visual.h
 * 
 *  @brief Node that handles visual markers and point-of-view camera in RViz for
 *         temoto teleoperation.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"
#include "view_controller_msgs/CameraPlacement.h"

// temoto includes
#include "temoto/temoto_common.h"
#include "griffin_powermate/PowermateEvent.h"

#ifndef RVIZ_VISUAL_H
#define RVIZ_VISUAL_H

class Visuals
{
public:
  // ___ CONSTRUCTOR ___
  Visuals(std::string human, std::string end_effector, std::string mobile_base)
  {
    human_frame_ = human;
    eef_frame_ = end_effector;
    mobile_frame_ = mobile_base;
    // Initialize point-of-view camera placement and all the required markers
    initPOWCamera(/*"temoto_end_effector"*/);
    initDisplacementArrow();
    initDistanceAsText();
    initHandPoseMarker();
    initActiveRangeBox();
    initCartesianPath();
    
    // Initial state setup
    adjust_camera_ = true;
    camera_is_aligned_ = true;
    latest_known_camera_mode_ = 0;
  };
   
  /** Server function for temoto/adjust_rviz_camera. Sets adjust_camera_ TRUE. */
  bool adjustPOWCameraPlacement (std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
  
  /** Callback function for /temoto/status. Looks for changes that require setting adjust_camera_ TRUE. */
  void updateStatus (temoto::Status status);
  
  /** Callback function for /griffin_powermate/events. Sets adjust_camera_ TRUE. */
  void powermateWheelEvent (griffin_powermate::PowermateEvent powermate);
  
  /** Updates RViz point-of-view and visualization markers as needed. */
  void crunch(ros::Publisher &marker_publisher, ros::Publisher &pow_publisher);
  
  /** ROS camera placement message that defines user's point-of-view in RViz. */
  view_controller_msgs::CameraPlacement point_of_view_;

private:
  // ___ INITIALIZERS ___
  
  /** Initializes camera placement to a preset pose in frame specified by frame_id */
  void initPOWCamera();
  
  /** Creates the initial marker that visualizes hand movement as a displacement arrow. */
  void initDisplacementArrow();
  
  /** Creates the initial marker that displays front-facing text. */
  void initDistanceAsText();

  /** Creates the initial marker that visualizes hand pose as a flattened box. */
  void initHandPoseMarker();
  
  /** Creates the initial marker for an active range box around the robot where target position is always in one of the corners. */
  void initActiveRangeBox();
  
  /** Creates the initial marker that visualizes cartesian path by connecting wayposes with lines. */
  void initCartesianPath();
  
  // ___ HELPER FUNCTIONS ___
  
  /** Calculates the distance between two points in meters or millimeters and returns it as a string. */
  std::string getDistanceString (std::vector <geometry_msgs::Point> & twoPointVector);

  /** Changes target_frame and header.frame_ids of every pose in point_of_view_ to frame_id. */
  void changePOWCameraFrameTo(std::string frame_id); 

  // ___ CLASS VARIABLES AND CONSTANTS ___
  
  /** Human input frame. */
  std::string human_frame_;
  
  /** Motion-planning control frame. */
  std::string eef_frame_;
  
  /** Navigation control frame. */
  std::string mobile_frame_;

  /** This is set TRUE, if it is needed to adjust the position of the point-of-view (POW) camera in RViz.*/
  bool adjust_camera_;

  /** Camera is algined with end effector (i.e., TRUE) as opposed to being in top-down point-of-view (i.e., FALSE).*/
  bool camera_is_aligned_;
  
  /** A state variable for keeping track of where rviz_visual node thinks the camera is. */
  uint8_t latest_known_camera_mode_; 	// 0-unknown, 1-natural, 2-inverted, 11-navigation natural, 12-navigation inverted

  /** Latest recieved full system status published by start_teleop node. */
  temoto::Status latest_status_;
  
  /** Arrow-shaped marker that visualizes target displacement of the robot. */
  visualization_msgs::Marker displacement_arrow_;
  
  /** Text marker for displaying the length of displacement represented by displacement_arrow_. */
  visualization_msgs::Marker distance_as_text_;
  
  /** Flattened-box-shaped marker that represents the position and orientation of the operator's primary hand. */
  visualization_msgs::Marker hand_pose_marker_;
  
  /** Translucent box or area that is centered around the starting pose while displacement_arrow_ always points to one of its corners. */
  visualization_msgs::Marker active_range_box_;
  
  /** Line strip marker that interconnects all the wayposes requested for a cartesian motion. */
  visualization_msgs::Marker cartesian_path_;

  /** A number that is used when some distance is required between robot and any marker or camera. */
  const double VIRTUAL_SCREEN_FRONT_ = 0;

  /** A number that is used when some distance is required between robot and any marker or camera. */
  const double VIRTUAL_SCREEN_TOP_ = 0.2;
};

#endif