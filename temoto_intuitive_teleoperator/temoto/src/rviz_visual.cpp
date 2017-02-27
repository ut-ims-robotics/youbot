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

/** @file rviz_visual.cpp
 * 
 *  @brief Node that handles visual markers and point-of-view camera in RViz for
 *         temoto teleoperation.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "temoto/rviz_visual.h"

/** The actual service call function for temoto/adjust_rviz_camera.
 *  @param req empty service request.
 *  @param res empty service response.
 *  @return always true.
 */
bool Visuals::adjustPOWCameraPlacement (std_srvs::Empty::Request  &req,
				     std_srvs::Empty::Response &res)
{
  ROS_INFO("[rviz_visual/adjust_rviz_camera] adjust_camera is set 'true' due to service request.");
  // Set adjust_camera to TRUE.
  adjust_camera_ = true;
  return true;
}

/** Callback function for /temoto/status.
 *  Looks for any changes that would require re-adjustment of the point-of-view camera.
 *  Stores the received status in a class variable latest_status_.
 *  @param status temoto::Status message
 */
void Visuals::updateStatus (temoto::Status status)
{
  // Before overwriting previous status, checks if switch between camera views is necesassry due to switch between navigation and manipulation modes.
  if (status.in_navigation_mode && !latest_status_.in_navigation_mode)
  {
    adjust_camera_ = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from MANIPULATION to NAVIGATION.");
  }
  else if (!status.in_navigation_mode && latest_status_.in_navigation_mode)
  {
    adjust_camera_ = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from NAVIGATION to MANIPULATION.");
  }

  // Before overwriting previous status, checks if switch between camera views is necesassry due to limited directions.
  if (status.position_forward_only && !latest_status_.position_forward_only)
  {
    adjust_camera_ = true;
    camera_is_aligned_ = false;	// Change to top-view as operator has switched to forward only motion pattern
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from 'not fwd only' to 'fwd only'.");
  }
  else if (!status.position_forward_only && latest_status_.position_forward_only)
  {
    adjust_camera_ = true;
    camera_is_aligned_ = true;	// Change to the so-called aligned view because operator has stopped using forward only
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due change from 'fwd only' to 'not fwd only'.");
  }
  
  // Checks if switch between camera views is necesassry due to change in control mode.
  if (status.in_natural_control_mode && !latest_status_.in_natural_control_mode)
  {
    adjust_camera_ = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due to switch to 'natural control mode'.");
  }
  else if (!status.in_natural_control_mode && latest_status_.in_natural_control_mode)
  {
    adjust_camera_ = true;
    ROS_INFO("[rviz_visual/updateStatus] adjust_camera is set 'true' due to switch to 'inverted control mode'.");
  }
  
  // Checks if switch between camera views is necesassry due to change in (un)limiting directions.
  if (status.position_unlimited != latest_status_.position_unlimited) adjust_camera_ = true;
  
  // Check difference between the known and new end effector positions
  std::vector <geometry_msgs::Point> now_and_before;				// Vector that contains current and target points
  now_and_before.push_back(status.end_effector_pose.pose.position);		// Add the just received position of end effector
  now_and_before.push_back(latest_status_.end_effector_pose.pose.position);	// Add the previous known position of end effector
  double shift = calculateDistance(now_and_before);				// Calculate the linear distance between the two positions
  if (shift > 0.001) adjust_camera_ = true;					// If the difference is more than 1 mm, set adjust_camera to true.
  
  // Overwrite latest_status values with the new status.
  latest_status_ = status;
  return;
}

/** Callback function for /griffin_powermate/events.
 *  Sets adjust_camera_ flag if griffin powermate turn knob was rotated, i.e., any turn knob rotation triggers re-adjustement of camera placement.
 *  @param powermate griffin_powermate::PowermateEvent message from Griffin Powermate.
 */
void Visuals::powermateWheelEvent (griffin_powermate::PowermateEvent powermate)
{
  if (abs(powermate.direction)) adjust_camera_ = true;
  return;
}

/** Creates the initial CameraPlacment message that is used for positioning point-of-view (POW) camera in RViz.
 */
void Visuals::initPOWCamera(/*std::string frame_id*/)
{
  // Set target frame and animation time
  point_of_view_.target_frame = eef_frame_;
  point_of_view_.time_from_start = ros::Duration(0.5);

  // Position of the camera relative to target_frame
  point_of_view_.eye.header.frame_id = eef_frame_;
  point_of_view_.eye.point.x = -2;
  point_of_view_.eye.point.y = 0;
  point_of_view_.eye.point.z = 0;

  // Target_frame-relative point for the focus
  point_of_view_.focus.header.frame_id = eef_frame_;
  point_of_view_.focus.point.x = 0;
  point_of_view_.focus.point.y = 0;
  point_of_view_.focus.point.z = 0;

  // Target_frame-relative vector that maps to "up" in the view plane.
  point_of_view_.up.header.frame_id = eef_frame_;
  point_of_view_.up.vector.x = 0;
  point_of_view_.up.vector.y = 0;
  point_of_view_.up.vector.z = 1;
  
}

/** Creates the initial marker that visualizes hand movement as a displacement arrow. */
void Visuals::initDisplacementArrow()
{
  displacement_arrow_.header.frame_id = human_frame_; // x is horizontal, y is vertical, z is forward-backward // "temoto_end_effector"; // x is forward, y is horizontal, z is vertical //
  displacement_arrow_.header.stamp = ros::Time();
  displacement_arrow_.ns = "displacement_arrow";
  displacement_arrow_.id = 0;
  displacement_arrow_.type = visualization_msgs::Marker::ARROW;
  displacement_arrow_.action = visualization_msgs::Marker::ADD;

  // Defining start and end points for the arrow marker, the actual values don't matter here as they will be overwritten in the main
  geometry_msgs::Point start_point, end_point;
  start_point.x = 0.0;
  start_point.y = 0.0;
  start_point.z = 0.0;
  end_point.x = 0.0;
  end_point.y = -0.2;
  end_point.z = 0.0;

  // The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end. 
  displacement_arrow_.points.push_back(start_point);
  displacement_arrow_.points.push_back(end_point);

  // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length. 
  displacement_arrow_.scale.x = 0.001;
  displacement_arrow_.scale.y = 0.002;
  displacement_arrow_.scale.z = 0;
  
  // Make the arrow visible by setting alpha to 1.
  displacement_arrow_.color.a = 1.0;  
  // make the arrow orange
  displacement_arrow_.color.r = 1.0;
  displacement_arrow_.color.g = 0.5;
  displacement_arrow_.color.b = 0.0;
  
  return;
} // end Visuals::initDisplacementArrow()

/** Creates the initial marker that displays front-facing text. */
void Visuals::initDistanceAsText()
{
  distance_as_text_.header.frame_id = human_frame_;
  distance_as_text_.header.stamp = ros::Time();
  distance_as_text_.id = 0;
  distance_as_text_.ns = "distance_as_text";
  distance_as_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  distance_as_text_.action = visualization_msgs::Marker::ADD;
  distance_as_text_.text = "loading ...";
  
  // set the position of text; the actual values don't matter here as they will be overwritten in the main
  distance_as_text_.pose.position.x = 0.0;
  distance_as_text_.pose.position.y = 0.2;
  distance_as_text_.pose.position.z = 0.0;
  
  // Text height
  distance_as_text_.scale.z = 0.1;
  
  // Text visible & blue
  distance_as_text_.color.a = 1;
  distance_as_text_.color.b = 1;
  
  return;
} // end Visuals::initDistanceAsText()

/** Creates the initial marker that visualizes hand pose as a flattened box. */
void Visuals::initHandPoseMarker()
{
  hand_pose_marker_.header.frame_id = human_frame_;
  hand_pose_marker_.header.stamp = ros::Time();
  hand_pose_marker_.ns = "hand_pose_marker";
  hand_pose_marker_.id = 0;
  hand_pose_marker_.type = visualization_msgs::Marker::CUBE;
  hand_pose_marker_.action = visualization_msgs::Marker::ADD;

  hand_pose_marker_.scale.x = 0.06;	// side
  hand_pose_marker_.scale.y = 0.02;	// thickness
  hand_pose_marker_.scale.z = 0.15;	// depth
  
  // begin at the position of end effector
  hand_pose_marker_.pose.position.x = 0.0;
  hand_pose_marker_.pose.position.y = 0.0;
  hand_pose_marker_.pose.position.z = 0.0;

  // Make the box visible by setting alpha to 1.
  hand_pose_marker_.color.a = 1;  
  // Make the box orange.
  hand_pose_marker_.color.r = 1.0;
  hand_pose_marker_.color.g = 0.5;
  hand_pose_marker_.color.b = 0.0;
  
  return;
} // end Visuals::initHandPoseMarker()

/** Creates the initial marker for an active range box around the robot where target position is always in one of the corners. */
void Visuals::initActiveRangeBox()
{
  active_range_box_.header.frame_id = human_frame_;
  active_range_box_.header.stamp = ros::Time();
  active_range_box_.ns = "active_range_box";
  active_range_box_.id = 0;
  active_range_box_.type = visualization_msgs::Marker::CUBE;
  active_range_box_.action = visualization_msgs::Marker::ADD;

  active_range_box_.scale.x = 0.2;	// side
  active_range_box_.scale.y = 0.2;	// thickness
  active_range_box_.scale.z = 0.2;	// depth
  
  // begin at the position of end effector
  active_range_box_.pose.position.x = 0.0;
  active_range_box_.pose.position.y = 0.0;
  active_range_box_.pose.position.z = 0.0;

  // Make the box barely visible by setting alpha to 0.2.
  active_range_box_.color.a = 0.2;  
  // Make the box red.
  active_range_box_.color.r = 1.0;
  active_range_box_.color.g = 0.0;
  active_range_box_.color.b = 0.0;
  
  return;
} // end Visuals::initActiveRangeBox()

/** Creates the initial marker that visualizes cartesian path by connecting wayposes with lines. */
void Visuals::initCartesianPath()
{
  cartesian_path_.header.frame_id = "base_link"/*"leap_motion"*/;	//TODO figure out a general frame type we need here
  cartesian_path_.header.stamp = ros::Time();
  cartesian_path_.ns = "cartesian_path";
  cartesian_path_.id = 0;
  cartesian_path_.type = visualization_msgs::Marker::LINE_STRIP;
  cartesian_path_.action = visualization_msgs::Marker::ADD;

  cartesian_path_.scale.x = 0.01;	// width of the line

  // Make the line visible by setting alpha to 1.
  cartesian_path_.color.a = 1;  
  // Make the line strip blue.
  cartesian_path_.color.r = 0.0;
  cartesian_path_.color.g = 0.0;
  cartesian_path_.color.b = 1.0;
  
  return;
} // end Visuals::initCartesianPath()

/** Calculates the distance between two points in meters or millimeters and returns it as a string.
 *  @param twoPointVector vector containing two points.
 *  @return string containing the distance between two points in meters or millimeters followed by ' m' or ' mm', respectively.
 */
std::string Visuals::getDistanceString (std::vector <geometry_msgs::Point> & twoPointVector)
{
  std::string  distance_as_text;
  std::string  units;
  int precision = 2;
  // Calculate the distance between the first two points in the input vector of points
//  double distance = sqrt( pow(twoPointVector[1].x-twoPointVector[0].x, 2) + pow(twoPointVector[1].y-twoPointVector[0].y, 2) + pow(twoPointVector[1].z-twoPointVector[0].z, 2));
  double distance = calculateDistance(twoPointVector);
  if (distance < 1)	// if distance is less than 1 m, use mm instead
  {
    // Covnvert the distance m -> mm
    distance = distance*1000;
    units = " mm";
    if (distance >= 10) precision = 1;
  }
  else			// otherwise, use meters as units
  {
    units = " m";
  }
  // use the number of fractional digits specified by presion to put distance into stringstream and add units.
  std::ostringstream sstream;
  sstream << std::fixed << std::setprecision(precision) << distance << units;

  // stringstream to string
  distance_as_text = sstream.str();
  return distance_as_text;
}

/** Changes target_frame and header.frame_ids of every pose in point_of_view_ to frame_id.
 *  @param frame_id frmae name to change 
 */
void Visuals::changePOWCameraFrameTo(std::string frame_id)
{
  point_of_view_.target_frame = frame_id;
  point_of_view_.eye.header.frame_id = frame_id;
  point_of_view_.focus.header.frame_id = frame_id;
  point_of_view_.up.header.frame_id = frame_id;
}

void Visuals::crunch(ros::Publisher &marker_publisher, ros::Publisher &pow_publisher)
{
  // ============================================================ 
  // ==  VISUALIZATION MARKERS  =================================
  // ============================================================

  // Setting markers in NAVIGATION mode
  if (latest_status_.in_navigation_mode)
  {
    /* ==  ARROW  ============================================ */
    // Displacement arrow is not needed in NAVIGATION mode, so make it invisible.
    displacement_arrow_.color.a = 0;
    
    // Publish displacement_arrow_
    marker_publisher.publish( displacement_arrow_ );
    
    /* ==  ACTIVE RANGE BOX  ================================= */
    // Active range box is not needed in NAVIGATION mode, so make it invisible.
    active_range_box_.color.a = 0;
    
    // Publish active_range_box_
    marker_publisher.publish( active_range_box_ );   
    
    /* ==  HAND POSE BOX MARKER  ============================= */
    // Resize of the hand pose marker to robot base dimensions
    hand_pose_marker_.scale.x = 0.5;
    hand_pose_marker_.scale.z = 1.0; 

    // Hand pose marker (relative to leap_motion frame)
    hand_pose_marker_.pose = latest_status_.live_hand_pose.pose;
    
    // Paint the marker based on restricted motion latest_status
    if (latest_status_.orientation_free) hand_pose_marker_.color.g = 0.0;	// if hand orientation is to be considered, paint the hand pose marker red 
    else hand_pose_marker_.color.g = 0.5;					// else, the marker is orange

    // In NAVIGATION, the hand pose marker must always be visible
    hand_pose_marker_.color.a = 1;

    // Publish hand_pose_marker_
    marker_publisher.publish( hand_pose_marker_ );

    /* ==  TEXT LABEL  ======================================= */
    // Update display_distance parameters (display_distance operates relative to leap_motion frame)
    std::vector<geometry_msgs::Point> temp;
    geometry_msgs::Point zero_point;
    temp.push_back(zero_point);
    temp.push_back(latest_status_.live_hand_pose.pose.position);
    distance_as_text_.text = getDistanceString(temp);			// get the linear distance from zero to hand position end point as string
    distance_as_text_.scale.z = 0.05 + latest_status_.scale_by/8;	// scale the display text based on scale_by value
    distance_as_text_.pose.position = temp[1];				// text is placed at the hand position
    distance_as_text_.pose.position.y = 1; 				// raise text to the top 1 m

    marker_publisher.publish( distance_as_text_ );			// publish info_text visualization_marker; this is picked up by rivz
  }
  // Setting markers in MANIPULATION mode
  else
  {
    /* ==  ARROW  ============================================ */
    // Update arrow marker properties and make hand tracking visible in RViz
    // Arrow marker is described in leap_motion frame.
    // Start point of the arrow is always at (0, 0, 0).
    displacement_arrow_.points[1] = latest_status_.live_hand_pose.pose.position;// set the end point of the arrow to actual hand position
    if (latest_status_.in_natural_control_mode)
    {
      displacement_arrow_.points[0].z = -VIRTUAL_SCREEN_FRONT_;		// shift the marker in front of the FT sensor and gripper
      displacement_arrow_.points[1].z -= VIRTUAL_SCREEN_FRONT_;		// shift the marker in front of the FT sensor and gripper
    }
    else
    {
      displacement_arrow_.points[0].z = VIRTUAL_SCREEN_FRONT_;		// shift the marker in front of the FT sensor and gripper
      displacement_arrow_.points[1].z += VIRTUAL_SCREEN_FRONT_;		// shift the marker in front of the FT sensor and gripper
    }

    // Change arrow's thickness based on scaling factor
    displacement_arrow_.scale.x = 0.001 + latest_status_.scale_by/1000;	// shaft diameter when start and end point are defined
    displacement_arrow_.scale.y = 0.002 + latest_status_.scale_by/100;	// head diameter when start and end point are defined
    displacement_arrow_.scale.z = 0;					// automatic head length
    
    displacement_arrow_.color.a = 1;					// the arrow is visibly solid

    // A tweak for extreme close-ups to make the arrow out of scale but more informative
    if (latest_status_.scale_by < 0.01 && camera_is_aligned_ /*&& !latest_status.in_natural_control_mode*/)// only when the rviz camera is in the front
    {
      displacement_arrow_.points[0].z -= 0.02;				// shift arrow marker away from the camera because camera is at the VIRTUAL_VIEW_SCREEN
      displacement_arrow_.points[1].z -= 0.02;     
      displacement_arrow_.scale.x = 0.0001;				// make the shaft thinner
      displacement_arrow_.scale.y = 0.0003;				// make the arrow head thinner
      displacement_arrow_.color.a = 0.6;				// and make the arrow a bit transparent
    }

    // Change arrow's appearance when camera in on the top, looking down
    if (!camera_is_aligned_ && !latest_status_.position_unlimited)	// if camera is on the top and facing down, the arrow marker must be made more visible
    {
      displacement_arrow_.scale.x = 0.15;				// shaft is now quite fat
      displacement_arrow_.scale.y = 0.18;				// head is also made large
      displacement_arrow_.color.a = 0.4;				// the arrow marker is translucent
    }

    if (latest_status_.position_unlimited) displacement_arrow_.color.g = 0.0;	// if hand position input is unresricted, paint the arrow red 
    else displacement_arrow_.color.g = 0.5;					// else, the arrow is orange

    // Publish displacement_arrow_
    marker_publisher.publish( displacement_arrow_ );

    /* ==  ACTIVE RANGE BOX  ================================= */
    // Use the same origin/pivot point as the start point of the arrow marker
    active_range_box_.pose.position = displacement_arrow_.points[0];
    // Dimensions of the box are determined by the hand position
    active_range_box_.scale.x = latest_status_.live_hand_pose.pose.position.x * 2;
    active_range_box_.scale.y = latest_status_.live_hand_pose.pose.position.y * 2;
    active_range_box_.scale.z = latest_status_.live_hand_pose.pose.position.z * 2;
    // If setting pose in one plane only, give the aid box thickness of the arrow
    if (!latest_status_.position_forward_only && !latest_status_.position_unlimited) active_range_box_.scale.z = displacement_arrow_.scale.x;
    // Coloring the active range box
    active_range_box_.color = displacement_arrow_.color;		// use the same color as arrow
    active_range_box_.color.a = 0.2;					// make it visible

    // Publish the active_range_box_
    marker_publisher.publish( active_range_box_ );
      
    /* ==  TEXT LABEL  ======================================= */
    // Update display_distance parameters (display_distance operates relative to leap_motion frame)
    // Get the distance between start and end point as string
    distance_as_text_.text = getDistanceString( displacement_arrow_.points );
    if (!latest_status_.in_natural_control_mode)			// INVERTED CONTROL MODE
    {
      distance_as_text_.scale.z = 0.001 + latest_status_.scale_by/20;	// scale the display text based on scale_by value
      distance_as_text_.pose.position = displacement_arrow_.points[1];	// text is positioned at the end of the arrow marker
    }
    else								// NATURAL CONTROL MODE
    {
      distance_as_text_.scale.z = 0.010 + latest_status_.scale_by/20;	// scale the display text based on scale_by value
      distance_as_text_.pose.position = displacement_arrow_.points[1];	// text is positioned at the end of the arrow marker
      if (latest_status_.scale_by < 0.1 && camera_is_aligned_) distance_as_text_.scale.z = 0.001 + latest_status_.scale_by/20; // extreme close-up
    }
    // A tweak for when the camera is on the top facing down; lift text above the arrow
    if (!camera_is_aligned_) distance_as_text_.pose.position.y = 0.7*displacement_arrow_.scale.y; // lift text to the top of arrows head
    // A tweak for bringing the text in front of the hand pose orientation marker for better visibility
    if (latest_status_.orientation_free && camera_is_aligned_) distance_as_text_.pose.position.z += 0.1;   // shift text in front of the hand pose rotation marker

    // Publish distance_as_text_
    marker_publisher.publish( distance_as_text_ );

    /* ==  HAND POSE MARKER  ================================= */
    // Size of the hand pose marker
    hand_pose_marker_.scale.x = 0.06;
    hand_pose_marker_.scale.z = 0.15; 
    // Hand pose marker (relative to leap_motion frame)
    hand_pose_marker_.pose = latest_status_.live_hand_pose.pose;

    // Shift the marker in front of the ft sensor and robotiq gripper
    if (latest_status_.in_natural_control_mode) hand_pose_marker_.pose.position.z -= VIRTUAL_SCREEN_FRONT_;
    else hand_pose_marker_.pose.position.z += VIRTUAL_SCREEN_FRONT_;

    // Paint the marker based on restricted motion
    if (latest_status_.orientation_free) hand_pose_marker_.color.g = 0.0;	// if hand orientation is to be considered, paint the hand pose marker red 
    else hand_pose_marker_.color.g = 0.5;					// else, the marker is orange

    // SPECIAL CASE! Hide hand pose marker when orientation is to be ignored.
    if (!latest_status_.orientation_free) hand_pose_marker_.color.a = 0;	// make the marker invisible
    else hand_pose_marker_.color.a = 1;

    // Publish hand_pose_marker_
    marker_publisher.publish( hand_pose_marker_ );
      
    /* ==  CARTESIAN PATH ++ ================================= */
    // Empty any previous waypoints from the visualized cartesian path
    cartesian_path_.points.clear();

    // Take only position members of available cartesian wayposes and push them to cartesian_path.
    for (int i = 0; i < latest_status_.cartesian_wayposes.size(); ++i)
    {
      cartesian_path_.points.push_back( latest_status_.cartesian_wayposes.at(i).position );
    }

    // Publish cartesian_path_ as LINE_STRIP marker
    marker_publisher.publish( cartesian_path_ );
      
  } // end setting markers in MANIPULATION mode
    
  // ============================================================ 
  // ==  CAMERA POSE  ===========================================
  // ============================================================
  // Setting 'adjust_camera_' triggers repositioning of the point-of-view (POW) camera.
    
  // Adjust camera in NAVIGATION mode
  if (latest_status_.in_navigation_mode && adjust_camera_)
  {
    // During NAVIGATION camera moves relative to 'base_link' frame.
    changePOWCameraFrameTo( mobile_frame_ );

    ROS_INFO("NAVIGATION/NATURAL: Switching to top view.");

    // For NAVIGATION/NATURAL +X is considered to be 'UP'.
    point_of_view_.up.vector.x = 1;
    point_of_view_.up.vector.z = 0;

    // Camera is positioned directly above the origin of base_link
    point_of_view_.eye.point.x = 0;
    point_of_view_.eye.point.y = 0;
    point_of_view_.eye.point.z = 2 + 10*latest_status_.scale_by;	// Never closer than 2 m, max distance at 12 m

    // Focus camera at the origin of base_link
    point_of_view_.focus.point.x = 0;

    latest_known_camera_mode_ = 11;					// set latest_known_camera_mode to 11, i.e navigation natural
    pow_publisher.publish( point_of_view_ );				// publish the modified CameraPlacement message
    adjust_camera_ = false;						// set adjust_camera 'false'     
  }
  // Adjust camera in MANIPULATION mode
  else if (!latest_status_.in_navigation_mode && adjust_camera_)
  {
    // During MANIPULATION camera akways moves relative to 'temoto_end_effector' frame.
    changePOWCameraFrameTo( eef_frame_ );

    // Adjust camera for NATURAL CONTROL MODE
    if (latest_status_.in_natural_control_mode)
    {
      if (camera_is_aligned_)					// here alignment means the so-called bird's view
      {
	ROS_INFO("NATURAL: Switching to aligned view.");
	// Set +Z as 'UP'
	point_of_view_.up.vector.x = 0;
	point_of_view_.up.vector.z = 1;

	// Camera will be behind temoto_end_effector, somewhat elevated
	point_of_view_.eye.point.x = -2*latest_status_.scale_by;// Distance backwards from the end effector
	point_of_view_.eye.point.y = 0;				// Align with end effector
	point_of_view_.eye.point.z = 0.2 + 2*latest_status_.scale_by;// Distance upwards from the end effector
	// if constrained to a plane and scaled down align camrea with the end effector in z-direction
	if (!latest_status_.position_unlimited && latest_status_.scale_by < 0.1) point_of_view_.eye.point.z = 0;

	// Look at the distance of VIRTUAL_VIEW_SCREEN from the origin temoto_end_effector frame, i.e. the palm of robotiq gripper
	point_of_view_.focus.point.x = VIRTUAL_SCREEN_FRONT_;
      }
      else							// natural control mode top view
      {
	ROS_INFO("NATURAL: Switching to top view.");
	// In the latest_known_camera_mode = 1;top view of natural control mode, +X is considered to be 'UP'.
	point_of_view_.up.vector.x = 1;
	point_of_view_.up.vector.z = 0;

	// Camera is positioned directly above the virtual FRONT view screen.
	point_of_view_.eye.point.x = VIRTUAL_SCREEN_FRONT_;				// Above the virtual FRONT view screen
	point_of_view_.eye.point.y = 0;
	point_of_view_.eye.point.z = VIRTUAL_SCREEN_TOP_ + 1.5*latest_status_.scale_by;	// Never closer than virtual TOP view screen, max distance at 1.5 m

	// Look at the distance of VIRTUAL_VIEW_SCREEN from the origin temoto_end_effector frame, i.e. look at virtual FRONT view screen
	point_of_view_.focus.point.x = VIRTUAL_SCREEN_FRONT_;
      }
      
      latest_known_camera_mode_ = 1;				// set latest_known_camera_mode to 1, i.e natural
      pow_publisher.publish( point_of_view_ );			// publish a CameraPlacement msg
      adjust_camera_ = false;					// set adjust_camera 'false'
    } // if adjust_camera in_natural_control_mode
      
    // adjust camera for INVERTED CONTROL MODE
    if (!latest_status_.in_natural_control_mode)
    {
      if (camera_is_aligned_)					// here alignment means the camera is in the front facing the end effector
      {
	ROS_INFO("INVERTED: Switching to aligned view.");
	// Set +Z as 'UP'
	point_of_view_.up.vector.z = 1;
	point_of_view_.up.vector.x = 0;

	// Position camera on the x-axis no closer than virtual FRONT view screen, max distance at 1.5 m.
	point_of_view_.eye.point.x = VIRTUAL_SCREEN_FRONT_ + 1.5*latest_status_.scale_by;
	point_of_view_.eye.point.y = 0;				// move camera to align with end effector along the y-axis
	point_of_view_.eye.point.z = 0;				// move camera to align with end effector along the z-axis
	if (latest_status_.position_unlimited) point_of_view_.eye.point.z = 0.1 + 1*latest_status_.scale_by;	// shift camera upwards ... 

	// Look at the origin of temoto_end_effector, i.e. all zeros
	point_of_view_.focus.point.x = VIRTUAL_SCREEN_FRONT_;
      }
      else							// else means camera should be in the top position
      {
	ROS_INFO("INVERTED: Switching to top view.");
	// In the top view of inverted control mode, -X is considered 'UP'.
	point_of_view_.up.vector.z = 0;
	point_of_view_.up.vector.x = -1;

	// Camera is positioned directly above the virtual FRONT view screen.
	point_of_view_.eye.point.x = VIRTUAL_SCREEN_FRONT_;				// Above the virtual FRONT view screen
	point_of_view_.eye.point.y = 0;
	point_of_view_.eye.point.z = VIRTUAL_SCREEN_TOP_ + 1.5*latest_status_.scale_by;	// Never closer than virtual TOP view screen, max distance at 1.5 m
	  
	// Look at the distance of VIRTUAL_VIEW_SCREEN from the origin temoto_end_effector frame, i.e. look at virtual FRONT view screen
	point_of_view_.focus.point.x = VIRTUAL_SCREEN_FRONT_;
      }
      
      latest_known_camera_mode_ = 2;				// set latest_known_camera_mode to 2, i.e inverted
      pow_publisher.publish( point_of_view_ );		// publish a CameraPlacement msg
      adjust_camera_ = false;					// set adjust_camera 'false'
    } // if adjust_camera not in_natural_control_mode
  } // else if (!latest_status.in_navigation_mode && adjust_camera)
    
  // Doublecheck    
  // If camera IS NOT in natural mode but system IS in natural mode, try adjusting the pow camera.
  if (latest_known_camera_mode_ != 1 && latest_status_.in_natural_control_mode)
  {
    adjust_camera_ = 1;
    // unless in recognized NAVIGATION mode, then all was OK
    if (latest_known_camera_mode_ == 11 && latest_status_.in_navigation_mode) adjust_camera_ = 0;
  }
  // If camera IS NOT in inverted mode but the system IS in inverted mode, try adjusting the pow camera.
  if (latest_known_camera_mode_ != 2 && !latest_status_.in_natural_control_mode) adjust_camera_ = 1;
  // If camera IS NOT in NAVIGATION natural mode but system IS in NAVIGATION natural mode, try adjusting the pow camera.
  if (latest_known_camera_mode_ != 11 && latest_status_.in_navigation_mode) adjust_camera_ = 1;
} // end Visuals::crunch()

/** Main method. */
int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "rviz_visual");
  
  // ROS node handle
  ros::NodeHandle n;
  
  // Setting the node rate at 1 kHz
  ros::Rate node_rate(1000);
  
  // Get all the relevant frame names from parameter server
  std::string human, end_effector, mobile_base;
  ROS_INFO("[rviz_visual] Getting frame names from parameter server.");
  n.param<std::string>("/temoto_frames/human_input", human, "leap_motion");
  ROS_INFO("[rviz_visual] Human frame is: %s", human.c_str());
  n.param<std::string>("/temoto_frames/end_effector", end_effector, "temoto_end_effector");
  ROS_INFO("[rviz_visual] End-effector frame is: %s", end_effector.c_str());
  n.param<std::string>("/temoto_frames/mobile_base", mobile_base, "base_link");
  ROS_INFO("[rviz_visual] Mobile base frame is: %s", mobile_base.c_str());
  
  Visuals rviz_visuals(human, end_effector, mobile_base);

  // Set up service /temoto/adjust_rviz_camera; if there's a service request, executes adjustCameraPlacement() function
  ros::ServiceServer srv_visuals = n.advertiseService("temoto/adjust_rviz_camera", &Visuals::adjustPOWCameraPlacement, &rviz_visuals);
  
  // ROS subscriber on /temoto/status
  ros::Subscriber sub_status = n.subscribe("temoto/status", 1, &Visuals::updateStatus, &rviz_visuals);

  // ROS subscriber on /griffin_powermate/events.
  ros::Subscriber sub_powermate = n.subscribe("/griffin_powermate/events", 1, &Visuals::powermateWheelEvent, &rviz_visuals); 
  
  // Publisher of CameraPlacement messages (this is picked up by rviz_animated_view_controller).
  // When latch is true, the last message published is saved and automatically sent to any future subscribers that connect. Using it to set camera during rviz startup.
  ros::Publisher pub_pow_camera = n.advertise<view_controller_msgs::CameraPlacement>( "/rviz/camera_placement", 3, true );

  // Publisher on /visualization_marker to depict the hand pose with several rviz markers (this is picked up by rviz)
  ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
 
  // Publish the initial CameraPlacement; so that rviz_animated_view_controller might _latch_ on to it
  pub_pow_camera.publish( rviz_visuals.point_of_view_ );
  
  while ( ros::ok() )
  {
    // Update point-of-view camera pose and all the visualization markers
    rviz_visuals.crunch(pub_marker, pub_pow_camera);
    
    // Spin
    ros::spinOnce();
    
    // Sleep to meet the node_rate
    node_rate.sleep();
  }
  
} // end main()
