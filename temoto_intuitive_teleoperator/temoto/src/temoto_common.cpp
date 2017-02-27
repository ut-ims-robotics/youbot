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

/** @file temoto_common.cpp
 * 
 *  @brief Library for common functions in temoto.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include "temoto/temoto_common.h"

// This is the file where I put functions that more than one node might need.

/** Calculates the distance between two points [(x1,y1,z1), (x2,y2,x2)] in a 3D space.
 *  @param twoPointVector vector containing 2 geometry_msgs::Point points.
 *  @return distance between the two points.
 */
double calculateDistance (std::vector <geometry_msgs::Point> & twoPointVector) {
  // Calculate the distance between the first two points in the input vector of points
  double distance = sqrt( pow(twoPointVector[1].x-twoPointVector[0].x, 2) + pow(twoPointVector[1].y-twoPointVector[0].y, 2) + pow(twoPointVector[1].z-twoPointVector[0].z, 2));

  return distance;
}

// Create a method that takes geometry_msgs::Quaternion, rotates in by a rotation Quaternion and returns geometry_msgs::Quaternion
geometry_msgs::Quaternion combineQuaternions(geometry_msgs::Quaternion q0, geometry_msgs::Quaternion q1) {
  
  // Three empty instances of tf::Quaternion
  tf::Quaternion tf_q0, tf_q1, tf_combined;

  // Convert incoming geometry_msgs::Quaternion to tf::Quaternion
  tf::quaternionMsgToTF(q0, tf_q0);
  tf::quaternionMsgToTF(q1, tf_q1);
  
  // Normalize the two input quaternions.
  tf_q0.normalize();
  tf_q1.normalize();
  
  // Multiply the two input quaternions.
  tf_combined = tf_q0*tf_q1;
  
  // Normalize the multiplication result.
  tf_combined.normalize();
  
  geometry_msgs::Quaternion combined;
  tf::quaternionTFToMsg(tf_combined, combined);
  return combined;
}