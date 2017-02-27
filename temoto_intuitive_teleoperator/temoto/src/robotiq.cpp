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

#include "ros/ros.h"
#include "robotiq_s_model_control/SModel_robot_output.h"
#include "stdio.h"

// == README ==
// This node simply initializes robotiq gripper. It sets it into a predefined position.
// This node may be expanded to be a server for gripper commands.

// NB! Output means "out from this machine" and to the robotiq

uint8_t gripper_cmd [19];		///< Working array that contains values needed for completing SModel_robot_output message

/** Converts the latest gripper_cmd into robotiq_s_model_control/SModel_robot_output message.
 *  @return robotiq's SModel_robot_output message.
 */
robotiq_s_model_control::SModel_robot_output retrieveCommandMessage () {
  robotiq_s_model_control::SModel_robot_output command;
  command.rACT = gripper_cmd[0];
  command.rMOD = gripper_cmd[1];
  command.rGTO = gripper_cmd[2];
  command.rATR = gripper_cmd[3];
  command.rGLV = gripper_cmd[4];
  command.rICF = gripper_cmd[5];
  command.rICS = gripper_cmd[6];
  command.rPRA = gripper_cmd[7];
  command.rSPA = gripper_cmd[8];
  command.rFRA = gripper_cmd[9];
  command.rPRB = gripper_cmd[10];
  command.rSPB = gripper_cmd[11];
  command.rFRB = gripper_cmd[12];
  command.rPRC = gripper_cmd[13];
  command.rSPC = gripper_cmd[14];
  command.rFRC = gripper_cmd[15];
  command.rPRS = gripper_cmd[16];
  command.rSPS = gripper_cmd[17];
  command.rFRS = gripper_cmd[18];
  return command;
}

/** Overwrites elements of gripper_cmd in correspondance to predefined_short_command.
 *  @param predefined_short_command is either 0 or a double digit decimal in the form of XY, where:
 *  X is the gripper mode (1-Basic; 2-Pinch);
 *  Y is 0 for gripper open and 1 for closed fingertip grip.
 *  For instance, 10 means basic mode opened whereas 21 means pinch mode closed fingertip grip.
 *  If predefined_short_command is 0, then reset.
 *  @return true when the input predefined_short_command was successfully interpreted; false otherwise.
 */
// http://support.robotiq.com/display/IMB/1.+General+Presentation
bool composeNextCommand (uint8_t predefined_short_command) {
  if (predefined_short_command == 0) {		// RESET
    for(int a = 0; a < 19; a++) {
      gripper_cmd[a] = 0;
    } //end for
    return true;
  } else if (predefined_short_command == 10) {	// BASIC MODE OPEN
    for(int a = 0; a < 19; a++) {
      gripper_cmd[a] = 0;
      if(a == 0 || a == 2) gripper_cmd[a] = 1;
      if(a == 8) gripper_cmd[a] = 255;
      if(a == 9) gripper_cmd[a] = 150;
    } // end for
    return true;
  } else if (predefined_short_command == 20) {	// PINCH MODE OPEN
    for(int a = 0; a < 19; a++) {
      gripper_cmd[a] = 0;
      if(a == 0 || a== 1 || a == 2) gripper_cmd[a] = 1;
      if(a == 8) gripper_cmd[a] = 255;
      if(a == 9) gripper_cmd[a] = 150;
    } // end for
    return true;
  } else if (predefined_short_command == 11) {	// FINGERTIP GRIP CLOSED IN BASIC MODE
    for(int a = 0; a < 19; a++) {
      gripper_cmd[a] = 0;
      if(a == 0 || a == 2) gripper_cmd[a] = 1;
      if(a == 7) gripper_cmd[a] = 110;		// set finger position to 110, which is half-way or closed.
      if(a == 8) gripper_cmd[a] = 255;
      if(a == 9) gripper_cmd[a] = 150;
    } // end for   
    return true;
  } else if (predefined_short_command == 21) {	// FINGERTIP GRIP CLOSED IN PINCH MODE
    for(int a = 0; a < 19; a++) {
      gripper_cmd[a] = 0;
      if(a == 0 || a== 1 || a == 2) gripper_cmd[a] = 1;
      if(a == 7) gripper_cmd[a] = 110;		// set finger position to 110, which is half-way or closed. 
      if(a == 8) gripper_cmd[a] = 255;
      if(a == 9) gripper_cmd[a] = 150;
    } // end for
    return true;
  }
  ROS_INFO("[temoto/robotiq] Failed to interpret predefine_command.");
  return false;
}


void initRobotiq(ros::Publisher &robotiq_publisher) {
  bool cmd_ok = composeNextCommand(0);				// Compose a RESET command.
  robotiq_publisher.publish( retrieveCommandMessage() );	// Convert the last command into an output command message.
  sleep(1);							// Sleep on it.
//   cmd_ok = composeNextCommand(10);				// Compose a command for BASIC MODE OPEN
//   pub_robotiq_gripper.publish(retrieveCommandMessage());	// Convert the last command into an output command message.
//   sleep(10);							// Sleep on it.
  cmd_ok = composeNextCommand(11);				// Compose a command for BASIC MODE FINGERTIP GRIP
  robotiq_publisher.publish( retrieveCommandMessage() );	// Convert the last command into an output command message.
  sleep(10);							// Sleep on it.
  return;
}

/** Main method. */
int main( int argc, char** argv ) {
  // ROS init
  ros::init(argc, argv, "robotiq");
  // ROS node handle
  ros::NodeHandle nh;
  // Publisher for "SModelRobotOutput"; robotiq_s_model_control/SModelTcpNode.py picks up these messages and sends them to robotiq
  ros::Publisher pub_robotiq_gripper = nh.advertise<robotiq_s_model_control::SModel_robot_output>("SModelRobotOutput", 1, true);

  // Wait for subscribers
  ROS_INFO("[robotiq] Waiting for subscribers of SModelRobotOutput topic ...");
  while (pub_robotiq_gripper.getNumSubscribers() < 1) {
    printf("...\n");
    ros::spinOnce();
  }
  ROS_INFO("[robotiq] Subscribers present! Moving forward ...");
  ROS_INFO("[robotiq] Waiting 5 seconds because ROS.");
  sleep(5);
  ROS_INFO("[robotiq] Done waiting! Moving actually forward :)");
  
  initRobotiq(pub_robotiq_gripper);
  
  ROS_INFO("[robotiq] Work is done! Exiting."); 
} // end main