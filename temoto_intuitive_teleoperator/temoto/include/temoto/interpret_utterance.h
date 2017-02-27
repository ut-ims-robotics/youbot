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

/** @file interpret_utterance.h
 *  Subscribes to "pocketsphinx_recognizer/output" topic and tries to extract valid voice commands
 *  from it. If valid voice command is extracted, an approproate command code is published on
 *  "temoto/voice_commands".
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sound_play/sound_play.h"

// temoto includes
#include "temoto/temoto_common.h"

// Other includes
#include "string.h"


class Interpreter
{
public:
  /** Constructor */
  Interpreter()
  {
  };
  
  /** Maps verbal instructions to specific command code. */
  std::map<std::string, uint8_t> command_map_ =
  {
    {"stop stop", 0x00},		// stop or abort command
    {"robot please plan", 0x01},	// command PLAN
    {"robot please execute", 0x02},	// command EXECUTE plan
    {"robot please go", 0x03},		// command PLAN&EXECUTE
    {"robot please plan home", 0xf1},	// command PLAN to a saved home pose
    {"robot please go home", 0xf3},	// command PLAN&EXECUTE the home pose
    {"natural control mode", 0x10},	// control mode determines whether operator has natural or inverted view
    {"inverted control mode", 0x11},	// control mode determines whether operator has natural or inverted view
    {"free directions", 0x20},		// complete position of hand is used
    {"limit directions", 0x21},		// some directions may be limited
    {"consider rotation", 0x22},	// factors in hand orientation
    {"ignore rotation", 0x23},		// hand orientation is ignored, i.e. using hand position only
    {"compute cartesian", 0x31},	// computes cartesian path based on waypoints
    {"execute cartesian", 0x32},	// executes the cartesian path
    {"cartesian go", 0x33},		// computes and executes cartesian path
    {"new", 0x34},			// first point in waypoints
    {"add", 0x35},			// adds a point into waypoints
    {"remove last", 0x36},		// removes the last waypoint
    {"cancel cartesian", 0x37},		// clears all the cartesian waypoints
    {"manipulation", 0x40},		// operator controls robot manipulator (MoveIt!)
    {"navigation", 0x41},		// operator navigates the robot base (ROS_navigation)
    
    {"okay robot execute", 0x66}	// demo testing: placeholder command for some subtask
  };
  
  /** Publisher for recognized voice commands. */
  ros::Publisher pub_voice_commands_;
  
  /** Subscriber for utterance strings. */
  ros::Subscriber sub_utterances_;
  
  /** Instance of SoundClient used for text-to-speech synthesis. */
  sound_play::SoundClient sound_client_;
  
  /** Callback for subscribed utterances. */
  void utteranceToRecognizedCommand (std_msgs::String utterance);
  
  /** Prints out all the keys in command_map_. */
  void displayRecognizedVoiceCommands();
};