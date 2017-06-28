/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#include <ros/ros.h>
#include <global.h>
#include <driving_common/TrafficLightStates.h>

using namespace driving_common;

int main(int argc, char **argv) {
  ros::init(argc, argv, "send_light_state");

  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <traffic light name> <traffic light color (r, y, g, u)>\n";
    ::exit(-5);
  }
  ros::NodeHandle nh("/driving");
  ros::Publisher tl_state_pub = nh.advertise<driving_common::TrafficLightStates> ("TrafficLightStates", 1);

    // send message
  TrafficLightStates light_states;
  TrafficLightState light_state;
  light_state.state = argv[2][0];
  light_state.name = argv[1];
  light_state.state_arrow = 'n';
  light_state.timestamp_rg_switch = 0.0; //time of switch from red to green
  light_state.timestamp_gy_switch = 0.0; //time of switch from green to red
  light_state.timestamp = driving_common::Time::current();
  light_state.confidence = 1.0;
  light_state.u = 50;                      //section of the camera image plane where the light is predicted to be located
  light_state.v = 50;
  light_states.light_state.push_back(light_state);
  tl_state_pub.publish(light_states);

  return 0;
}
