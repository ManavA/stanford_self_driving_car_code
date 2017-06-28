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


#include <iostream>
#include <trafficlights.h>

using namespace std;
using namespace vlr::rndf;

namespace vlr {
TrafficLightSimulator::TrafficLightSimulator(RoadNetwork& rn, ros::NodeHandle& nh) :
  nh_(nh), state_(0), lastTime_(0.), switching_(false) {

  readParameters();

  TrafficLight light;
  light.state.confidence = 1.;
  light.state.state_arrow = 'n';
  light.state.state = 'z';

  const TTrafficLightMap& lights = rn.trafficLights();
  TTrafficLightMap::const_iterator tlit = lights.begin(), tlit_end = lights.end();
  for (; tlit != tlit_end; tlit++) {
    std::string name = (*tlit).first;
    vlr::rndf::TrafficLight* tl = (*tlit).second;
    light.state.name = name;
    light.group = tl->groupId();
    lights_.push_back(light);
  }

  pub_ = nh.advertise<driving_common::TrafficLightStates> ("TrafficLightStates", 5);
}

TrafficLightSimulator::~TrafficLightSimulator() {
}

void TrafficLightSimulator::update(double time) {

  if(!params_.switch_light_states_) {return;}

  std::vector<TrafficLight>::iterator it;

  if (time - lastTime_ > params_.state_duration - .5 * params_.yellow_duration && params_.yellow_duration > 0.
      && !switching_) {
    switching_ = true;
    int temp_state = state_ + 1;

    bool have_group = false;

    for (int i = 0; i < NUM_LIGHT_GROUPS; ++i, ++temp_state) {
      if (temp_state == NUM_LIGHT_GROUPS) temp_state = 0;

      for (it = lights_.begin(); it != lights_.end(); ++it) {
        if (it->group & (1 << temp_state)) {have_group = true;}
      }
      if (have_group) break;
    }
    if (!have_group)
      std::cerr << "ERROR: void TrafficLightSimulator::update(double time) no light groups defined!" << std::endl;
    else {
      std::cout << "Next traffic lights state: " << temp_state << std::endl;
      for (it = lights_.begin(); it != lights_.end(); ++it) {

        if (it->group & (1 << state_) && !(it->group & (1 << temp_state))) {
          it->state.state = 'y';
          std::cout << "Light " << it->state.name << " switched to YELLOW" << std::endl;
        }
      }
    }
  }

  if (time - lastTime_ > params_.state_duration + .5 * params_.yellow_duration) {
    switching_ = false;
    lastTime_ = time;
    ++state_;

    bool have_group = false;

    for (int i = 0; i < NUM_LIGHT_GROUPS; ++i, ++state_) {
      if (state_ == NUM_LIGHT_GROUPS) state_ = 0;

      for (it = lights_.begin(); it != lights_.end(); ++it) {
        if (it->group & (1 << state_)) {have_group = true;}
      }
      if (have_group) break;
    }
    if (!have_group)
      std::cerr << "ERROR: void TrafficLightSimulator::update(double time) no light groups defined!" << std::endl;
    else {
      std::cout << "New traffic lights state: " << state_ << std::endl;
      for (it = lights_.begin(); it != lights_.end(); ++it) {
        if (it->group & (1 << state_)) {
          it->state.state = 'g';
          std::cout << "Light " << it->state.name << " switched to GREEN" << std::endl;
        }
        else {
          std::cout << "Light " << it->state.name << " switched to RED" << std::endl;
          it->state.state = 'r';
        }
      }
    }
  }

  //publish light states

  for (std::vector<TrafficLight>::iterator it = lights_.begin(); it != lights_.end(); ++it) {
    light_states_.light_state.push_back(it->state);
  }

  pub_.publish(light_states_);
}

void TrafficLightSimulator::readParameters() {
  nh_.getParam("sim/traffic_light_switch_states", params_.switch_light_states_);
  nh_.getParam("sim/traffic_light_state_duration", params_.state_duration);
  nh_.getParam("sim/traffic_light_yellow_duration", params_.yellow_duration);
}

} // namespace vlr
