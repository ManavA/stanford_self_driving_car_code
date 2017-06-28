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


#include <sys/types.h>
#include <algorithm>

#include <aw_Topology.hpp>

#include <aw_ChsmPlanner.hpp>
#include <aw_TrafficLightManager.hpp>

using namespace std;

namespace vlr {

#undef TRACE
//#define TRACE(str) cout << "[TrafficLightManager] " << str << endl;
#define TRACE(str)

TrafficLightManager::TrafficLightManager(Topology* top) : top_(top), graph_(NULL) {
	if(!top_) {throw VLRException("zero pointer to topology");}
	graph_ = top_->complete_graph;
  if(!graph_) {throw VLRException("zero pointer to complete graph");}

  std::vector<std::string> tl_names;
  top->distToNextTrafficLight(&tl_names, NULL);
  if(tl_names.empty()) {throw VLRException("could not determine associated traffic light names");}

  std::vector<std::string>::const_iterator tlnit=tl_names.begin(), tlnit_end=tl_names.end();
  for(; tlnit != tlnit_end; tlnit++) {
//    rndf::TrafficLight* tl = const_cast<rndf::RoadNetwork*>(&top->roadNetwork())->trafficLight(*tlnit);
//    if(tl) {traffic_light_names_.push_back(tl);}
    traffic_light_names_.push_back(*tlnit);
  }
  if(traffic_light_names_.empty()) {throw VLRException("could not determine associated traffic lights");}
}

TrafficLightManager::~TrafficLightManager() {
}

bool TrafficLightManager::hasToStop(std::map<std::string, driving_common::TrafficLightState>& traffic_light_states) {
  std::map<std::string, driving_common::TrafficLightState>::const_iterator tlsit = traffic_light_states.find(traffic_light_names_[0]);
  if (tlsit != traffic_light_states.end()) {
    printf("Current TLID: %s (%c)\n", traffic_light_names_[0].c_str(), (*tlsit).second.state);
    return (*tlsit).second.state != 'g';
  }

  std::cout << "Warning: state of traffic light " << traffic_light_names_[0] << " was requested but is unavailable...assuming red...\n";
  return true;
}

} // namespace vlr
