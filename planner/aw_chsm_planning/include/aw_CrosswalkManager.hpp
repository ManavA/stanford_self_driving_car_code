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


#ifndef AW_CROSSWALKMANAGER_H
#define AW_CROSSWALKMANAGER_H

#include <string>
#include <set>
#include <map>

#include <aw_roadNetwork.h>
#include <obstaclePrediction.h>

namespace vlr {

#define CROSSWALK_MIN_FREE_TIME 1.5

class Topology;

class CrosswalkManager {
public:
	CrosswalkManager(Topology* top);
	~CrosswalkManager();

	bool isOccupied(std::vector<ObstaclePrediction>& pedestrians);
  inline static void minFree(double t) {min_free_time_ = t;}
  inline static double minFree() {return min_free_time_;}

 private:
  bool pedestriansOnCrosswalk(rndf::Crosswalk* crosswalk, std::vector<ObstaclePrediction>& pedestrians);

 private:
	Topology* top_;
	RndfGraph* graph_;
	std::vector<rndf::Crosswalk*> crosswalks_;
  double last_occupied_;
  static double min_free_time_;
};

} // namespace vlr

#endif // AW_CROSSWALKMANAGER_H
