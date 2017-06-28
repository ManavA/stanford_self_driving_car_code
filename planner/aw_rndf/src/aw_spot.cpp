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
#include <global.h>

#include "aw_spot.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {
Spot::Spot(uint32_t id, const string& strName) :
  NetElement(id, strName) {
}

Spot::~Spot(void) {
}

void Spot::dump() {
  cout << "Spot: " << name() << endl;
  cout << "  width: " << spot_width_ << endl;
  NetElement::dump();
  //cout << "  # of SpotPoints: " << m_SpotPoints.size() << endl;
  //cout << "  # of CheckPoints: " << checkpoints_.size() << endl;
  //TSpotPointMap::iterator it,it_end;
  //for(it = m_SpotPoints.begin(),it_end = m_SpotPoints.end(); it != it_end; ++it)
  //  (*it).second->dump();
  //cout << endl;;
}

uint32_t Spot::wayPointIndex(const WayPoint* wp) const {
  for (uint32_t i = 0; i < waypoints_.size(); ++i) {
    if (waypoints_[i] == wp) return i;
  }
  return waypoints_.size();
}

bool Spot::addWayPoint(WayPoint* pWayPoint) {
  waypoints_.push_back(pWayPoint);
  return true;
}

void Spot::removeWayPoint(uint32_t index) {
  if (index >= waypoints_.size()) return;
  waypoints_.erase(waypoints_.begin() + index);
}

void Spot::removeWayPoint(WayPoint* wp) {
  removeWayPoint(wayPointIndex(wp));
}

bool Spot::centerLatLon(double& clat, double& clon) const {
  if (waypoints_.size() == 0) {
    return false;
  }

  clat = 0.;
  clon = 0.;

  TWayPointVec::const_iterator wit, wit_end;

  for (wit = waypoints_.begin(), wit_end = waypoints_.end(); wit != wit_end; ++wit) {
    clat += (*wit)->lat();
    clon += (*wit)->lon();
  }

  clat /= waypoints_.size();
  clon /= waypoints_.size();

  return true;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Spot& s) {
  os << RNDF_SPOT_BEGIN << " " << s.name_ << endl;
  os << RNDF_SPOT_WIDTH << " " << dgc::dgc_meters2feet(s.spot_width_) << endl;

  TCheckPointMap checkpoints;

  TWayPointVec::const_iterator wpit = s.waypoints_.begin(), wpit_end = s.waypoints_.end();
  for (; wpit != wpit_end; wpit++) {
    CheckPoint* cp = (*wpit)->checkPoint();
    if (cp) {
      checkpoints.insert(std::make_pair(cp->name(), cp));
    }
  }
  TCheckPointMap::const_iterator cit, cit_end;
  for (cit = s.checkpoints_.begin(), cit_end = s.checkpoints_.end(); cit != cit_end; ++cit) {
    os << *cit->second;
  }

  TWayPointVec::const_iterator it, it_end;
  for (it = s.waypoints_.begin(), it_end = s.waypoints_.end(); it != it_end; ++it) {
    os << **it;
  }

  os << RNDF_SPOT_END << endl;

  return os;
}
}

} // namespace vlr

