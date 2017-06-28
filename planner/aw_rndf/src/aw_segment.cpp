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


/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#include <iostream>
#include <algorithm>

#include "aw_segment.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {
Segment::Segment(uint32_t id, const string& strName) :
  NetElement(id, strName), speed_limit_(NULL), lat_sum_(0), lon_sum_(0), length_(0), offroad_(false) {
}

Segment::~Segment(void) {
}

void Segment::addLane(Lane* l) {
  if (lanes_.find(l) != lanes_.end()) return;
  lanes_.insert(l);

  double lat, lon;
  if (l->centerLatLon(lat, lon)) {
    lat_sum_ += lat;
    lon_sum_ += lon;
  }
}

void Segment::removeLane(Lane* l) {
  if (lanes_.find(l) == lanes_.end()) return;
  lanes_.erase(l);

  double lat, lon;
  if (l->centerLatLon(lat, lon)) {
    lat_sum_ -= lat;
    lon_sum_ -= lon;
  }
}

Lane* Segment::getLaneById(uint32_t id) {
  for (TLaneSet::iterator it = lanes_.begin(); it != lanes_.end(); ++it)
    if ((*it)->id() == id) return *it;
  return NULL;
}

bool Segment::centerLatLon(double& clat, double& clon) const {
  double latSum = 0, lonSum = 0;
  uint32_t num = 0;

  if (lanes_.size() == 0) {
    return false;
  }

  TLaneSet::iterator lit, lit_end;
  for (lit = lanes_.begin(), lit_end = lanes_.end(); lit != lit_end; ++lit) {
    if ((*lit)->centerLatLon(clat, clon)) {
      latSum += clat;
      lonSum += clon;
      num++;
    }
  }

  clat = latSum / num;
  clon = lonSum / num;
  return true;
}

void Segment::dump() {
  cout << "----------------------------------------" << endl;
  cout << "Segment: " << name() << endl;
  cout << "description: " << description_ << endl;
  cout << "# of lanes: " << lanes_.size() << endl;
  if (speed_limit_) {
    cout << "Speedlimit:" << endl;
    speed_limit_->dump();
  }
  TLaneSet::iterator it, it_end;
  for (it = lanes_.begin(), it_end = lanes_.end(); it != it_end; ++it) {
    (*it)->dump();
  }
  cout << "----------------------------------------" << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Segment& s) {

  os << RNDF_SEGMENT_BEGIN << " " << s.name_ << endl;
  os << RNDF_SEGMENT_NAME << " " << s.description_ << endl;
  if (s.speed_limit_) {
    os << RNDF_SEGMENT_SPEED_LIMIT << " " << s.speed_limit_->maxSpeed() << " " << s.speed_limit_->minSpeed() << endl;
  }
  os << RNDF_SEGMENT_NUM_LANES << " " << s.lanes_.size() << endl;

  TLaneSet::const_iterator it, it_end;
  for (it = s.lanes_.begin(), it_end = s.lanes_.end(); it != it_end; ++it)
    os << **it;

  os << RNDF_SEGMENT_END << endl;

  return os;
}
}

} // namespace vlr

