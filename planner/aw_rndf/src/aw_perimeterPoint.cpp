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
#include <iomanip>
#include <string.h>

#include <aw_roadNetwork.h>
#include <aw_perimeterPoint.h>

using namespace std;

namespace vlr {

namespace rndf {

PerimeterPoint::PerimeterPoint(uint32_t id, const string& strName) :
  NetElement(id, strName), parent_perimeter_(NULL), lat_(0), lon_(0), utm_x_(0), utm_y_(0) {
}

PerimeterPoint::~PerimeterPoint() {
}

void PerimeterPoint::addExit(Exit* e) {
  if (!e) return;
  if (exits_.find(e->name()) != exits_.end()) return;
  if (parent_perimeter_) parent_perimeter_->addExit(e);
  exits_.insert(make_pair(e->name(), e));
}

void PerimeterPoint::removeExit(Exit* e) {
  if (!e) return;
  if (exits_.find(e->name()) == exits_.end()) return;
  if (parent_perimeter_) {parent_perimeter_->removeExit(e);}
  exits_.erase(e->name());
}

void PerimeterPoint::addEntry(Exit* e) {
  entries_.insert(make_pair(e->name(), e));
}

void PerimeterPoint::removeEntry(Exit* e) {
  entries_.erase(e->name());
}

void PerimeterPoint::setLatLon(double lat, double lon) {
  lat_ = lat;
  lon_ = lon;
  latLongToUtm(lat_, lon_, &utm_x_, &utm_y_, utm_zone_);
}

void PerimeterPoint::setUtm(double utm_x, double utm_y, const std::string& utm_zone) {
  utm_x_ = utm_x;
  utm_y_ = utm_y;
  utm_zone_ = utm_zone;

  utmToLatLong(utm_x, utm_y, utm_zone, &lat_, &lon_);
}

uint32_t PerimeterPoint::index() const {
  if (parent_perimeter_) {
    return parent_perimeter_->perimeterPointIndex(this);
  }

  return 0;
}

void PerimeterPoint::dump() {
  cout << "  PerimeterPoint " << name() << " " << utm_x_ << " " << utm_y_ << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const PerimeterPoint& p) {
  os << p.name_ << " " << setprecision(16) << p.lat_ << " " << p.lon_ << endl;
  return os;
}
}

} // namespace vlr

