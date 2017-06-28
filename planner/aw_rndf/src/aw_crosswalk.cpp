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
#include <stdio.h>

#include <global.h>
#include <lltransform.h>

#include <aw_wayPoint.h>
#include <aw_crosswalk.h>

namespace vlr {

namespace rndf {

using namespace std;

Crosswalk::Crosswalk(uint32_t id, const std::string& strName) :
                          NetElement(id, strName), lat1_(0), lon1_(0), lat2_(0), lon2_(0),
                          utm_x1_(0), utm_y1_(0), utm_x2_(0), utm_y2_(0),
                          width_(0) { //, parentsegment_(NULL) {
}

Crosswalk::~Crosswalk() {
  for (TWayPointMap::iterator it = linked_waypoints_.begin(); it != linked_waypoints_.end(); ++it) {
    (*it).second->removeCrosswalk(this);
  }
}

Crosswalk::Crosswalk(const Crosswalk & c) : NetElement(c.id(), c.name()) {
  lat1_ = c.lat1_;
  lon1_ = c.lon1_;
  lat2_ = c.lat2_;
  lon2_ = c.lon2_;
  utm_x1_ = c.utm_x1_;
  utm_y1_ = c.utm_y1_;
  utm_x2_ = c.utm_x2_;
  utm_y2_ = c.utm_y2_;
  width_ = c.width_;
  utm_zone_ = c.utm_zone_;

  for (TWayPointMap::const_iterator it = c.linked_waypoints_.begin(); it != c.linked_waypoints_.end(); ++it) {
    linked_waypoints_.insert(*it);

  }
 // parentsegment_ = NULL;
}

Crosswalk& Crosswalk::operator=(const Crosswalk &c) {
  printf("TODO: copy operator!\n");
  lat1_ = c.lat1_;
  lon1_ = c.lon1_;
  lat2_ = c.lat2_;
  lon2_ = c.lon2_;
  utm_x1_ = c.utm_x1_;
  utm_y1_ = c.utm_y1_;
  utm_x2_ = c.utm_x2_;
  utm_y2_ = c.utm_y2_;
  width_ = c.width_;
  return *this;
}

//int Crosswalk::lookup_crosswalk_id() const
//{
//  int i;
//
//  for(i = 0; i < parentsegment()->num_crosswalks(); i++)
//    if(parentsegment()->Crosswalk(i) == this)
//      return i;
//
//  return -1;
//}

void Crosswalk::centerLatLon(double& clat, double& clon) const {
  utmToLatLong(0.5*(utm_x1_+utm_x2_), 0.5*(utm_y1_+utm_y2_), utm_zone_, &clat, &clon);
}

bool Crosswalk::addWayPoint(WayPoint* w) {
  if (!w) {return false;}
  if (linked_waypoints_.find(w->name()) != linked_waypoints_.end()) {return false;}

  linked_waypoints_.insert( make_pair(w->name(), w) );
  return true;
}

void Crosswalk::removeWayPoint(WayPoint* w) {
  if (!w) {return;}
  if (linked_waypoints_.find(w->name()) == linked_waypoints_.end()) {return;}

  linked_waypoints_.erase(w->name());
}

void Crosswalk::setLatLon1(double lat, double lon) {
  lat1_ = lat;
  lon1_ = lon;
  latLongToUtm(lat, lon, &utm_x1_, &utm_y1_, utm_zone_);
}

void Crosswalk::setLatLon2(double lat, double lon)
{
  lat2_ = lat;
  lon2_ = lon;
  latLongToUtm(lat, lon, &utm_x2_, &utm_y2_, utm_zone_);
}


void Crosswalk::setUtm1(double utm_x, double utm_y, const std::string& utm_zone)
{
  utm_x1_ = utm_x;
  utm_y1_ = utm_y;
  utm_zone_ = utm_zone;
  utmToLatLong(utm_x, utm_y, utm_zone_, &lat1_, &lon1_);
}

void Crosswalk::setUtm2(double utm_x, double utm_y, const std::string& utm_zone)
{
  utm_x2_ = utm_x;
  utm_y2_ = utm_y;
  utm_zone_ = utm_zone;
  utmToLatLong(utm_x, utm_y, utm_zone_, &lat2_, &lon2_);
}

void Crosswalk::dump() const
{
  std::cout << "  width: " << width() << endl;
  std::cout << "  first point: " << utmX1() << ", " << utmY1() << " (" << lat1() << ", " << lon1() << ") " << endl;
  std::cout << "  second point: " << utmX2() << ", " << utmY2() << " (" << lat2() << ", " << lon2() << ") " << endl;
  std::cout << "  # linked WayPoints: " << linked_waypoints_.size() << endl;

  TWayPointMap::const_iterator wit, wit_end;
  for(wit = linked_waypoints_.begin(), wit_end = linked_waypoints_.end(); wit != wit_end; ++wit)
    std::cout << *wit->second;
}

std::ostream& operator<<(std::ostream& os, const Crosswalk& cw) {

os << RNDF_CROSSWALK_BEGIN << " " << cw.name() << endl;
os << RNDF_CROSSWALK_WIDTH << " " << dgc::dgc_meters2feet(cw.width()) << endl;
os << RNDF_CROSSWALK_P1 << " " << setprecision(16) << cw.lat1() << " " << cw.lon1() << endl;
os << RNDF_CROSSWALK_P2 << " " << setprecision(16) << cw.lat2() << " " << cw.lon2() << endl;
os << RNDF_CROSSWALK_END << endl;

return os;
}

} // namespace rndf
} // namespace vlr
