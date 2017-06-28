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


#ifndef AW_RNDF_CROSSWALK_H
#define AW_RNDF_CROSSWALK_H

#include <string>
#include <map>
#include <cmath>
#include <aw_wayPoint.h>

namespace vlr {

namespace rndf {

typedef enum { incoming_waypoint, stop_waypoint } crosswalk_linktype;

class WayPoint;
class Crosswalk;

class CrosswalkLink {
public:

 Crosswalk* crosswalk_;
 crosswalk_linktype type_;
};

class Crosswalk : public NetElement {
 public:
  Crosswalk(uint32_t id, const std::string& strName);
  ~Crosswalk();
  Crosswalk(const Crosswalk &c);
  Crosswalk& operator=(const Crosswalk &c);

  inline double lat1() const { return lat1_; }
  inline double lon1() const { return lon1_; }
  inline double utmX1() const { return utm_x1_; }
  inline double utmY1() const { return utm_y1_; }
  inline double lat2() const { return lat2_; }
  inline double lon2() const { return lon2_; }
  inline double utmX2() const { return utm_x2_; }
  inline double utmY2() const { return utm_y2_; }
  inline const std::string& utmZone() const { return utm_zone_; }

  inline double width() const { return width_; }
  inline void width(double w) { width_ = w; }

  double orientation() const {return atan2(utm_y2_ - utm_y1_, utm_x2_ - utm_x1_);}

  void centerLatLon(double& clat, double& clon) const;

  bool addWayPoint(WayPoint* w);
  void removeWayPoint(WayPoint* w);
  const std::map<std::string, WayPoint*>& linkedWayPoints() {return linked_waypoints_;}

  void setLatLon1(double lat, double lon);
  void setLatLon2(double lat, double lon);
  void setUtm1(double utm_x, double utm_y, const std::string& utm_zone);
  void setUtm2(double utm_x, double utm_y, const std::string& utm_zone);

  void dump() const;

 protected:
  double lat1_, lon1_;
  double lat2_, lon2_;
  double utm_x1_, utm_y1_;
  double utm_x2_, utm_y2_;
  std::string utm_zone_;

  double width_;

  std::map<std::string, WayPoint*> linked_waypoints_;

  friend std::ostream& operator<<(std::ostream& os, const Crosswalk& cw);
};


// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Crosswalk& cw);

typedef std::map<std::string, CrosswalkLink> TCrosswalkLinkMap;
typedef std::map<std::string, Crosswalk*> TCrosswalkMap;
} // namespace rndf

} // namespace vlr

#endif // AW_RNDF_CROSSWALK_H
