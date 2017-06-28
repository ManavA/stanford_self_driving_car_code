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


#ifndef AW_RNDF_TRAFFICLIGHT_H_
#define AW_RNDF_TRAFFICLIGHT_H_

#include <string>
#include <stdint.h>
#include <aw_wayPoint.h>

namespace vlr {
namespace rndf {

enum lightState {
  LIGHT_STATE_RED,
  LIGHT_STATE_YELLOW,
  LIGHT_STATE_GREEN,
  LIGHT_STATE_UNKNOWN,
};

class TrafficLight;

typedef std::map<std::string, TrafficLight*> TTrafficLightMap;

class TrafficLight : public NetElement {
 public:
  TrafficLight(uint32_t id, const std::string& strName);
  ~TrafficLight();
//  TrafficLight(const double& lat, const double& lon, const double& z);
  TrafficLight(const TrafficLight& c);
  TrafficLight& operator=(const TrafficLight &c);

  inline double lat() const { return lat_; }
  inline double lon() const { return lon_; }
  inline double utmX() const { return utm_x_; }
  inline double utmY() const { return utm_y_; }
  inline const std::string& utmZone() const {return utmzone_;}
  inline double orientation() const {return orientation_;}
  inline void orientation(double yaw) {orientation_ = yaw;}
//  void computeOrientation();

  double z() const { return z_; }
  void z(double z) { z_ = z; }

  uint32_t groupId() const { return group_id_; }
  void groupId(uint32_t id) { group_id_=id; }

  void setLatLon(double lat, double lon);
  void setUtm(double utm_x, double utm_y, std::string utmzone);

  bool addWayPoint(WayPoint *wp);
  void removeWayPoint(const WayPoint* wp);
  inline const std::map<std::string, WayPoint*>& linkedWayPoints() const {return linked_waypoints_;}

 private:

  std::map<std::string, WayPoint*> linked_waypoints_;

  double lat_, lon_;
  double utm_x_, utm_y_;
  std::string utmzone_;
  double z_;
  double orientation_;
  uint32_t group_id_;
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const TrafficLight& cw);

} // namespace rndf
} // namespace vlr

#endif // AW_RNDF_TRAFFICLIGHT_H_
