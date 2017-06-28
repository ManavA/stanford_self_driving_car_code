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


#ifndef AW_RNDF_WAYPOINT_H_
#define AW_RNDF_WAYPOINT_H_

#include <string>

#include "aw_netElement.h"
#include "aw_exit.h"
#include "aw_crosswalk.h"
#include "aw_trafficLight.h"

namespace vlr {

namespace rndf {

class Stop;
class Exit;
class Lane;
class Spot;
class CheckPoint;

class WayPoint : public NetElement
{
public:
  WayPoint(uint32_t id, const std::string& strName);
  virtual ~WayPoint();


  void setLatLon(double lat, double lon);
  void setUtm(double utm_x, double utm_y, const std::string& utm_zone);

  uint32_t index() const;

  void setCheckPoint(CheckPoint* cp);
  void setStop(Stop* sp);
  void addExit(Exit* ex);
  void removeExit(Exit* ex);
  void addEntry(Exit* ex);
  void removeEntry(Exit* ex);

  void setVirtual(bool is_virtual = true) {is_virtual_ = is_virtual;}
  bool isVirtual() const { return is_virtual_; }

  Lane* parentLane() {return parentlane_;}
  void setParentLane(Lane* lane) {parentlane_=lane;}

  Spot* parentSpot() {return parentspot_;}
  void setParentSpot(Spot* spot) {parentspot_=spot;}

  CheckPoint* checkPoint() {return checkpoint_;}
  Stop* stop() {return stop_;}
  TExitMap& exits() {return exits_;}
  TExitMap& entries() {return entries_;}
  TCrosswalkLinkMap& crosswalks() {return crosswalks_;}
  TTrafficLightMap& trafficLights() {return trafficlights_;}

  uint32_t nextCrosswalkId() const;
  std::string nextCrosswalkStr() const;
  void addCrosswalk(Crosswalk* cw, crosswalk_linktype type);
  void removeCrosswalk(Crosswalk* cw);

  uint32_t nextTrafficLightId() const;
  std::string nextTrafficLightStr() const;
  void addTrafficLight(TrafficLight* tl);
  void removeTrafficLight(TrafficLight* tl);

  double lat() { return lat_; };
  double lon() { return lon_; };
  double utmX() { return utm_x_; };
  double utmY() { return utm_y_; };
  const std::string& utmZone() { return utm_zone_; };
  double x() { return utm_x_; };
  double y() { return utm_y_; };

  virtual void dump() const;

protected:
  double lat_;
  double lon_;
  double utm_x_;
  double utm_y_;
  std::string utm_zone_;

  bool is_virtual_;

  TExitMap exits_;
  TExitMap entries_;
  CheckPoint* checkpoint_;
  Stop* stop_;
  TCrosswalkLinkMap crosswalks_;
  std::map<std::string, TrafficLight*> trafficlights_;

  Lane* parentlane_;
  Spot* parentspot_;

  friend std::ostream& operator<<(std::ostream& os, const WayPoint& w);

private:
  WayPoint(const WayPoint&);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const WayPoint& w);

typedef std::map<std::string, WayPoint*>    TWayPointMap;
typedef std::vector<WayPoint*>              TWayPointVec;
}

} // namespace vlr

#endif
