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


#ifndef _AW_SPOT_H_
#define _AW_SPOT_H_

#include <aw_RndfId.h>
#include <aw_wayPoint.h>
#include <aw_checkPoint.h>


namespace vlr {

namespace rndf
{
class Zone;
class Spot;

typedef std::map<std::string, Spot*> TSpotMap;


class Spot : public NetElement
{
public:
	Spot(uint32_t id, const std::string& strName);
	virtual ~Spot(void);

	uint32_t wayPointIndex(const WayPoint* wp) const;

	void setZone(Zone* z) { parent_zone_ = z; }
	Zone* zone() const {return parent_zone_;}

	void setSpotWidth(int spot_width) { spot_width_=spot_width;}
	int getSpotWidth() {return spot_width_;}

	bool addWayPoint(WayPoint* wp);
	void removeWayPoint(uint32_t index);
	void removeWayPoint(WayPoint* wp);

	const TWayPointVec& wayPoints() { return waypoints_; }
	const TCheckPointMap& checkPoints() { return checkpoints_;}

	bool centerLatLon(double& clat, double& clon) const;

  inline std::string nextSpotPointStr() const {
    return name() + "." + nextIdStr(waypoints_);
  }

	uint32_t numSpotPoints() { return waypoints_.size(); } // should alway be 2

	void dump();

private:
	int spot_width_;

	TWayPointVec      waypoints_;
	TCheckPointMap    checkpoints_;

	Zone* parent_zone_;

	friend std::ostream& operator<<(std::ostream& os, const Spot& s);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Spot& s);
};

} // namespace vlr

#endif


