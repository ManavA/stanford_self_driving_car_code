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


#ifndef AW_SEGMENT_H_
#define AW_SEGMENT_H_

#include <aw_RndfId.h>
#include <aw_netElement.h>
#include <aw_SpeedLimit.h>
#include <aw_lane.h>

namespace vlr {

namespace rndf {

class RoadNetwork;

typedef std::map<std::string ,Segment*>	TSegmentMap;

class Segment : public NetElement
{
public:
	friend class rndf::RoadNetwork;

	Segment(uint32_t id, const std::string& strName);
	Segment(const Segment&);
	virtual ~Segment(void);

	Segment& operator=(const Segment& other);
	Segment& copy(const Segment& other);

	void setDescription(std::string description) { description_ = description; }
	std::string& getDescription(void) { return description_; }

	// add lanes
	void addLane(Lane* pLane);
	void removeLane(Lane* pLane);
	Lane* getLaneById(uint32_t id);
	uint32_t numLanes() { return lanes_.size(); }

	bool centerLatLon(double& clat, double& clon) const;
	//double& getLength(); // TODO: make it work ...

	void setSpeedLimit(SpeedLimit * limit) { speed_limit_ = limit; }
	SpeedLimit * speedLimit() { return speed_limit_; }
	SpeedLimit const * speedLimit() const { return speed_limit_; }

	void dump();

	const TLaneSet& getLanes() const { return lanes_; }

	inline std::string nextLaneStr() const {
	  return name() + "." + nextIdStr(lanes_);
	}

	void setOffroad() { offroad_ = true; }
	bool offroad() const { return offroad_; }

private:
	std::string description_;
	TLaneSet lanes_;
	SpeedLimit* speed_limit_;
	double lat_sum_, lon_sum_, length_;
	bool offroad_;

protected:
	friend std::ostream& operator<<(std::ostream& os, const Segment& rn);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Segment& rn);
}

} // namespace vlr

#endif

