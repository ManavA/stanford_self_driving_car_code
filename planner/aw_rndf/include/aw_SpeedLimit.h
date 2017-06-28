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


#ifndef _SPEEDLIMIT_H_
#define _SPEEDLIMIT_H_

#include <iostream>
#include <aw_netElement.h>


namespace vlr {

namespace rndf {

class Segment;
class Zone;
class Lane;

class SpeedLimit : public NetElement
{
public:
	SpeedLimit(uint32_t id, const std::string& name);
	virtual ~SpeedLimit();

  inline double minSpeed() const {return min_speed_;}
  inline double maxSpeed() const {return max_speed_;}

	void setRefName(const std::string& name) { ref_name_ = name; }
  void setSegment(Segment * seg);
  void setLane(Lane* lane);
	void setZone(Zone * Zone);
	void minSpeed(double speed) { min_speed_ = speed; }
	void maxSpeed(double speed) { max_speed_ = speed; }

  const Lane* lane() const { return lane_; };
  const Segment* segment() const { return segment_; };
	const Zone* zone() const { return zone_; };

	void dump();

	void setOverride(bool o=true) { override = o; };
	bool isOverride() const { return override; };

private:
	double min_speed_;
	double max_speed_;

	std::string ref_name_;
	bool ref_is_lane_, ref_is_segment_, ref_is_zone_;
	Lane* lane_;
	Segment* segment_;
	Zone* zone_;
	bool override;
};

}

} // namespace vlr

#endif
