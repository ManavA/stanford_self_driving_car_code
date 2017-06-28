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
#include "aw_SpeedLimit.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {

SpeedLimit::SpeedLimit(uint32_t id, const std::string& name="Default speed limit") :
                            NetElement(id, name), min_speed_(0), max_speed_(0),
                            ref_is_lane_(false), ref_is_segment_(false), ref_is_zone_(false),
                            lane_(NULL), segment_(NULL), zone_(NULL), override(false) {
}

SpeedLimit::~SpeedLimit() {
}

void SpeedLimit::setLane(Lane* lane) {
  ref_is_lane_ = true;
  ref_is_segment_ = false;
  ref_is_zone_ = false;
  lane_ = lane;
  segment_ = NULL;
  zone_ = NULL;
  lane->setSpeedLimit(this);
}

void SpeedLimit::setSegment(Segment* segment) {
  ref_is_lane_ = false;
  ref_is_segment_ = true;
  ref_is_zone_ = false;
  lane_ = NULL;
  segment_ = segment;
  zone_ = NULL;
  segment->setSpeedLimit(this);
}

void SpeedLimit::setZone(Zone* zone) {
  ref_is_lane_ = false;
  ref_is_segment_ = false;
  ref_is_zone_ = true;
  lane_ = NULL;
  segment_ = NULL;
	zone_ = zone;
	zone_->setSpeedLimit(this);
}


void SpeedLimit::dump() {
	std::cout << "  speed limit for " << (ref_is_zone_ ? "zone " : (ref_is_segment_ ? "segment " : "lane ")) << ref_name_ << ": [" << min_speed_ << ", " << max_speed_ << "]" << std::endl;
}

}

} // namespace vlr
