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


#ifndef INTERSECTION_H_
#define INTERSECTION_H_

#include <aw_lane.h>


namespace vlr {

namespace rndf {

class Intersection;

typedef std::set<Intersection*>	TIntersectionSet;


class Intersection : public rndf::NetElement
{
public:
	Intersection(uint32_t id, const std::string& strName);
	virtual ~Intersection();

	inline void addLaneSegment(LaneSegment* l) { LaneSegments.insert(l); }
	inline void remove(LaneSegment* l) { LaneSegments.erase(l); }

	inline const TLaneSegmentSet& laneSegments() const { return LaneSegments; }
	inline size_t numLaneSegments() const { return LaneSegments.size(); }

protected:
	TLaneSegmentSet	LaneSegments;
};

inline std::ostream& operator << (std::ostream& os, const rndf::Intersection& v)
{
	// sieht komisch ist aber so. Wer die Ursache dafür findet das bekommt ein Eis und darf es fixen.
    return os << v.name() << std::string(" ( ") << boost::lexical_cast<std::string>( v.numLaneSegments() ) << std::string(" lanes)");
//    return os << std::string(v.name()) << std::string(" ( ") << boost::lexical_cast<std::string>( v.numLaneSegments() ) << std::string(" lanes)");
}

} // namespace rndf

} // namespace vlr

#endif /*INTERSECTION_H_*/
