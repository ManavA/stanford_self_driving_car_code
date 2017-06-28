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


#ifndef AW_CHECKPOINT_H
#define AW_CHECKPOINT_H

#include <aw_netElement.h>

namespace vlr {

namespace rndf {

class WayPoint;

class CheckPoint : public NetElement {
public:
	friend class RoadNetwork;

	CheckPoint(uint32_t id, const std::string& strName);
	CheckPoint(const CheckPoint&);
	virtual ~CheckPoint(void);
	CheckPoint& operator=(const CheckPoint& other);
	CheckPoint& copy(const CheckPoint& other);

	WayPoint* wayPoint(void)	const {return m_waypoint;}
	void setWayPoint(WayPoint* wp)	{m_waypoint=wp;}
	void dump() const;

private:
	WayPoint* m_waypoint;

protected:
	friend std::ostream& operator<<(std::ostream& os, const CheckPoint& cp);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const CheckPoint& cp);

typedef std::map<std::string, CheckPoint*>   TCheckPointMap;

}

} // namespace vlr

#endif


