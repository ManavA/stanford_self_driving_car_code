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


#ifndef _EXIT_H_
#define _EXIT_H_

#include <aw_netElement.h>

namespace vlr {

namespace rndf
{
class Lane;
class Perimeter;
class WayPoint;
class PerimeterPoint;
class Exit;

typedef std::map<std::string, Exit*>	TExitMap;


class Exit : public NetElement
{
public:
	friend class RoadNetwork;

	enum eExitTypes {LaneToLane,LaneToPerimeter,PerimeterToLane, PerimeterToPerimeter};

	Exit(uint32_t id, const std::string& strName);
	Exit(const Exit&);
	virtual ~Exit(void);
	Exit& operator=(const Exit& other);
	Exit& copy(const Exit& other);

	void setExitType(Exit::eExitTypes exitType) { m_exitType = exitType; }
	eExitTypes exitType() { return m_exitType; }

	WayPoint* getExitFromLane() {return m_exitFromWayPoint;}
	PerimeterPoint* getExitFromPerimeter() {return m_exitFromPerimeterPoint;}
	WayPoint* getExitToLane() {return m_exitToWayPoint;}
	PerimeterPoint* getExitToPerimeter() {return m_exitToPerimeterPoint;}

	void setExitFrom(WayPoint* exitFrom) {m_exitFromWayPoint = exitFrom;}
	void setExitFrom(PerimeterPoint* exitFrom) {m_exitFromPerimeterPoint = exitFrom;}
	void setExitTo(WayPoint* exitTo) {m_exitToWayPoint = exitTo;}
	void setExitTo(PerimeterPoint* exitTo) {m_exitToPerimeterPoint = exitTo;}

	void dump() const;

private:
	WayPoint*       m_exitFromWayPoint;
	WayPoint*       m_exitToWayPoint;
	PerimeterPoint* m_exitFromPerimeterPoint;
	PerimeterPoint* m_exitToPerimeterPoint;
	eExitTypes      m_exitType;

protected:
	friend std::ostream& operator<<(std::ostream& os, const Exit& e);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Exit& e);

};

} // namespace vlr

#endif


