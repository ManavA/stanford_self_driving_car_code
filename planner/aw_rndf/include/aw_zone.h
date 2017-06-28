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
#ifndef AW_ZONE_H
#define AW_ZONE_H

#include "aw_SpeedLimit.h"
#include "aw_perimeter.h"
#include "aw_spot.h"


namespace vlr {

namespace rndf
{
class Zone;

typedef std::map<std::string,Zone*>			TZoneMap;


class Zone : public NetElement
{
public:
	Zone(uint32_t id, const std::string& strName);
	Zone(const Zone&);
	virtual ~Zone(void);

	Zone& operator=(const Zone& other);
	Zone& copy(const Zone& other);

	void setDescription(std::string description) {
		m_description = description;
	}

	// add Perimeter
	bool addPerimeter(Perimeter* pPerimeter);
	void removePerimeter(Perimeter* pPerimeter);
	Perimeter* getPerimterById(uint32_t id);
	const TPerimeterMap& perimeters(void) { return m_perimeters; }
	uint32_t getNextPerimeterId() const;
	std::string getNextPerimeterStr() const;
	uint32_t numPerimeters() { return m_perimeters.size(); }

	bool addSpot(Spot* pSpot);
	void removeSpot(Spot* pSpot);
	Spot* getSpotById(uint32_t id);
	const TSpotMap& spots(void) { return m_spots; }
	uint32_t getNextSpotId() const;
	std::string getNextSpotStr() const;
	uint32_t numSpots() { return m_spots.size(); }

	void setSpeedLimit(SpeedLimit * limit) { m_speedLimit = limit; }
	SpeedLimit const * speedLimit() const { return m_speedLimit; }

	bool centerLatLon(double& clat, double& clon);

	void dump();


	void setOffroad() { m_offroad = true; }
	bool offroad() { return m_offroad; }

protected:
	std::string m_description;
	SpeedLimit * m_speedLimit;

	TPerimeterMap m_perimeters;
	TSpotMap m_spots;
	bool m_offroad;
	friend std::ostream& operator<<(std::ostream& os, const Zone& z);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Zone& z);
}

} // namespace vlr

#endif

