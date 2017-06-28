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


#ifndef AW_PERIMETER_H
#define AW_PERIMETER_H

#include <aw_exit.h>
#include <aw_perimeterPoint.h>


namespace vlr {

namespace rndf
{
class Zone;

typedef std::map<std::string, Perimeter*>		TPerimeterMap;

class Perimeter : public NetElement
{
public:
	Perimeter(uint32_t id, const std::string& strName);
	Perimeter(const Perimeter&);
	virtual ~Perimeter(void);
	Perimeter& operator=(const Perimeter& other);
	Perimeter& copy(const Perimeter& other);

	void setZone(Zone* z) { parent_zone_ = z; }
	Zone* zone() const { return parent_zone_; }

	bool addPerimeterPoint(PerimeterPoint* pPerimeterPoint);
	bool addPerimeterPoint(PerimeterPoint* pPerimeterPoint, uint32_t insert_before);
	void removePerimeterPoint(PerimeterPoint* pPerimeterPoint);
	void removePerimeterPoint(uint32_t index);
	PerimeterPoint* perimeterPoint(uint32_t index) { return (index<perimeter_points_.size() ? perimeter_points_[index] : NULL); }
	PerimeterPoint* perimeterPointById(uint32_t id);
	uint32_t perimeterPointIndex(const PerimeterPoint* pPerimeterPoint) const;
	size_t numPerimeterPoints() { return perimeter_points_.size(); }

	bool addExit(Exit* pExit);
	void removeExit(Exit* pExit);

  inline size_t nextPerimeterPointId() const {
    return nextId(perimeter_points_);
  }

  inline std::string nextPerimeterPointStr() const {
    return name() + "." + nextIdStr(perimeter_points_);
  }

	bool centerLatLon(double& clat, double& clon) const;

	void dump();

	const TPerimeterPointVec& perimeterPoints() const { return perimeter_points_;}
	const TExitMap& exits() const { return exits_;}

protected:
 	TPerimeterPointVec  perimeter_points_;
	TExitMap            exits_;
	Zone*				        parent_zone_;

	friend std::ostream& operator<<(std::ostream& os, const Perimeter& p);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Perimeter& p);

};

} // namespace vlr

#endif


