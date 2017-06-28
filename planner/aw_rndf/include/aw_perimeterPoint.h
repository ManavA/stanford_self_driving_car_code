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


#ifndef _PERIMETERPOINT_H_
#define _PERIMETERPOINT_H_

#include <aw_netElement.h>
#include <aw_exit.h>

namespace vlr {

namespace rndf {

class Perimeter;
class Exit;

typedef std::map<RndfId, PerimeterPoint*, RndfIdLess> TPerimeterPointMap;
typedef std::vector<PerimeterPoint*> TPerimeterPointVec;

class PerimeterPoint: public NetElement {
public:
  typedef std::map<std::string, rndf::Exit*> TExitMap;

  PerimeterPoint(uint32_t id, const std::string& strName);
  PerimeterPoint(const PerimeterPoint&);
  virtual ~PerimeterPoint();
  PerimeterPoint& operator=(const PerimeterPoint& other);
  PerimeterPoint& copy(const PerimeterPoint& other);

  void setPerimeter(Perimeter* p) {
    parent_perimeter_ = p;
  }
  const Perimeter* perimeter() const {
    return parent_perimeter_;
  }

  void addExit(Exit* e);
  void removeExit(Exit* e);
  void addEntry(Exit* e);
  void removeEntry(Exit* e);

  const TExitMap& exits() const {return exits_;}
  const TExitMap& entries() const {return entries_;}

  void setLatLon(double lat, double lon);
  void setUtm(double utm_x, double utm_y, const std::string& utm_zone);

  uint32_t index() const;

  double lat() const {return lat_;};
  double lon() const {return lon_;};
  double utmX() const {return utm_x_;};
  double utmY() const {return utm_y_;};
  const std::string& utmZone() { return utm_zone_; };
  double x() const {return utm_x_;};
  double y() const {return utm_y_;};

  void dump();

protected:
  Perimeter* parent_perimeter_;

  TExitMap exits_;
  TExitMap entries_;

  double lat_, lon_;
  double utm_x_, utm_y_;
  std::string utm_zone_;

  friend std::ostream& operator<<(std::ostream& os, const PerimeterPoint& p);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const PerimeterPoint& p);

};

} // namespace vlr

#endif
