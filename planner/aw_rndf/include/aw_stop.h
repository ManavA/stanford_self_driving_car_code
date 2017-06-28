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


#ifndef _STOP_H_
#define _STOP_H_

#include <aw_netElement.h>
#include <aw_wayPoint.h>

namespace vlr {

namespace rndf {
class Stop;
typedef std::map<std::string, Stop*> TStopMap;

class Stop: public NetElement {
public:
  friend class RoadNetwork;

  Stop(uint32_t id, const std::string& strName);
  Stop(const Stop&);
  virtual ~Stop(void);
  Stop& operator=(const Stop& other);
  Stop& copy(const Stop& other);

  WayPoint* wayPoint(void) {
    return way_point_;
  }
  void setWayPoint(WayPoint* way_point) {
    way_point_ = way_point;
  }
  void dump() const;

private:
  WayPoint* way_point_;
protected:
  friend std::ostream& operator<<(std::ostream& os, const Stop& s);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Stop& s);

}
;

} // namespace vlr

#endif

