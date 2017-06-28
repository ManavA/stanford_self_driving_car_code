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

#include <aw_SpeedLimitList.h>
#include <aw_segment.h>
#include <aw_zone.h>

using namespace std;

namespace vlr {

namespace rndf {

  SpeedLimitList::SpeedLimitList(unsigned int id, std::string& strName) : NetElement(id, strName) {
  }

  SpeedLimitList::~SpeedLimitList() {
  }

  int SpeedLimitList::size() {
    return m_vector.size();
  }

  bool SpeedLimitList::addSpeedLimit(SpeedLimit * limit) {
    m_vector.push_back(limit);
  return true;
  }

  void SpeedLimitList::dump() {
    std::cout << "speed limit list " << name() << std::endl;
    for (TSpeedLimitIterator it = begin(); it != end(); ++it) {
      (*it)->dump();
    }
  }

  SpeedLimit * SpeedLimitList::speedLimitForSegment(std::string segmentName)
  {
  	//cout << "search for " << segmentName << endl;
    for (TSpeedLimitIterator it = m_vector.begin(); it != m_vector.end(); ++it) {
      SpeedLimit* sl = (*it);
      //sl->dump();
    	if (sl->segment() && sl->segment()->name() == segmentName) {
    		return sl;
    	}
    }
    return 0;
  }
  SpeedLimit * SpeedLimitList::speedLimitForZone(std::string zoneName)
  {
    for (TSpeedLimitIterator it = m_vector.begin(); it != m_vector.end(); ++it) {
      SpeedLimit* sl = (*it);
    	if (sl->zone() && sl->zone()->name() == zoneName) {
    		return sl;
    	}
    }
    return 0;
  }

}

} // namespace vlr
