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


#ifndef AW_SPEEDLIMITLIST_H
#define AW_SPEEDLIMITLIST_H

#include <iostream>
#include <vector>

#include <aw_netElement.h>
#include <aw_SpeedLimit.h>

namespace vlr {

namespace rndf {

  typedef std::vector<SpeedLimit*> TSpeedLimitVector;
  typedef TSpeedLimitVector::iterator TSpeedLimitIterator;

  class SpeedLimitList : public NetElement {

  public:
    SpeedLimitList(unsigned int id, std::string & strName);
    virtual ~SpeedLimitList();

    int size();

    virtual bool addSpeedLimit(SpeedLimit * limit);

    TSpeedLimitIterator begin() { return m_vector.begin(); }
    TSpeedLimitIterator end() { return m_vector.end(); }

    SpeedLimit * speedLimitForSegment(std::string segmentName);
    SpeedLimit * speedLimitForZone(std::string zoneName);

    void dump();

  private:
    TSpeedLimitVector m_vector;

  };
}

} // namespace vlr

#endif
