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


#ifndef AW_MISSION_H
#define AW_MISSION_H

#include <aw_roadNetwork.h>
#include <aw_CheckpointList.h>
#include <aw_SpeedLimitList.h>

namespace vlr {

namespace rndf {

  class Mission : public NetElement {
  public:
    Mission(RoadNetwork* rndf);
    virtual ~Mission();
    void clear();

    bool loadMDF(const std::string& strFileName);

    CheckpointList * addCheckpointList(const std::string & strData);
    SpeedLimitList * addSpeedLimitList(const std::string & strData);
    SpeedLimit * addSpeedLimit(const std::string & strName, const std::string & strMin, const std::string & strMax);

    inline void setName(std::string strName) {name_ = strName;}
    inline const std::string& name() {return name_;}
    void setStatus(std::string strStatus) {status_ = strStatus;}
    void appendStatus(std::string strStatus) {status_ += strStatus + '\n';}
    const std::string& status() {return status_;}

    CheckpointList* checkpointList() {return check_point_list_;}

    void dump();

  private:
    CheckpointList* check_point_list_;
    SpeedLimitList* speed_limit_list_;

    std::string status_;
    std::string name_;
    float format_version_;
    std::string creation_date_;
    RoadNetwork* rn_;
    std::string m_rndfName;

    void changeSpeedLimit(SpeedLimit* limit, const std::string & strMin, const std::string & strMax);
  };

}

} // namespace vlr

#endif
