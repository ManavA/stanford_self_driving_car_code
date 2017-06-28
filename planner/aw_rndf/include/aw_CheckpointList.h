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


#ifndef AW_CHECKPOINTLIST_H
#define AW_CHECKPOINTLIST_H

#include <iostream>
#include <deque>

#include <aw_checkPoint.h>

namespace vlr {

class Topology;

namespace rndf {

  typedef std::deque<CheckPoint*> TCheckpointVec;

  class CheckpointList : public NetElement {

  public:
    CheckpointList(uint32_t id, std::string & strName);
    virtual ~CheckpointList();

    inline size_t size() {return vector_.size();}
    inline bool empty() {return vector_.empty();}

    virtual bool addCheckpoint(CheckPoint* pCheckpoint, bool atBeginning=false);

    TCheckpointVec::const_iterator begin() const { return vector_.begin(); }
    TCheckpointVec::const_iterator end() const { return vector_.end(); }

    TCheckpointVec& checkpoints() { return vector_; }

    void dump();

  private:
    TCheckpointVec vector_;

    friend class Topology;
  };

}

} // namespace vlr

#endif
