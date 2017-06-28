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


#ifndef AW_BLOCKADE_MANAGER_HPP
#define AW_BLOCKADE_MANAGER_HPP

#include <aw_RndfGraph.h>

namespace vlr {

using namespace RoutePlanner;

class Topology;

/*this struct servers to return info about road blockades.*/
struct BlockadeInfo{
      float thres_critical_zdif; //specify that value, you will get number of cells that violate (are above) this threshold
      double borderDist;      //specify that value, the width to each side of the track spanning the area to verify
      int thres_cirticalN;    //specify that value, if nAboveThreshold is above this threshold blocked will be set to true
      int nAboveThreshold;    //you receive that value, the number of cells that violate @threshold_to_use
      float maxValue;         //you receive that value, the maximum value in the area
      float averageValue;     //the average value in the area


   /*says whether the path blocked*/
   bool bBlocked;
   /*says whether verification was successfull, only in this case @blocked can be trusted*/
   bool bVerificationWasSuccessfull;
};

//--------------------------------------------------------
//             BlockadeManager
//--------------------------------------------------------

class BlockadeManager {
public:
  class Blockade {
  public:
    Blockade();
    Blockade(RndfEdge* edge);
    ~Blockade();
    void update();
    void publish();
    void forcePublish();
    double getLastUpdateTime();

    RndfEdge* edge() const { return edge_; }
    bool isPublished() const { return published; }
    double getLastUpdateTime() const { return last_update_time; }

  private:
    RndfEdge* edge_;
    int update_counter;
    bool published;
    double last_update_time;
  };
  typedef std::map<RndfEdge*, Blockade> TBlockageMap;

  BlockadeManager(Topology* topology);

  bool update();
  TBlockageMap& getBlockades(void) { return blockade_map; }

  Blockade* getBlockadeByEdge(RndfEdge* edge);
  void addBlockade(RndfEdge* edge);
  void forceBlockade(RndfEdge* edge);
  void deleteBlockade(RndfEdge* edge);
  void clearBlockades() {blockade_map.clear(); current_blockade_id = -1;}

public:
  TBlockageMap blockade_map; // this map holds all blockages
  Topology* topology;
  int current_blockade_id;
};


inline std::ostream& operator << (std::ostream& os, const BlockadeManager::Blockade& v)
{
	return os << "Blockade: "<< v.edge() <<"  last update: "<< v.getLastUpdateTime() <<"  "<< (v.isPublished() ? "PUBLISHED" : "NOT PUBLISHED");
}

} // namespace vlr

#endif
