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


#include <iostream>
#include <global.h>
#include <aw_BlockadeManager.hpp>

namespace vlr {

using namespace RoutePlanner;

#undef TRACE
#define TRACE(str)
//#define TRACE(str) std::cout << "[BlockadeManager] " << str << std::endl;

#define BLOCKADE_PUBLISH_MIN          30
#define BLOCKADE_MAX_TIME_UNOBSERVED  60.0
//-----------------------------------------------------------------------------
//             Blockade
//-----------------------------------------------------------------------------
BlockadeManager::Blockade::Blockade() : edge_(NULL), update_counter(0), published(false)
{
}

BlockadeManager::Blockade::Blockade(RndfEdge* edge) : edge_(edge), update_counter(0), published(false)
{
}

BlockadeManager::Blockade::~Blockade()
{
}

void BlockadeManager::Blockade::update()
{
  last_update_time = Time::current();
  update_counter++;
  if(update_counter > BLOCKADE_PUBLISH_MIN) {
    publish();
  }
}

void BlockadeManager::Blockade::publish()
{
  edge_->setBlocked();
  published = true;
}

double BlockadeManager::Blockade::getLastUpdateTime()
{
  return last_update_time;
}

void BlockadeManager::Blockade::forcePublish()
{
  last_update_time = Time::current();
  update_counter = BLOCKADE_PUBLISH_MIN+1;
  publish();
}
//-----------------------------------------------------------------------------
//             BlockadeManager
//-----------------------------------------------------------------------------

BlockadeManager::BlockadeManager(Topology* topology) :
	topology(topology), current_blockade_id(0)
{
}

BlockadeManager::Blockade* BlockadeManager::getBlockadeByEdge(RndfEdge* edge) {
  TBlockageMap::iterator it;
  if(edge==NULL) return NULL;

  it = blockade_map.find(edge);
  if(it == blockade_map.end()) {
    return NULL;
  }
  else {
    Blockade& blockade = it->second;
    return &blockade;
  }
}

void BlockadeManager::addBlockade(RndfEdge* edge)
{
  TBlockageMap::iterator it;
  assert(edge);

  // Zone Edges auslassen
  if (edge->isBlockedEdge()) return;
  if (edge->isZoneEdge()) return;
  if (edge->intersection()) return;

  it = blockade_map.find(edge);
  if(it == blockade_map.end()) {
    blockade_map[ edge ] = Blockade(edge);
    blockade_map[ edge ].update();
  }
  else {
    Blockade& blockade = it->second;
    blockade.update();
  }
}

void BlockadeManager::forceBlockade(RndfEdge* edge)
{
  TBlockageMap::iterator it;
  assert(edge);

  // Zone Edges auslassen
  if (edge->isBlockedEdge()) return;
  if (edge->isZoneEdge()) return;
  //if (edge->intersection()) return; // forceBlockade gets called to block intersections

  it = blockade_map.find(edge);
  if(it == blockade_map.end()) {
    blockade_map[ edge ] = Blockade(edge);
    blockade_map[ edge ].forcePublish();
  }
  else {
    Blockade& blockade = it->second;
    blockade.forcePublish();
  }
}

void BlockadeManager::deleteBlockade(RndfEdge* edge)
{
  blockade_map.erase(edge);
}

bool BlockadeManager::update( )
{
  TBlockageMap::iterator it, it_end;
  double current_timestamp = Time::current();
  double diff_time;
  std::vector< RndfEdge* > deleted_objects;

//  TRACE("Aktuelle Blockaden:");
  for(it = blockade_map.begin(), it_end = blockade_map.end(); it!=it_end; ++it) {
	  Blockade& blockade = it->second;
//	  TRACE("  "<< blockade);
    diff_time = current_timestamp-blockade.getLastUpdateTime();
    // if blockade was not observed for some time -> erase from blockade manager
    if(diff_time>BLOCKADE_MAX_TIME_UNOBSERVED) {
      deleted_objects.push_back(it->first);
    } else if (blockade.isPublished() && blockade.edge()) {
    	blockade.edge()->setBlocked();
    }
  }

  std::vector< RndfEdge* >::iterator itd, itd_end;
  for(itd = deleted_objects.begin(), itd_end = deleted_objects.end(); itd!=itd_end; ++itd) {
    blockade_map.erase(*itd);
  }

  return true;
}

} // namespace vlr
