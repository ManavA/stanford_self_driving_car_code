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
#include <algorithm>

#include "aw_zone.h"
#include "aw_roadNetwork.h"

using namespace std;

namespace vlr {

namespace rndf {
Zone::Zone(uint32_t id, const string& strName) :
  NetElement(id, strName), m_speedLimit(NULL), m_offroad(false) {
}

Zone::~Zone(void) {
}

bool Zone::addPerimeter(Perimeter* pPerimeter) {
  pair<TPerimeterMap::iterator, bool> result = m_perimeters.insert(make_pair(pPerimeter->name(), pPerimeter));
  if (!result.second) {
    // setStatus("addPerimeter: Perimeter with name " + strName + " was not added to the network.");
    return false;
  }
  return true;
}

Perimeter* Zone::getPerimterById(uint32_t id) {
  for (TPerimeterMap::iterator it = m_perimeters.begin(); it != m_perimeters.end(); ++it)
    if (it->second->id() == id) return it->second;
  return NULL;
}

bool Zone::addSpot(Spot* pSpot) {
  pair<TSpotMap::iterator, bool> result = m_spots.insert(make_pair(pSpot->name(), pSpot));
  if (!result.second) {
    // setStatus("addPerimeter: Perimeter with name " + strName + " was not added to the network.");
    return false;
  }
  return true;
}

Spot* Zone::getSpotById(uint32_t id) {
  for (TSpotMap::iterator it = m_spots.begin(); it != m_spots.end(); ++it)
    if (it->second->id() == id) return it->second;
  return NULL;
}

void Zone::removePerimeter(Perimeter* p) {
  if (m_perimeters.find(p->name()) == m_perimeters.end()) {
    return;
  }
  m_perimeters.erase(p->name());
}

void Zone::removeSpot(Spot* s) {
  if (m_spots.find(s->name()) == m_spots.end()) {
    return;
  }
  m_spots.erase(s->name());
}

void Zone::dump() {
  cout << "----------------------------------------" << endl;
  cout << "Zone: " << name() << endl;
  cout << "description: " << m_description << endl;
  cout << "# of perimeters: " << m_perimeters.size() << endl;
  TPerimeterMap::iterator it, it_end;
  for (it = m_perimeters.begin(), it_end = m_perimeters.end(); it != it_end; ++it)
    (*it).second->dump();
  TSpotMap::iterator sit, sit_end;
  for (sit = m_spots.begin(), sit_end = m_spots.end(); sit != sit_end; ++sit)
    (*sit).second->dump();
  cout << "----------------------------------------" << endl;
}

uint32_t Zone::getNextPerimeterId() const {
  vector<uint32_t> ids;
  for (TPerimeterMap::const_iterator it = m_perimeters.begin(); it != m_perimeters.end(); ++it)
    ids.push_back(it->second->id());
  for (TSpotMap::const_iterator it = m_spots.begin(); it != m_spots.end(); ++it)
    ids.push_back(it->second->id());
  sort(ids.begin(), ids.end());

  for (uint32_t i = 1; i <= ids.size(); ++i)
    if (i != ids[i - 1]) return i;

  return ids.size() + 1;
}

std::string Zone::getNextPerimeterStr() const {
  return name() + "." + boost::lexical_cast<std::string>(getNextPerimeterId());
}

uint32_t Zone::getNextSpotId() const {
  return getNextPerimeterId();
}

std::string Zone::getNextSpotStr() const {
  return getNextPerimeterStr();
}

bool Zone::centerLatLon(double& clat, double& clon) {
  double latSum = 0, lonSum = 0;
  uint32_t num = 0;

  if (m_perimeters.size() == 0) {
    return false;
  }

  // sum valid Perimeter positions
  TPerimeterMap::iterator pit, pit_end;
  for (pit = m_perimeters.begin(), pit_end = m_perimeters.end(); pit != pit_end; ++pit) {
    if ((*pit).second->centerLatLon(clat, clon)) {
      latSum += clat;
      lonSum += clon;
      num++;
    }
  }

  // add valid Spot positions
  TSpotMap::iterator sit, sit_end;
  for (sit = m_spots.begin(), sit_end = m_spots.end(); sit != sit_end; ++sit) {
    if ((*sit).second->centerLatLon(clat, clon)) {
      latSum += clat;
      lonSum += clon;
      num++;
    }
  }

  clat = latSum / num;
  clon = lonSum / num;
  return true;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Zone& z) {
  os << RNDF_ZONE_BEGIN << " " << z.name_ << endl;
  os << RNDF_ZONE_NAME << " " << z.m_description << endl;
  os << RNDF_ZONE_NUM_SPOTS << " " << z.m_spots.size() << endl;

  TPerimeterMap::const_iterator it, it_end;
  for (it = z.m_perimeters.begin(), it_end = z.m_perimeters.end(); it != it_end; ++it)
    os << *it->second;
  TSpotMap::const_iterator sit, sit_end;
  for (sit = z.m_spots.begin(), sit_end = z.m_spots.end(); sit != sit_end; ++sit)
    os << *sit->second;

  os << RNDF_ZONE_END << endl;

  return os;
}
}

} // namespace vlr

