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


#include <multibooster/name_mapping.h>

using namespace std;

NameMapping::NameMapping(vector<string> idToName) :
  serialization_version_(0)
{
  for(size_t i=0; i<idToName.size(); ++i) {
    addName(idToName[i]);
  }
}

NameMapping::NameMapping(istream& is) :
  serialization_version_(0)
{
  string str;
  is >> str;
  assert(str.compare("NameMapping.") == 0);
  size_t this_serialization_version;
  is >> this_serialization_version;
  if(this_serialization_version != serialization_version_) {
    cerr << "Wrong serialization version: " << this_serialization_version << " vs " << serialization_version_ << endl;
    throw;
  }
  size_t num_elements;
  is >> num_elements;
  idToName_.resize(num_elements);
  size_t id;
  for(size_t i=0; i<num_elements; ++i) { 
    is >> id;
    assert(id == i);
    is >> str;
    idToName_[i] = str;
    nameToId_[str] = id;
  }
  is >> str;
  assert(str == "End.");
}

NameMapping::NameMapping(const NameMapping& nm) :
  serialization_version_(nm.serialization_version_),
  idToName_(nm.idToName_),
  nameToId_(nm.nameToId_)
{
} 

string NameMapping::serialize() const {
  ostringstream oss(ostringstream::out);
  oss << "NameMapping." << endl;
  oss << serialization_version_ << endl;
  oss << idToName_.size() << endl;
  for(size_t i=0; i<idToName_.size(); ++i) { 
    oss << i << " " << idToName_[i] << endl;
  }
  oss << "End.";
  return oss.str();
}

size_t NameMapping::size() const {
  return idToName_.size();
}

bool NameMapping::compare(const NameMapping & m2) const {
  vector<string> idToName2 = m2.getIdToNameMapping();
  map<string, size_t> nameToId2 = m2.getNameToIdMapping();
  if(serialization_version_ != m2.serialization_version_ ||
     idToName_.size() != idToName2.size()) {
    return false;
  }

  for(size_t i=0; i<idToName_.size(); ++i) {
    if(idToName_[i] != idToName2[i])  {
      //cout << "found: " << idToName_[i] << " " << idToName2[i] << endl;
      return false;
    }
    string name = idToName2[i]; // == idToName_[i].
    map<string, size_t>::const_iterator it;
    it = nameToId_.find(name);
    size_t id1 = it->second;
    it = nameToId2.find(name);
    size_t id2 = it->second;
    if(id1 != id2) {
      //cout << "found: " << id1 << " " << id2 << endl;
      return false;
    }
  }
  return true;
}

void
NameMapping::diff(const NameMapping& m2,
		  vector<string>* not_there,
		  vector<string>* not_here) const
{
  assert(not_here->empty());
  assert(not_there->empty());
  
  for(size_t i = 0; i < idToName_.size(); ++i) {
    if(m2.nameToId_.find(idToName_[i]) == m2.nameToId_.end())
      not_there->push_back(idToName_[i]);
  }

  vector<string> there_not_here;
  for(size_t i = 0; i < m2.idToName_.size(); ++i) {
    if(nameToId_.find(m2.idToName_[i]) == nameToId_.end())
      not_here->push_back(m2.idToName_[i]);
  }
}

bool NameMapping::isPermutation(const NameMapping& m2) const {
  for(size_t i = 0; i < idToName_.size(); ++i) {
    if(m2.nameToId_.find(idToName_[i]) == m2.nameToId_.end())
      return false;
  }

  for(size_t i = 0; i < m2.idToName_.size(); ++i) {
    if(nameToId_.find(m2.idToName_[i]) == nameToId_.end())
      return false;
  }

  return true;
}

map<string, size_t> NameMapping::getNameToIdMapping() const {
  return nameToId_;
}

vector<string> NameMapping::getIdToNameMapping() const {
  return idToName_;
}

void NameMapping::addName(string name) {
  if(name.find(" ") != string::npos) {
    cerr << "You cannot use spaces in mapping names.  " << name << endl;
    throw 5;
  }
  assert(nameToId_.find(name) == nameToId_.end());
  nameToId_[name] = idToName_.size();
  idToName_.push_back(name);
}

void NameMapping::augment(const NameMapping & m1) {
  vector<string> names = m1.getIdToNameMapping();
  for(size_t i=0; i<names.size(); ++i) {
    if(nameToId_.find(names[i]) == nameToId_.end())
      addName(names[i]);
  }
}

string NameMapping::toName(size_t id) const {
  assert(idToName_.size() > id);
  if(id > idToName_.size()) {
    cerr << "No id " << id << " in this mapping." << endl;
    throw 3;
  }
  return idToName_[id];
}

size_t NameMapping::toId(string name) const {
  map<string, size_t>::const_iterator it;
  it = nameToId_.find(name);
  if(it == nameToId_.end()) {
    cerr << "No name " << name << " in this mapping." << endl;
    throw 4;
  }
  
  return it->second;
}



NameTranslator::NameTranslator(NameMapping m1, NameMapping m2) {
  m1.augment(m2); // Does not change the orig.
  m2.augment(m1);

  vector<string> idToName1 = m1.getIdToNameMapping();
  vector<string> idToName2 = m2.getIdToNameMapping();
  map<string, size_t> nameToId1 = m1.getNameToIdMapping();
  map<string, size_t> nameToId2 = m2.getNameToIdMapping();
  
  id2ToId1_.resize(idToName1.size());
  id1ToId2_.resize(idToName1.size());

  for(size_t i=0; i<idToName1.size(); ++i) {
    size_t id2 = nameToId2[idToName1[i]];
    id1ToId2_[i] = id2;
    id2ToId1_[id2] = i;
  }
}

size_t NameTranslator::toMap1(size_t id) const {
  return id2ToId1_[id];
}

size_t NameTranslator::toMap2(size_t id) const {
  return id1ToId2_[id];
}

size_t NameTranslator::size() const {
  return id1ToId2_.size();
}

