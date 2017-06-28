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


#ifndef NAME_MAPPING_H
#define NAME_MAPPING_H

#include <assert.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <sstream>

class NameMapping {
public:
  NameMapping(std::vector<std::string> idToName);
  NameMapping(std::istream& is);
  NameMapping(const NameMapping& nm);
  void addName(std::string name);
  void augment(const NameMapping & m1);
  std::string toName(size_t id) const;
  size_t toId(std::string name) const;
  std::vector<std::string> getIdToNameMapping() const;
  std::map<std::string, size_t> getNameToIdMapping() const;
  size_t getSerializationVersion() const;
  std::string serialize() const;
  bool compare(const NameMapping & m2) const;
  //! Returns a string that reports how this NameMapping is different than m2.
  void diff(const NameMapping& m2, std::vector<std::string>* not_there, std::vector<std::string>* not_here) const;

  size_t size() const;
  //! Checks if m2 has the same names, regardless of order.
  bool isPermutation(const NameMapping& m2) const;
  
private:
  size_t serialization_version_;
  std::vector<std::string> idToName_;
  std::map<std::string, size_t> nameToId_;
};

class NameTranslator {
public:
  //! @param m1 The original mapping.
  //! @param m2 The destination mapping.
  NameTranslator(NameMapping m1, NameMapping m2);
  size_t toMap1(size_t id) const;
  size_t toMap2(size_t id) const;
  size_t size() const;
  
private:
  std::vector<size_t> id1ToId2_;
  std::vector<size_t> id2ToId1_;
};

#endif
