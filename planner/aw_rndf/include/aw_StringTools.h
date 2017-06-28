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


#ifndef AW_STRINGTOOLS_H
#define AW_STRINGTOOLS_H

#include <string>
#include <cstdio>
#include <vector>

namespace vlr {

namespace CStringTools {

// Konvertiert einen String zu einem Bool Wert
bool gbCBool(const std::string& str, const bool bDefault = false);
// Konvertiert einen String zu einem Integer Wert
int gnCInt(const std::string& str, const int nDefault = 0);
// Konvertiert einen String zu einem Float Wert
float gfCFloat(const std::string& str, const float fDefault = 0.0f);
// Konvertiert einen String zu einem Double Wert
double gdCDouble(const std::string& str, const double dDefault = 0.0);
// Konvertiert einen String zu einem Double Wert
long glCLong(const std::string& str, const long lDefault = 0);

// Konvertiert einen String Wert zu einem String
std::string gsCString(const std::string& var);
// Konvertiert einen Double Wert zu einem String
std::string gsCString(const double& var);
// Konvertiert einen Float Wert zu einem String
std::string gsCString(const float& var);
// Konvertiert einen Bool Wert zu einem String
std::string gsCString(const bool& var);
// Konvertiert einen Integer Wert zu einem String
std::string gsCString(const int& var);

void splitString(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ");

std::string clearCComments(std::string);

}; // CStringTools

} // namespace vlr

#endif
