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


/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <sstream>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <boost/lexical_cast.hpp>

#include "aw_StringTools.h"

using std::string;
using std::vector;

namespace vlr {

// Konvertiert einen String zu einem Bool Wert
bool CStringTools::gbCBool(const std::string& str, const bool bDefault)
{
  const char* c_str = str.c_str();
  if (strcmp(c_str,"true")==0 ||
      strcmp(c_str,"True")==0 ||
      strcmp(c_str,"TRUE")==0 ||
      strcmp(c_str,"1")==0 ||
      strcmp(c_str,"-1")==0)
    return true;
  else if (
      strcmp(c_str,"false")==0 ||
      strcmp(c_str,"False")==0 ||
      strcmp(c_str,"FALSE")==0 ||
      strcmp(c_str,"0")==0)
    return false;
  else
    return bDefault;
}

// Konvertiert einen String zu einem Integer Wert
int CStringTools::gnCInt(const std::string& str, const int nDefault)
{
  int value;
  int res;
  if(str.length() == 0)
    return nDefault;
  res=sscanf(str.c_str(),"%i",&value);
  if (res==1)
    return value;
  return 0;
}

// Konvertiert einen String zu einem Float Wert
float CStringTools::gfCFloat(const std::string& str, const float fDefault)
{
	double value = boost::lexical_cast<float>(str);
	try { value = boost::lexical_cast<float>(str); }
	catch(boost::bad_lexical_cast &) { value = fDefault; }
	return value;
}

// Konvertiert einen String zu einem Double Wert
double CStringTools::gdCDouble(const std::string& str, const double dDefault )
{
	double value = boost::lexical_cast<double>(str);
	try { value = boost::lexical_cast<double>(str); }
	catch(boost::bad_lexical_cast &) { value = dDefault; }
	return value;
}

// Konvertiert einen String zu einem Double Wert
long CStringTools::glCLong(const std::string& str, const long lDefault )
{
  int value;
  int res;
  if(str.length() == 0)
    return lDefault;
  res=sscanf(str.c_str(),"%i",&value);
  if (res!=1)
    return lDefault;
  return (long)value;
}

// Konvertiert einen string Wert zu einem String
std::string CStringTools::gsCString(const std::string& var)
{
  return var;
}

// Konvertiert einen double Wert zu einem String
std::string CStringTools::gsCString(const double& var)
{
  std::stringstream ss;
  ss << var;
  return ss.str();
}

// Konvertiert einen float Wert zu einem String
std::string CStringTools::gsCString(const float& var)
{
  std::stringstream ss;
  ss << var;
  return ss.str();
}

// Konvertiert einen bool Wert zu einem String
std::string CStringTools::gsCString(const bool& var)
{
  if(var)
    return std::string("true");
  else
    return std::string("false");
}

// Konvertiert einen int Wert zu einem String
std::string CStringTools::gsCString(const int& var){
  std::stringstream ss;
  ss << var;
  return ss.str();
}



void CStringTools::splitString(const string& str, vector<string>& tokens, const string& delimiters)
{
	// Skip delimiters at beginning.
	string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	string::size_type pos     = str.find_first_of(delimiters, lastPos);

	while (string::npos != pos || string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

string CStringTools::clearCComments(string s)
{
	while(1) {
		string::size_type start_pos = s.find("/*", 0);
		if (start_pos == string::npos) break;
		string::size_type end_pos   = s.find("*/", start_pos+2);
		if (end_pos == string::npos) break;
		s.erase(start_pos, end_pos+2);
	}
	return s;
}

} // namespace vlr
