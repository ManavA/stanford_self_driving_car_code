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
#include <fstream>
#include <sstream>
#include <memory>

#include <boost/format.hpp>

#include <global.h>

#include "aw_Mission.h"
#include "aw_MDFTokens.h"

#include "aw_roadNetwork.h"
#include "aw_StringTools.h"

using namespace std;

namespace vlr {

namespace rndf {

Mission::Mission(RoadNetwork* rndf) :
	NetElement(0, "Mission"), check_point_list_(NULL), speed_limit_list_(NULL), format_version_(-1), rn_(rndf) {
}

Mission::~Mission() {
	clear();
}

bool Mission::loadMDF(const string & strFileName) {
	clear();

	// load file
	ifstream file(strFileName.c_str());
	string line;
	vector<string> tokens;

	if (!file.is_open() || !file.good()) {
		appendStatus((boost::format("Could not load file '%1%'") % strFileName).str());
	}
	// insert checkpoints and speed limits
	while (true) {
		tokens.clear();
		if (!getline(file, line))
			break;
		if (!line.length())
			continue;
		CStringTools::splitString(line, tokens, RNDF_DELIMITER);
		if (tokens.size()==0)
			continue;

		if (tokens[0]==MDF_MISSION_NAME)
			setName(tokens[1]);
		else if (tokens[0]==MDF_MISSION_FORMAT_VERSION)
			format_version_ = CStringTools::gfCFloat(tokens[1]);
		else if (tokens[0]==MDF_MISSION_CREATION_DATE)
			creation_date_ = tokens[1];
		else if (tokens[0]==MDF_CHECKPOINTLIST_BEGIN) {
			string strData = RoadNetwork::section(file, MDF_CHECKPOINTLIST_BEGIN, MDF_CHECKPOINTLIST_END);
			if (!strData.length())
				continue;
			addCheckpointList(strData);
		} else if (tokens[0]==MDF_MISSION_RNDF_REF) {
			m_rndfName = tokens[1];
			// TODO: (dj) shoreline_rndf_lot.txt
			//if (m_rndfName != rn_->name())
			//	setStatus("RNDF Name does not match: " + tokens[1]+ ", "+ rn_->name());
		} else if (tokens[0]==MDF_SPEEDLIMITLIST_BEGIN) {
			string strData = RoadNetwork::section(file, MDF_SPEEDLIMITLIST_BEGIN, MDF_SPEEDLIMITLIST_END);
			if (!strData.length())
				continue;
			addSpeedLimitList(strData);
		} else if (tokens[0]==MDF_MISSION_END) {
		} else
			appendStatus("Unknown token \'" + tokens[0]+ "\' in MDF data.");
	}

	file.close();

//	cout << "------------------------------------" << endl;
//	dump();
//	cout << "------------------------------------" << endl;

	return (status().length() == 0);
}

CheckpointList * Mission::addCheckpointList(const string & strData) {
	if (check_point_list_) {
		appendStatus("multiple checkpoint lists");
		return NULL;
	}

	std::string cpname="class rndf::CheckpointList"+name()+"CheckpointList";
	CheckpointList * pList = new CheckpointList(0, cpname);
	assert(pList);
	check_point_list_ = pList;
	istringstream iStream(strData);
	string line;
	vector<string> tokens;
	int n = -1;
	// loop lines
	while (true) {
		tokens.clear();
		if (!getline(iStream, line))
			break;
		if (!line.length())
			continue;
		CStringTools::splitString(line, tokens, RNDF_DELIMITER);
		if (tokens[0]==MDF_CHECKPOINTLIST_NUM_CHECKPOINTS)
			n = CStringTools::gnCInt(tokens[1]);
		else {
			cout << "getCheckpoint "<< tokens[0]<< " of "<< n << endl;
			CheckPoint * cp = rn_->checkPoint(tokens[0]);
			if (cp) {
				pList->addCheckpoint(cp);
			} else {
				cerr << "checkpoint " << tokens[0] << " could not be found -> skip it" << std::endl;
				--n;
			}
		}
	}

	if ((int32_t)pList->size() != n)
		appendStatus((boost::format("wrong number of checkpoints RNDF states %1% but has %2% defined") % n % pList->size()).str());

	return pList;
}

SpeedLimitList * Mission::addSpeedLimitList(const string & strData) {
	//cout << strData << endl;
	if (speed_limit_list_) {
		appendStatus("multiple speed limit lists");
		return NULL;
	}

	std::string sllname=name()+"SpeedLimitList";
	SpeedLimitList * pList = new SpeedLimitList(0, sllname);

	speed_limit_list_ = pList;
	istringstream iStream(strData);
	string line;
	vector<string> tokens;
	int n = -1;
	// loop lines
	while (true) {
		tokens.clear();
		if (!getline(iStream, line)) {
			break;
		}
		if (!line.length())
			continue;
		CStringTools::splitString(line, tokens, RNDF_DELIMITER);
		if (tokens[0]==MDF_SPEEDLIMITLIST_NUM_SPEEDLIMITS) {
			n = CStringTools::gnCInt(tokens[1]);
		} else if (tokens.size()> 2) {
			SpeedLimit * limit = addSpeedLimit(tokens[0], tokens[1], tokens[2]);
			pList->addSpeedLimit(limit);
		} else {
			cout << line << " could not be parsed" << endl;
		}
	}

	if (pList->size() != n)
		appendStatus("wrong number of speed limits");

	return pList;
}

SpeedLimit * Mission::addSpeedLimit(const string & name, const string & str_min, const string & str_max) {
	std::string sllname=name;
	SpeedLimit* speed_limit = new SpeedLimit(0, sllname);
	assert(speed_limit);

	cout << "speed limit for " << name << endl;
	Segment* seg = rn_->segment(name);
	if (speed_limit == NULL) {
		appendStatus("addSpeedLimit: speed limit was not added to the network.");
		return NULL;
	}
	if (seg == NULL) {
		Zone* zone = rn_->zone(name);
		if (zone == NULL) {
		  std::cout << "WARNING: speed limit list: segment/zone not found: " << name << "\n";
		  //			appendStatus("speed limit list: segment/zone not found "+name);
			return speed_limit;
		}
		speed_limit->setZone(zone);
	} else {
		speed_limit->setSegment(seg);
		//cout << " seg found and set" << endl;
	}
	speed_limit->minSpeed(dgc::dgc_mph2ms(CStringTools::gdCDouble(str_min)));
	speed_limit->maxSpeed(dgc::dgc_mph2ms(CStringTools::gdCDouble(str_max)));
	speed_limit->setRefName(name);
	return speed_limit;
}

void Mission::dump() {
	cout << "Dumping mission "<< name() << "..."<< endl;
	cout << "Creation Date: "<< creation_date_ << endl;
	cout << "Format Version: "<< format_version_ << endl;
	cout << "RNDF: "<< m_rndfName << endl;
	if (check_point_list_)
		check_point_list_->dump();
	if (speed_limit_list_)
		speed_limit_list_->dump();
}

void Mission::clear() {
	cout << "clear mission"<< endl;
	if (check_point_list_) {
		delete check_point_list_;
		check_point_list_ = NULL;
	}
	if (speed_limit_list_) {
		for (TSpeedLimitIterator it = speed_limit_list_->begin(); it != speed_limit_list_->end(); ++it) {
			SpeedLimit * limit = *it;
			delete limit;
		}
		delete speed_limit_list_;
	}
	speed_limit_list_ = NULL;
	setStatus("");
}

void Mission::changeSpeedLimit(SpeedLimit * limit, const string & str_min, const string & str_max) {
	double min = CStringTools::gdCDouble(str_min);
	double max = CStringTools::gdCDouble(str_max);
	if (min > limit->minSpeed()) {
		limit->minSpeed(min);
	}
	if (max < limit->maxSpeed()) {
		limit->maxSpeed(max);
	}
}


}

} // namespace vlr
