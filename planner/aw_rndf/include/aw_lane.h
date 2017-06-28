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


#ifndef AW_LANE_H
#define AW_LANE_H

#include <aw_RndfId.h>
#include <aw_wayPoint.h>
#include <aw_checkPoint.h>
#include <aw_stop.h>
#include <aw_exit.h>
#include <aw_laneSegment.h>
#include <aw_SpeedLimit.h>

#define FT2M_FACTOR 0.3048

namespace vlr {

namespace rndf {

class Segment;
class Intersection;

typedef std::map<std::string,Lane*>	  TLaneMap;
typedef std::set<Lane*>				  TLaneSet;


class Lane : public NetElement
{
public:
	friend class RoadNetwork;

	static const double default_width = 4.5;

	enum eBoundaryTypes
	{
		UnknownBoundary = 0,
		NoBoundary = 1,
		SolidWhite = 2,
		BrokenWhite = 3,
		SolidYellow = 4,
		DoubleYellow = 5
	};

  enum eType{car_lane, bike_lane};

	Lane(uint32_t id, const std::string& strName, const bool isVirtual = false);
	virtual ~Lane();
	Lane(const Lane&);
	Lane& operator=(const Lane& other);
	Lane& copy(const Lane& other);

	void setSegment(Segment* segm) { segment_=segm; }
	Segment* segment() const { return segment_; }

  void setLaneWidth(double laneWidth) { lane_width_ = laneWidth; }
  double laneWidth() const { return lane_width_; }

  void setLaneType(eType t) { type_ = t; }
  eType laneType() const { return type_; }

	void setLeftBoundaryType(Lane::eBoundaryTypes leftBoundaryType) { left_boundary_type_ = leftBoundaryType; }
	void setRightBoundaryType(Lane::eBoundaryTypes rightBoundaryType) { right_boundary_type_ = rightBoundaryType; }

	Lane::eBoundaryTypes leftBoundaryType() {return left_boundary_type_; }
	Lane::eBoundaryTypes rightBoundaryType() { return right_boundary_type_; }

	void setSpeedLimit(SpeedLimit* limit) {speed_limit_ = limit;}
	SpeedLimit* speedLimit() {return speed_limit_;}
	const SpeedLimit* speedLimit() const {return speed_limit_;}


	//! adds a waypoint to the Lane
	bool addWayPoint(WayPoint* pWayPoint);
	bool addWayPoint(WayPoint* pWayPoint, uint32_t insert_before);
	void removeWayPoint(WayPoint* pWayPoint);
	void removeWayPoint(uint32_t index);
	WayPoint* wayPoint(uint32_t index) { return (index<waypoints_.size() ? waypoints_[index] : NULL); }
	WayPoint* wayPointFromId(uint32_t id);
	uint32_t wayPointIndex(const WayPoint* wp) const;
	size_t numWayPoints() { return waypoints_.size(); }


	void addExit(Exit* e);
	void removeExit(Exit* e);
	void addEntry(Exit* e);
	void removeEntry(Exit* e);

	inline void addLaneSegment(LaneSegment* ls) {
	  lane_segments_.push_back(ls);
	  bbox_valid_ = false;
	}

	LaneSegment* laneSegment(uint32_t index) { return (index < lane_segments_.size() ? lane_segments_[index] : NULL); }
	size_t numLaneSegments() { return lane_segments_.size(); }
	LaneSegment* laneSegmentWithFromPoint(WayPoint* fromPoint);
	LaneSegment* laneSegmentWithToPoint(WayPoint* toPoint);

	bool isVirtual() { return is_virtual_; }

	bool centerLatLon(double& clat, double& clon) const;

	static std::string boundaryToRndfString(eBoundaryTypes boundary);

	inline const TWayPointVec& wayPoints() const { return waypoints_; }
    inline const TExitMap& exits() const { return exits_; }
    inline const TExitMap& entries() const { return entries_; }
	inline const TCheckPointMap& checkpoints() const { return checkpoints_; }
	inline const TStopMap& stops() const { return stops_; }
	inline const TLaneSegmentVec& laneSegments() const { return lane_segments_; }

	void dump() const;

  void boundingBox(double& xmin, double& ymin, double& xmax, double& ymax);

  std::string nextWayPointStr() const;
//  inline std::string nextWayPointStr() const {
//    std::cout << __FUNCTION__ <<": " << segment_->name() << ", " << name() << ", "<< nextIdStr(waypoints_) << std::endl;
//    return segment_->name() + name() + nextIdStr(waypoints_);
//  }

private:
	TWayPointVec	 waypoints_;
	TExitMap			 exits_;
	TExitMap			 entries_;
	TCheckPointMap checkpoints_;
	TStopMap			 stops_;

	TLaneSegmentVec		lane_segments_;

	Segment* segment_; // parent Segment of the Lane
  SpeedLimit* speed_limit_;

	double lane_width_;
	eBoundaryTypes left_boundary_type_;
	eBoundaryTypes right_boundary_type_;
	double lat_sum_, lon_sum_, length_;

	bool is_virtual_;
  bool bbox_valid_;
  double xmin_, ymin_, xmax_, ymax_;
  eType type_;
	friend std::ostream& operator<<(std::ostream& os, const Lane& l);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Lane& l);

};

} // namespace vlr

#endif


