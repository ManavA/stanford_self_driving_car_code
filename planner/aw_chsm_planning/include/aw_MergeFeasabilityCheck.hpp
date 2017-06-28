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


#ifndef AW_MERGEFEASABILITYCHECK_HPP
#define AW_MERGEFEASABILITYCHECK_HPP

#include <vector>
#include <cassert>
#include <aw_CGAL.h>
#include <aw_RndfEdge.h>

namespace vlr {

class c2_mergePar;

class MFCIntersection {};
extern MFCIntersection mfcIntersection;
class MFCPassObstacle {};
extern MFCPassObstacle mfcPassObstacle;

/*!
 * Handles feasibility checks for one single merging point. Because this class has
 * an internal state you should use one object for the same merging point over and over again.
 */
class MergeFeasabilityCheck
{
public:

	enum Result {
		Merge,
		Stop
	};

	struct Entity {
		Entity(double distance, double speed, Vehicle* veh = NULL) : distance(distance), speed(speed), veh(veh) {};
		double distance;
		double speed;
		Vehicle* veh;
	};

	static const double nominal_acceleration;

	typedef std::vector<Entity> Entities;

	//! constructs a MFC object for an intersection
	MergeFeasabilityCheck(MFCIntersection /*noname*/, Result startState, double radius);
	//! constructs a MFC object for pass obstacle
	MergeFeasabilityCheck(MFCPassObstacle /*noname*/, Result startState);
	virtual ~MergeFeasabilityCheck();

	void setState(Result s) {state = s;};
	Result getState() const {return state;};

	void setEgoGeoConstrain(double before_MP, double after_MP);
	void setOtherGeoConstrain(double before_MP, double after_MP);
	void setTimeConstrain(double ego_before_other, double other_before_ego);
	void setHysterese(double hyst);

	Result test(const Entity& ego_vehicle, const Entity& vehicle, const double velocity_desired);
	Result test(const Entity& ego_vehicle, const Entities& vehicles, const double velocity_desired);

	static Entity getEntity(const CGAL_Geometry::Point_2& merging_point, const CGAL_Geometry::Point_2& vehicle, const double yaw, const double speed);

	static Entities getEntities(RoutePlanner::RndfEdge* edge, double offset, double dist_in_from_direction, double dist_in_to_direction, bool same_lane = true);
protected:
	Result state; // need acutal state for hysterese
	c2_mergePar* par;
};

} // namespace vlr

#endif // AW_MERGEFEASABILITYCHECK_HPP
