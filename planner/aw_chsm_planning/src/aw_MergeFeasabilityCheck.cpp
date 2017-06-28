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


#include <cmath>
#include <deque>
#include <aw_MergeFeasabilityCheck.hpp>
#include <aw_ChsmPlanner.hpp>
#include <aw_Vehicle.h>

namespace vlr {

using namespace RoutePlanner;

#define TRACE(str) std::cout << "[MergeFeasabilityCheck] " << str << std::endl;

struct c2_mergePar
{
    double D_A1;
    double D_A2;
    double D_B1;
    double D_B2;
    double dT_AB;
    double dT_BA;
    double hyst;
};

#define C2_STATE_MERGING   1
#define C2_STATE_STOPPING  2

#define C2_EPS 1e-10

const double MergeFeasabilityCheck::nominal_acceleration = 1.0;



/*    B@d_B---[P_B1]----->--------[MP]---------[P_B2]----[P_A2]---->------
                                  /
                                 /
                                /
                             [P_A1]
                              /
                             /
                            /
                         A@d_A
 */

int c2_merge_checkSingleCar(c2_mergePar *par, int state, double a, double v_d, double v_A, double v_B, double d_A, double d_B);

/* Function tests whether the crossing / intersection / passing maneuver is save within the specified parameters and measurement values
 * par      ..  Pointer to geometric and dynamic parameters
 * state    ..  {C2_STATE_MERGING, C2_STATE_STOPPING}
 * a        ..  nominal acceleration, e.g. 1.5 m/s^2
 * v_d      ..  desired end velocity during merging
 * v_A      ..  current velocity of autonomous vehicle (A)
 * v_B      ..  current velocity of other vehicle (B)
 * d_A      ..  current distance of autonomous vehicle to MP (A)
 * d_B      ..  current distance of other vehicle to MP (B)
 */

double c2_merge_deltaSA_t(double v_0, double a, double v_d, double t);
/* Function returns the distance traveled by a constantly accelerating vehicle to final speed v_d at time t*/

double c2_merge_t_deltaSA(double v_0, double a, double v_d, double deltaS);
/* Inverse function of c2_merge_deltaSA_t */


int c2_merge_checkSingleCar(c2_mergePar *par, int state, double a, double v_d, double v_A, double v_B, double d_A, double d_B)
{
    int free_geom_AB, free_geom_BA, free_dyn_AB, free_dyn_BA;
    double deltaSA_tB1, deltaSA_tB2, tMPA, tMPB;
    double tB1,tB2;
    double h = 1;  // C2_STATE_STOPPING

    /* Hysteresis against chattering */
    if ( state == C2_STATE_MERGING ) h = par->hyst;

    /* Avoid division by zero */
    v_B = ( v_B == 0.0 ? C2_EPS : v_B );
    a   = ( a   == 0.0 ? C2_EPS : a   );

    tB1 = ( d_B - h*par->D_B1 ) / v_B;    // point in time where B passes P_B1
    tB2 = ( d_B + h*par->D_B2 ) / v_B;    // point in time where B passes P_B2

//    printf("tB1 = %f\n", tB1);
//    printf("tB2 = %f\n", tB2);


    /* Check geometric conditions */
    deltaSA_tB2 = c2_merge_deltaSA_t( v_A, a, v_d, tB2 );       // position of A at time tB2
    deltaSA_tB1 = c2_merge_deltaSA_t( v_A, a, v_d, tB1 );       // position of A at time tB1

    if ( v_B > 0 )
    {
        free_geom_BA = ( deltaSA_tB2 < d_A - h*par->D_A1 );     // Will A be still before P_A1 at tB2?
        free_geom_AB = ( deltaSA_tB1 > d_A + h*par->D_A2 );     // Will A have passed P_A2 at tB1?
    }
    else /* B going backwards */
    {
        free_geom_BA = ( deltaSA_tB1 < d_A - h*par->D_A1 );     // Will A be still bevore P_A1 at tB1?
        free_geom_AB = ( deltaSA_tB2 > d_A + h*par->D_A2 );     // Will A have passed P_A2 at tB2?
    }

    /* Check dynamic conditions */
    tMPA = c2_merge_t_deltaSA ( v_A, a, v_d, d_A );         // time for A to reach MP
    tMPB = d_B / v_B;                                       // time for B to reach MP

    free_dyn_AB  = ( tMPB > ( tMPA + h*par->dT_AB ) );        // hast dT_AB passed since A was at MP?
    free_dyn_BA  = ( tMPA > ( tMPB + h*par->dT_BA ) );        // hast dT_BA passed since B was at MP?

    /*Special cases */
    /* A is already beyond MP */
    if ( d_A <= 0 )
    {
        printf("A is already beyond MP\n");
        if ( (v_B >= 0.1) || (d_B > d_A) || (d_B < -par->D_B2) ) {
          printf("(v_B > 0) || (d_B > d_A)\n");
          return C2_STATE_MERGING;   // If B is moving into the same direction of infront, there is no reason to stop
        }
        else {
          printf("else\n");
          return C2_STATE_STOPPING;
        }
    }


    /* A is already between P_A1 and MP */
    if ( ( d_A < h*par->D_A1) && ( d_A > 0 ) )
    {
//        printf("A is already between P_A1 and MP\n");
        if( v_B > 0 ) free_geom_BA = ( -d_B >= h*(par->D_B2) );    // is B already behind P_B2?
        else free_geom_BA = ( d_B > h*(par->D_B1) );              // has B already passed  P_B1?
    }

//    printf("tMPA = %f\n",tMPA);
//    printf("tMPB = %f\n",tMPB);
//
//    printf("free_geom_AB = %d\n",free_geom_AB);
//    printf("free_dyn_AB = %d\n",free_dyn_AB);
//    printf("free_geom_BA = %d\n",free_geom_BA);
//    printf("free_dyn_BA = %d\n",free_dyn_BA);

    return  ( ( free_geom_AB && free_dyn_AB ) || ( free_geom_BA && free_dyn_BA ) ) ? C2_STATE_MERGING : C2_STATE_STOPPING;
}

double c2_merge_deltaSA_t(double v_0, double a, double v_d, double t)
{
    double t_sw = ( v_d - v_0 ) / a;
    double res;

//    printf("t_sw = %f\n",t_sw);

    /* Just in case v_0 > v_d */
    if ( t_sw < 0 )
    {
        t_sw *= (-1);
        a    *= (-1);   // Deceleration assumed to have same magnitude
    }
//    printf("t_sw = %f\n",t_sw);
    if( t <= t_sw ) res = v_0*t    + a/2.0 * t*t;
    else            res = v_0*t_sw + a/2.0 * t_sw*t_sw + v_d*(t-t_sw);
    if (t < 0 )     res = 0;         // Where was A at t<0 ??? Assume at s=0
//    printf("res = %f\n",res);
    return res;
}

double c2_merge_t_deltaSA(double v_0, double a, double v_d, double deltaS)
{
    double res, t_s1;
    double t_sw = ( v_d - v_0 ) / a;

    /* Just in case v_0 > v_d */
    if ( t_sw < 0 )
    {
        t_sw *= (-1);
        a    *= (-1);   // Deceleration assumed to have same magnitude
    }

    t_s1 = ( deltaS - v_0 * t_sw - a/2.0 * t_sw*t_sw ) / ( v_0 + a* t_sw ) +t_sw;
    if ( t_s1 > t_sw ) res = t_s1;
    else if ( (v_0*v_0 + 2*a*deltaS) < 0) res = 0;     // solution in the past is not interesting
    else               res = 1.0 / a *( -v_0 + sqrt(v_0*v_0 + 2*a*deltaS) );
    //printf('%d',res);
    return res;
}

MFCIntersection mfcIntersection;
MFCPassObstacle mfcPassObstacle;

//! constructs a MFC object for an intersection
MergeFeasabilityCheck::MergeFeasabilityCheck(MFCIntersection /*noname*/, Result startState, double radius)
: state(startState)
{
	par = new c2_mergePar;
	assert(par);
	radius = radius*1.1;
	par->D_A1 = radius;
	par->D_A2 = radius;
	par->D_B1 = radius;
	par->D_B2 = radius;
	par->dT_AB = 6.0; // TODO: tune value
	par->dT_BA = 1.0; // TODO: tune value
	par->hyst = 0.8;
}
//! constructs a MFC object for pass obstacle
MergeFeasabilityCheck::MergeFeasabilityCheck(MFCPassObstacle /*noname*/, Result startState)
: state(startState)
{
	par = new c2_mergePar;
	assert(par);
	par->D_A1 = STD_VEHICLE_LENGTH;
	par->D_A2 = STD_VEHICLE_LENGTH;
	par->D_B1 = STD_VEHICLE_LENGTH;
	par->D_B2 = STD_VEHICLE_LENGTH;
	par->dT_AB = 4.0;// TODO: tune value
	par->dT_BA = 2.0;// TODO: tune value
	par->hyst = 0.8;
}
MergeFeasabilityCheck::~MergeFeasabilityCheck()
{
	delete par;
}

MergeFeasabilityCheck::Result MergeFeasabilityCheck::test(const Entity& ego_vehicle, const Entity& vehicle, const double velocity_desired) {
	int s = state==Stop?C2_STATE_STOPPING:C2_STATE_MERGING;
	int r = c2_merge_checkSingleCar(par, s, nominal_acceleration, velocity_desired, ego_vehicle.speed, vehicle.speed, ego_vehicle.distance, vehicle.distance);
//	std::cout << "c2_merge_checkSingleCar({"
//		<< " D_A1:" << par->D_A1 << ""
//		<< " D_A2:" << par->D_A2 << ""
//		<< " D_B1:" << par->D_B1 << ""
//		<< " D_B2:" << par->D_B2 << ""
//		<< " dT_AB:" << par->dT_AB << ""
//		<< " dT_BA:" << par->dT_BA << ""
//		<< " hyst:" << par->hyst << "}\ns: " << s <<" \nnominal_acceleration: "<<nominal_acceleration<< "\n"
//		<< "velocity_desired:" << velocity_desired << " \nego_vehicle.speed:"<<ego_vehicle.speed << "\n"
//		<< "other.speed:"<<vehicle.speed << "\nego_vehicle.distance:"<<ego_vehicle.distance<<" \nother.distance:"<<vehicle.distance<<")\n = "<<r << std::endl;
	if (r == C2_STATE_MERGING) {
		return Merge;
	} else {
		return Stop;
	}
}

MergeFeasabilityCheck::Result MergeFeasabilityCheck::test(const MergeFeasabilityCheck::Entity& ego_vehicle, const MergeFeasabilityCheck::Entities& vehicles, const double velocity_desired)
{
	assert(vehicles.size());

	for (Entities::const_iterator iter = vehicles.begin(); iter!=vehicles.end(); ++iter) {
		Result r = test(ego_vehicle, *iter, velocity_desired);
		if (r == Stop) {
			state = Stop;
			return state;
		}
	}
	state = Merge;
	return state;
}

MergeFeasabilityCheck::Entity MergeFeasabilityCheck::getEntity(const CGAL_Geometry::Point_2& merging_point, const CGAL_Geometry::Point_2& vehicle, const double yaw, const double speed)
{
	using namespace CGAL_Geometry;
	Vector_2 mp_vector(merging_point, vehicle);
	Vector_2 veh_vector(cos(yaw), sin(yaw));
	double dist_sign = 1.0;
	double speed_sign = 1.0;
	double mp_sqr_length = mp_vector.squared_length();
	if ((mp_vector + veh_vector).squared_length() > mp_sqr_length) {
		dist_sign = -1.0;
	}
	if (speed < -0.1) {
		speed_sign = -1.0;
		dist_sign *= -1.0;
		//assert(false);
	}

	return Entity(dist_sign * sqrt(mp_sqr_length), speed_sign * speed);
}

struct StackEntry {
	StackEntry(RndfEdge* edge, double dist) : edge(edge), dist(dist) {};
	RndfEdge* edge;
	double dist;
};

MergeFeasabilityCheck::Entities MergeFeasabilityCheck::getEntities(RndfEdge* edge, double offset, double dist_in_from_direction, double dist_in_to_direction, bool same_lane)
{

	assert(edge);
	Entities result;
	std::deque<StackEntry> stack;
	stack.push_back(StackEntry(edge, edge->length() - offset));
//	TRACE("++++++++++++++++++");
	while (stack.size()) {
		StackEntry current = stack.front();
		stack.pop_front();
		assert(current.edge);
		assert(current.edge->fromVertex());
		assert(current.edge->toVertex());
		double d = current.dist;
//		TRACE("searching "<<current.edge->name() << " d="<<d);

		for (std::map<int, Vehicle*>::const_iterator iter=current.edge->vehicles_on_edge.begin();
				iter!=current.edge->vehicles_on_edge.end(); ++iter) {
			double v_d = d - iter->second->distToEnd();
			if (v_d < 0) {
				result.push_back(Entity( ( same_lane ? -v_d : v_d ), iter->second->speed(), iter->second));
//				TRACE("   found vehicle (added) "<<iter->second);
			} else {
//				TRACE("   found vehicle (not added) "<<iter->second);
			}
		}

		d -= current.edge->length();
		if (d > -dist_in_from_direction) {
//			TRACE("  fromvertex " << current.edge->fromVertex()->name() << " size="<< current.edge->fromVertex()->inEdges().size());
			const std::set< RndfEdge* >& edges = current.edge->fromVertex()->inEdges();
			for (std::set< RndfEdge* >::const_iterator iter = edges.begin();	iter != edges.end(); ++iter) {
				assert(*iter != current.edge);
				assert(*iter);
				stack.push_back(StackEntry(*iter, d));
//				TRACE("  adding "<<(*iter)->name() << " d="<<d);
			}
		}

	}
//	TRACE("::::::::::::::::");
	stack.push_back(StackEntry(edge, -offset));
	while (stack.size()) {
		StackEntry current = stack.front();
		stack.pop_front();
		assert(current.edge);
		double d = current.dist;
//		TRACE("searching "<<current.edge->name() << " d="<<d);

		for (std::map<int, Vehicle*>::const_iterator iter=current.edge->vehicles_on_edge.begin();
				iter!=current.edge->vehicles_on_edge.end(); ++iter) {
			double v_d = d + iter->second->distFromStart();
			if (v_d > 0) {
				result.push_back(Entity( ( same_lane ? v_d : -v_d ) , iter->second->speed(), iter->second));
//				TRACE("   found vehicle (added) "<<iter->second);
			} else {
//				TRACE("   found vehicle (not added) "<<iter->second);
			}
		}

		d += current.edge->length();
		if (d < dist_in_to_direction) {
//			TRACE("  tovertex " << current.edge->toVertex()->name() << " size="<< current.edge->toVertex()->outEdges().size());
			for (RndfVertex::TEdgeSet::const_iterator iter = current.edge->toVertex()->outEdges().begin(); iter != current.edge->toVertex()->outEdges().end(); ++iter) {
				assert(*iter != current.edge);
				assert(*iter);
				stack.push_back(StackEntry(*iter, d));
//				TRACE("  adding "<<(*iter)->name() << " d="<<d);
			}
		}

	}
	TRACE("--------------------");

	return result;
}

void MergeFeasabilityCheck::setEgoGeoConstrain(double before_MP, double after_MP)
{
	par->D_A1 = before_MP;
	par->D_A2 = after_MP;
}
void MergeFeasabilityCheck::setOtherGeoConstrain(double before_MP, double after_MP)
{
	par->D_B1 = before_MP;
	par->D_B2 = after_MP;
}
void MergeFeasabilityCheck::setTimeConstrain(double ego_before_other, double other_before_ego)
{
	par->dT_AB = ego_before_other;
	par->dT_BA = other_before_ego;
}
void MergeFeasabilityCheck::setHysterese(double hyst)
{
	par->hyst = hyst;
}

} // namespace vlr
