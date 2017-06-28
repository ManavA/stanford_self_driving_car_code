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


#ifndef AW_STLANECHANGE_HPP
#define AW_STLANECHANGE_HPP

#include <aw_StBase.hpp>
#include <aw_StActive.hpp>
#include <aw_StLaneChangeTypes.hpp>
#include <aw_graph_tools.hpp>

namespace vlr {

struct StPrepareLaneChange;

using GraphTools::PlaceOnGraph;


/*---------------------------------------------------------------------------
 * StLaneChange
 *---------------------------------------------------------------------------*/
struct StLaneChange : sc::state< StLaneChange, StDrive, StPrepareLaneChange >, StBase<StLaneChange>
{
	// on enter
	StLaneChange(my_context ctx);
	// on exit
	~StLaneChange();

	//StLaneChange(const StLaneChange& src); - dont know how to do that

	// reactions
	sc::result react(const EvProcess& evt);
	sc::result react(const EvPause& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >,
	sc::custom_reaction< EvPause >
	> reactions;


	bool mergePossible(double desired_speed, bool check_only_forward = true);

	bool placeIsValid(const GraphPlace& place) const;
	bool snailThroughPossible( const Vehicle& base_vehicle, StLaneChangeLaneChangeType change_type ) const;

//	void setLaneChangeTentacleParams();
//	void storeTentacleParams();
//	void restoreTentacleParams();

	void restorePauseState();


	PlaceOnGraph change_point;
	GraphPlace merge_point;
	GraphPlace merge_end_point;
	StLaneChangeLaneChangeReason change_reason;
	StLaneChangeLaneChangeType change_type;
	double lateral_offset;               // offset between the two lanes
	double change_length;                // length of the change
	double merge_length;                 // length of the merging zone
	bool merge_allowed;
	bool has_to_stop;
	StLaneChangeRecoverType recover_type;

//	MasterOfTheTentacles::Parameters old_params;


	MergeFeasabilityCheck* mfc;
	std::map<Vehicle*, double> obstacles_in_merge_zone;
	std::map<Vehicle*, double> merging_suckers;	// cars that prohibit lane changing due to negativ mergecheck
};

/*---------------------------------------------------------------------------
 * StPrepareLaneChange
 *---------------------------------------------------------------------------*/
struct StPrepareLaneChange : sc::state< StPrepareLaneChange, StLaneChange >, StBase<StPrepareLaneChange>
{
	// on enter
	StPrepareLaneChange(my_context ctx);
	// on exit
	~StPrepareLaneChange();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	double prep_start;
};

/*---------------------------------------------------------------------------
 * StChangeToLeftLane
 *---------------------------------------------------------------------------*/
struct StChangeToLeftLane : sc::state< StChangeToLeftLane, StLaneChange >, StBase<StChangeToLeftLane>
{
	// on enter
	StChangeToLeftLane(my_context ctx);
	// on exit
	~StChangeToLeftLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StChangeToRightLane
 *---------------------------------------------------------------------------*/
struct StChangeToRightLane : sc::state< StChangeToRightLane, StLaneChange >, StBase<StChangeToRightLane>
{
	// on enter
	StChangeToRightLane(my_context ctx);
	// on exit
	~StChangeToRightLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StChangeToLeftOncomingLane
 *---------------------------------------------------------------------------*/
struct StChangeToLeftOncomingLane : sc::state< StChangeToLeftOncomingLane, StLaneChange >, StBase<StChangeToLeftOncomingLane>
{
	// on enter
	StChangeToLeftOncomingLane(my_context ctx);
	// on exit
	~StChangeToLeftOncomingLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StDriveOnOncomingLane
 *---------------------------------------------------------------------------*/
struct StDriveOnOncomingLane : sc::state< StDriveOnOncomingLane, StLaneChange >, StBase<StDriveOnOncomingLane>
{
	// on enter
	StDriveOnOncomingLane(my_context ctx);
	// on exit
	~StDriveOnOncomingLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};


/*---------------------------------------------------------------------------
 * StReturnFromOncomingLane
 *---------------------------------------------------------------------------*/
struct StReturnFromOncomingLane : sc::state< StReturnFromOncomingLane, StLaneChange >, StBase<StReturnFromOncomingLane>
{
	// on enter
	StReturnFromOncomingLane(my_context ctx);
	// on exit
	~StReturnFromOncomingLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};


/*---------------------------------------------------------------------------
 * StReturnFromOncomingLane
 *---------------------------------------------------------------------------*/
struct StAbortLaneChange : sc::state< StAbortLaneChange, StLaneChange >, StBase<StAbortLaneChange>
{
	// on enter
	StAbortLaneChange(my_context ctx);
	// on exit
	~StAbortLaneChange();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};


/*---------------------------------------------------------------------------
 * StLaneChangeRecover
 *---------------------------------------------------------------------------*/
class StLaneChangeRecoverPrepare;
struct StLaneChangeRecover: sc::state< StLaneChangeRecover, StLaneChange, StLaneChangeRecoverPrepare >, StBase<StLaneChangeRecover>
{
	// on enter
	StLaneChangeRecover(my_context ctx);
	// on exit
	~StLaneChangeRecover();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	double map_timer;
};

/*---------------------------------------------------------------------------
 * StLaneChangeRecoverPrepare
 *---------------------------------------------------------------------------*/
struct StLaneChangeRecoverPrepare: sc::state< StLaneChangeRecoverPrepare, StLaneChangeRecover >, StBase<StLaneChangeRecoverPrepare>
{
	// on enter
	StLaneChangeRecoverPrepare(my_context ctx);
	// on exit
	~StLaneChangeRecoverPrepare();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

//=============================================================================
//		Convenience Functions
//=============================================================================

bool leftLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length);
bool rightLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length);
bool leftOncomingLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length);

} // namespace vlr

#endif // AW_STLANECHANGE_HPP
