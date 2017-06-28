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


#ifndef AW_ST_CROSSWALK_HPP
#define AW_ST_CROSSWALK_HPP

#include <aw_StBase.hpp>
#include <aw_StActive.hpp>
#include <aw_CrosswalkManager.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StCrosswalk
 *---------------------------------------------------------------------------*/
struct StCrosswalkApproach;
struct StCrosswalk : sc::state< StCrosswalk, StActive, StCrosswalkApproach>, StBase<StCrosswalk>
{
	// on enter
	StCrosswalk(my_context ctx);
	// on exit
	~StCrosswalk();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list <	sc::custom_reaction< EvProcess > > reactions;

	// internal
	CrosswalkManager* cwm_;

//private:
	// generates curvepoints taking multi-matching in intersections into account
	void generateCurvepoints(const double stop_distance, const double max_speed);
};

/*---------------------------------------------------------------------------
 * StCrosswalkApproach
 *---------------------------------------------------------------------------*/
struct StCrosswalkApproach: sc::state< StCrosswalkApproach, StCrosswalk >, StBase<StCrosswalkApproach>
{
	// on enter
	StCrosswalkApproach(my_context ctx);
	// on exit
	~StCrosswalkApproach();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

};

/*---------------------------------------------------------------------------
 * StCrosswalkStop
 *---------------------------------------------------------------------------*/
struct StCrosswalkStop: sc::state< StCrosswalkStop, StCrosswalk >, StBase<StCrosswalkStop>
{
	// on enter
	StCrosswalkStop(my_context ctx);
	// on exit
	~StCrosswalkStop();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StCrosswalkWait
 *---------------------------------------------------------------------------*/
struct StCrosswalkWait: sc::state< StCrosswalkWait, StCrosswalk >, StBase<StCrosswalkWait>
{
	// on enter
	StCrosswalkWait(my_context ctx);
	// on exit
	~StCrosswalkWait();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	// internal variables
	double stop_time;
};


/*---------------------------------------------------------------------------
 * StCrosswalkQueue
 *---------------------------------------------------------------------------*/
struct StCrosswalkQueue: sc::state< StCrosswalkQueue, StCrosswalk >, StBase<StCrosswalkQueue>
{
	// on enter
	StCrosswalkQueue(my_context ctx);
	// on exit
	~StCrosswalkQueue();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	double congestionTimeout;
	driving_common::GlobalPose congestion_last_pose_;
};

} // namespace vlr

#endif // AW_ST_CROSSWALK_HPP
