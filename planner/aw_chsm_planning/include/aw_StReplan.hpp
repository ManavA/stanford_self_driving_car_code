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


#ifndef AW_STREPLAN_HPP
#define AW_STREPLAN_HPP

#include <aw_StBase.hpp>
#include <aw_StActive.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StReplan
 *---------------------------------------------------------------------------*/
class StReplanStop;
struct StReplan : sc::state< StReplan, StActive, StReplanStop >, StBase<StReplan> {
  // on enter
	StReplan(my_context ctx);
  // on exit
  ~StReplan();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

struct StReplanStop : sc::state< StReplanStop, StReplan >, StBase<StReplanStop> {
  // on enter
	StReplanStop(my_context ctx);
  // on exit
  ~StReplanStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StReroute
 *---------------------------------------------------------------------------*/
struct StReroute : sc::state< StReroute, StReplan >, StBase<StReroute> {
  // on enter
	StReroute(my_context ctx);
  // on exit
  ~StReroute();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StGetBackOnTrack
 *---------------------------------------------------------------------------*/
class StGetBackOnTrackPrepare;
struct StGetBackOnTrack : sc::state< StGetBackOnTrack, StReplan, StGetBackOnTrackPrepare >, StBase<StGetBackOnTrack> {
  // on enter
	StGetBackOnTrack(my_context ctx);
  // on exit
  ~StGetBackOnTrack();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

	//! before starting the A* we wait some time so that the map becomes available
	double map_timer;
};

/*---------------------------------------------------------------------------
 * StGetBackOnTrackPrepare
 *---------------------------------------------------------------------------*/
struct StGetBackOnTrackPrepare : sc::state< StGetBackOnTrackPrepare, StGetBackOnTrack >, StBase<StGetBackOnTrackPrepare> {
  // on enter
	StGetBackOnTrackPrepare(my_context ctx);
  // on exit
  ~StGetBackOnTrackPrepare();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

};

/*---------------------------------------------------------------------------
 * StGetBackOnTrackAStar
 *---------------------------------------------------------------------------*/
struct StGetBackOnTrackAStar : sc::state< StGetBackOnTrackAStar, StGetBackOnTrack >, StBase<StGetBackOnTrackAStar> {
  // on enter
	StGetBackOnTrackAStar(my_context ctx);
  // on exit
  ~StGetBackOnTrackAStar();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;


  double next_replan;

};

} // namespace vlr

#endif // AW_STREPLAN_HPP
