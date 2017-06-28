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


#ifndef AW_STZONE_HPP
#define AW_STZONE_HPP

#include <aw_StBase.hpp>
#include <aw_StActive.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StZone
 *---------------------------------------------------------------------------*/
struct StZoneApproach;
struct StZone : sc::state< StZone, StActive , StZoneApproach>, StBase<StZone> {
  // on enter
  StZone(my_context ctx);
  // on exit
  ~StZone();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  //! list of the parking maneuers ins this zone
  std::vector<AnnotatedRouteEdge*> zone_maneuvers;

  int no_perimeter_points;
};

/*---------------------------------------------------------------------------
 * StZoneApproach
 *---------------------------------------------------------------------------*/
struct StZoneApproach: sc::state< StZoneApproach, StZone >, StBase<StZoneApproach>
{
  // on enter
  StZoneApproach(my_context ctx);
  // on exit
  ~StZoneApproach();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StZoneEntering
 *---------------------------------------------------------------------------*/
struct StZoneEntering : sc::state< StZoneEntering, StZone>, StBase<StZoneEntering> {
  // on enter
  StZoneEntering(my_context ctx);
  // on exit
  ~StZoneEntering();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StZoneParking
 *---------------------------------------------------------------------------*/
struct StZoneParking : sc::state< StZoneParking, StZone>, StBase<StZoneParking> {
  // on enter
  StZoneParking(my_context ctx);
  // on exit
  ~StZoneParking();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StZoneParked
 *---------------------------------------------------------------------------*/
struct StZoneParked : sc::state< StZoneParked, StZone>, StBase<StZoneParked> {
  // on enter
  StZoneParked(my_context ctx);
  // on exit
  ~StZoneParked();

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
 * StZoneDriveToExit
 *---------------------------------------------------------------------------*/
struct StZoneDriveToExit : sc::state< StZoneDriveToExit, StZone>, StBase<StZoneDriveToExit> {
  // on enter
  StZoneDriveToExit(my_context ctx);
  // on exit
  ~StZoneDriveToExit();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StZoneRecover
 *---------------------------------------------------------------------------*/
struct StZoneRecover : sc::state< StZoneRecover, StZone>, StBase<StZoneRecover> {
  // on enter
  StZoneRecover(my_context ctx);
  // on exit
  ~StZoneRecover();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // AW_STZONE_HPP
