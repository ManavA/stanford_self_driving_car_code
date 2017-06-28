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


#ifndef AW_STDRIVE_HPP
#define AW_STDRIVE_HPP

#include <aw_StBase.hpp>
#include <aw_StActive.hpp>
#include <aw_ChsmEvents.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StDrive
 *---------------------------------------------------------------------------*/
struct StDriveStart;
struct StIntersection;
struct StDrive: sc::state< StDrive, StActive, StDriveStart>, StBase<StDrive>
{
  // on enter
  StDrive(my_context ctx);
  // on exit
  virtual ~StDrive();

  // reactions
  sc::result react(const EvProcess& evt);
  sc::result react(const EvStop& evt);


  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >,
    sc::custom_reaction< EvStop >,
    sc::transition< EvCloseToIntersection, StIntersection >
  > reactions;

};


/*---------------------------------------------------------------------------
 * StDriveStart
 *---------------------------------------------------------------------------*/
struct StDriveStart: sc::state< StDriveStart, StDrive >, StBase<StDriveStart>
{
  // on enter
	StDriveStart(my_context ctx);
  // on exit
  virtual ~StDriveStart();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StDriveOnLane
 *---------------------------------------------------------------------------*/
struct StDriveOnLane: sc::state< StDriveOnLane, StDrive >, StBase<StDriveOnLane>
{
  // on enter
  StDriveOnLane(my_context ctx);
  // on exit
  virtual ~StDriveOnLane();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StDriveRecover
 *---------------------------------------------------------------------------*/
class StDriveRecoverPrepare;
struct StDriveRecover: sc::state< StDriveRecover, StDrive, StDriveRecoverPrepare >, StBase<StDriveRecover>
{
  // on enter
  StDriveRecover(my_context ctx);
  // on exit
  virtual ~StDriveRecover();

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
 * StDriveRecoverPrepare
 *---------------------------------------------------------------------------*/
struct StDriveRecoverPrepare: sc::state< StDriveRecoverPrepare, StDriveRecover >, StBase<StDriveRecoverPrepare>
{
  // on enter
	StDriveRecoverPrepare(my_context ctx);
  // on exit
  virtual ~StDriveRecoverPrepare();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StDriveStop
 *---------------------------------------------------------------------------*/
struct StDriveStop: sc::state< StDriveStop, StDrive >, StBase<StDriveStop>
{
  // on enter
	StDriveStop(my_context ctx);
  // on exit
  virtual ~StDriveStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};
struct StDriveStopped: sc::state< StDriveStopped, StDrive >, StBase<StDriveStopped>
{
  // on enter
	StDriveStopped(my_context ctx);
  // on exit
  virtual ~StDriveStopped();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;

  double stop_timer;
};

} // namespace vlr

#include "aw_StDriveKTurn.hpp"
#include "aw_StDrivePassObstacle.hpp"

#endif // AW_STDRIVE_HPP
