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


#ifndef AW_STDRIVEPASSOBSTACLE_HPP
#define AW_STDRIVEPASSOBSTACLE_HPP


#include <aw_StBase.hpp>
#include <aw_StActive.hpp>

#include <aw_LaneChangeManager.hpp>
#include <aw_PassObstacleManager.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StDrivePassObstacle
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleBranch;
struct StDrivePassObstacle : sc::state< StDrivePassObstacle, StDrive, StDrivePassObstacleBranch>, StBase<StDrivePassObstacle> {
  // on enter
  StDrivePassObstacle(my_context ctx);
  // on exit
  ~StDrivePassObstacle();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  LaneChangeManager* lcm;
  PassObstacleManager* pom/*mes*/;
};

/*---------------------------------------------------------------------------
 * StDrivePassObstacleBranch
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleBranch : sc::state< StDrivePassObstacleBranch, StDrivePassObstacle>, StBase<StDrivePassObstacleBranch> {
  // on enter
  StDrivePassObstacleBranch(my_context ctx);
  // on exit
  ~StDrivePassObstacleBranch();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  double decision_time;

};


/*---------------------------------------------------------------------------
 * StDrivePassObstacleStop
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleStop : sc::state< StDrivePassObstacleStop, StDrivePassObstacle>, StBase<StDrivePassObstacleStop> {
  // on enter
  StDrivePassObstacleStop(my_context ctx);
  // on exit
  ~StDrivePassObstacleStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StDrivePassObstacleWait
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleWait : sc::state< StDrivePassObstacleWait, StDrivePassObstacle>, StBase<StDrivePassObstacleWait> {
  // on enter
  StDrivePassObstacleWait(my_context ctx);
  // on exit
  ~StDrivePassObstacleWait();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  double waitTimeout;
  double congestionTimeout;
};


/*---------------------------------------------------------------------------
 * StDrivePassObstaclePass
 *---------------------------------------------------------------------------*/
struct StDrivePassObstaclePass : sc::state< StDrivePassObstaclePass, StDrivePassObstacle>, StBase<StDrivePassObstaclePass> {
  // on enter
  StDrivePassObstaclePass(my_context ctx);
  // on exit
  ~StDrivePassObstaclePass();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StDrivePassObstacleChangeLanes
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleChangeLanes : sc::state< StDrivePassObstacleChangeLanes, StDrivePassObstacle>, StBase<StDrivePassObstacleChangeLanes> {
  // on enter
  StDrivePassObstacleChangeLanes(my_context ctx);
  // on exit
  ~StDrivePassObstacleChangeLanes();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // STDRIVEPASSOBSTACLE_HPP
