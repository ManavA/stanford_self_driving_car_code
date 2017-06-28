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


#ifndef AW_STTRAFFICLIGHT_HPP_
#define AW_STTRAFFICLIGHT_HPP_

#include <aw_StBase.hpp>
#include <aw_StActive.hpp>
#include <aw_TrafficLightManager.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StTrafficLight
 *---------------------------------------------------------------------------*/
struct StTrafficLightApproach;
struct StTrafficLight : sc::state< StTrafficLight, StActive, StTrafficLightApproach>, StBase<StTrafficLight>
{
  // on enter
  StTrafficLight(my_context ctx);
  // on exit
  ~StTrafficLight();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list < sc::custom_reaction< EvProcess > > reactions;

  // internal
  TrafficLightManager* tlm_;

//private:
  // generates curvepoints taking multi-matching in intersections into account
  void generateCurvepoints(const double stop_distance, const double max_speed);
};

/*---------------------------------------------------------------------------
 * StTrafficLightApproach
 *---------------------------------------------------------------------------*/
struct StTrafficLightApproach: sc::state< StTrafficLightApproach, StTrafficLight >, StBase<StTrafficLightApproach>
{
  // on enter
  StTrafficLightApproach(my_context ctx);
  // on exit
  ~StTrafficLightApproach();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

};

/*---------------------------------------------------------------------------
 * StTrafficLightStop
 *---------------------------------------------------------------------------*/
struct StTrafficLightStop:
        sc::state< StTrafficLightStop, StTrafficLight >,
        StBase<StTrafficLightStop>
{
  // on enter
  StTrafficLightStop(my_context ctx);
  // on exit
  ~StTrafficLightStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  // internal variables
};

/*---------------------------------------------------------------------------
 * StTrafficLightWait
 *---------------------------------------------------------------------------*/
struct StTrafficLightWait:
        sc::state< StTrafficLightWait, StTrafficLight>,
        StBase<StTrafficLightWait>
{
  // on enter
  StTrafficLightWait(my_context ctx);
  // on exit
  ~StTrafficLightWait();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  // internal variables
};

/*---------------------------------------------------------------------------
 * StTrafficLightQueue
 *---------------------------------------------------------------------------*/
struct StTrafficLightQueue: sc::state< StTrafficLightQueue, StTrafficLight >, StBase<StTrafficLightQueue>
{
  // on enter
  StTrafficLightQueue(my_context ctx);
  // on exit
  ~StTrafficLightQueue();

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

#endif // AW_STTRAFFICLIGHT_HPP_
