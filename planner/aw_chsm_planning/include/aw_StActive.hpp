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


#ifndef AW_STACTIVE_HPP
#define AW_STACTIVE_HPP

#include <aw_StBase.hpp>
#include <aw_ChsmPlanner.hpp>

namespace vlr {

struct StPause;
struct StDrive;

///! history pseudo state
typedef sc::deep_history< StDrive > StActiveHistory;
struct StActive: sc::state< StActive, ChsmPlanner, StDrive, sc::has_deep_history >, StBase< StActive >
{
  // on enter
  StActive(my_context ctx);
  // on exit
  virtual ~StActive();
  // reactions
  sc::result react(const EvProcess& evt);
  sc::result react(const EvAfterProcess& evt);
  sc::result react(const sc::exception_thrown& evt);

  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >,
    sc::transition< EvPause, StPause >,
    sc::custom_reaction< sc::exception_thrown>
  > reactions;
};

// if state-machine is errornous this state will come to the rescue
struct StGlobalRecover: sc::state< StGlobalRecover, StActive >, StBase< StGlobalRecover >
{
  // on enter
	StGlobalRecover(my_context ctx);
  // on exit
  virtual ~StGlobalRecover();
  // reactions
  sc::result react(const EvAfterProcess& evt);

  typedef mpl::list
  <
    sc::custom_reaction< EvAfterProcess >
  > reactions;

  bool done;
  RndfVertex* checkpoint;
};

} // namespace vlr

#include <aw_StDrive.hpp>
#include <aw_StStop.hpp>
#include <aw_StIntersection.hpp>
#include <aw_StZone.hpp>

#endif // AW_STACTIVE_HPP
