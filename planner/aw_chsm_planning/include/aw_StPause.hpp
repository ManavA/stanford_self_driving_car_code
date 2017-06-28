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


#ifndef AW_STPAUSE_HPP
#define AW_STPAUSE_HPP

#include <aw_StBase.hpp>
#include <aw_ChsmPlanner.hpp>

namespace vlr {

struct StWaitForActivation;
struct StPauseShortTerm;
struct StPause: public sc::state< StPause, ChsmPlanner, StPauseShortTerm >, public StBase< StPause >
{
  // on enter
  StPause(my_context ctx);
  // on exit
  ~StPause();
  // reaction
  sc::result react(const EvProcess& evt);
  sc::result react(const sc::exception_thrown& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >,
    sc::transition< EvActivate, StWaitForActivation >,
    sc::custom_reaction< sc::exception_thrown>
  > reactions;

  //CurvePoints save_curvepoints;
};

struct StPauseShortTerm: public sc::state< StPauseShortTerm, StPause >, public StBase< StPauseShortTerm >
{
  // on enter
  StPauseShortTerm(my_context ctx);
  // on exit
  ~StPauseShortTerm();
  // reaction
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;

  double switchTime;
};

struct StPauseLongTerm: public sc::state< StPauseLongTerm, StPause >, public StBase< StPauseLongTerm >
{
  // on enter
  StPauseLongTerm(my_context ctx);
  // on exit
  ~StPauseLongTerm();
  // reaction
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // AW_STPAUSE_HPP
