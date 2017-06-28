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


#ifndef AW_STSTOP_HPP
#define AW_STSTOP_HPP

#include <aw_StBase.hpp>
#include <aw_StActive.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StStop
 *---------------------------------------------------------------------------*/
struct StStopping;
struct StStop : sc::state< StStop, StActive , StStopping>, StBase<StStop> {
  // on enter
  StStop(my_context ctx);
  // on exit
  ~StStop();

  // reactions
  sc::result react(const EvProcess& evt);
  sc::result react(const EvDrive& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >,
  sc::custom_reaction< EvDrive >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StStopping
 *---------------------------------------------------------------------------*/
struct StStopping : sc::state< StStopping, StStop >, StBase<StStopping>
{
  // on enter
  StStopping(my_context ctx);
  // on exit
  ~StStopping();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // AW_STSTOP_HPP
