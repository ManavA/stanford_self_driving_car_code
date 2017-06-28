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


#ifndef AW_CHSMEVENTS_HPP
#define AW_CHSMEVENTS_HPP

#include <boost/statechart/event.hpp>
#include <boost/pool/pool_alloc.hpp>

namespace sc = boost::statechart;

namespace vlr {
//! signals the desire to go in pause mode.
/*!
 * This event is fired when the "Pause" button on the remote was pressed.
 */
struct EvPause : sc::event< EvPause, boost::pool_allocator < char > > {};

//! signals the desire to go in active mode.
/*!
 * This event is fired when the "Pause" button on the remote was released.
 */
struct EvActivate : sc::event< EvActivate, boost::pool_allocator < char > > {};

//! The "do work" event. Gets fired every cycle.
/*!
 * On reacting to this event, the states can do what they have to do (e.g. generate curvepoints).
 */
struct EvProcess : sc::event< EvProcess, boost::pool_allocator < char > > {};

//! Gets fired every cycle after EvProcess was fired.
/*!
 * With reacting on this event the states can alter and/or override
 * the output of the EvProcess phase. Safty checks can be implemented as reaction
 * on this event.
 * This event should bubble up to outer states.
 *
 * @remark states _should not_ call transit<>() as reaction on this special event.
 *         They should nearly always call forward_event() as their last reaction
 *         on this event to bubble it up to outer states so that this states also can
 *         react on the EvAfterProcess event.
 *
 * @see StActive::react(const EvAfterProcess& evt) for an example reaction on this event.
 */
struct EvAfterProcess : sc::event< EvAfterProcess, boost::pool_allocator < char > > {};

struct EvStop : sc::event< EvStop, boost::pool_allocator < char > > {};
struct EvDrive : sc::event< EvDrive, boost::pool_allocator < char > > {};
struct EvCloseToIntersection : sc::event< EvCloseToIntersection, boost::pool_allocator < char > > {};

} // namespace vlr

#endif // CHSMEVENTS_HPP_
