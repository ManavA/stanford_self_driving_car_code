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


#ifndef AW_MANEUVER_H
#define AW_MANEUVER_H

namespace vlr {

enum maneuver_t
{
  UC_MANEUVER_START_MISSION, //!< before mission start
  UC_MANEUVER_TRAVEL, //!< normal travel
  UC_MANEUVER_STOP_SIGN, //!< driving towards a stop sign
  UC_MANEUVER_CROSSWALK, //!< driving towards a crosswalk
  UC_MANEUVER_TRAFFIC_LIGHT, //!< driving towards a traffic light
  UC_MANEUVER_INT_TURN_RIGHT, //!< turnoff at intersection required (apply indicator!)
  UC_MANEUVER_INT_TURN_LEFT,
  UC_MANEUVER_INT_STRAIGHT,
  UC_MANEUVER_CURVE_RIGHT, //!< curves without intersection
  UC_MANEUVER_CURVE_LEFT,
  UC_MANEUVER_LANECHANGE_RIGHT, //!< lane changes
  UC_MANEUVER_LANECHANGE_LEFT,
  UC_MANEUVER_U_TURN, //!< U turn
  UC_MANEUVER_NAVIGATE, //!< free navigation behavior in parking zones
  UC_MANEUVER_PARKING, //!< parking spot
  UC_MANEUVER_CHECKPOINT, //!< other checkpoint (cross precisely!)
  UC_K_TURN,        //!< Dreipunktwende
  UC_MANEUVER_GOAL_REACHED, //!< goal checkpoint has been reached, stop
  UC_MANEUVER_ZONE_ENTRY, //!< entering a zone
  UC_MANEUVER_ZONE_EXIT, //!< exiting a zone
};

enum area_type_t
{
  UC_TRAVEL_AREA,
  UC_ZONE,
  UC_PARKING_ZONE
};

} // namespace vlr

#endif // AW_MANEUVER_H
