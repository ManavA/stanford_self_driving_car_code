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


#ifndef AW_STBASE_HPP
#define AW_STBASE_HPP

#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <aw_ChsmPlanner.hpp>

namespace vlr {

enum TransitionEnum {
	TRANSIT_DEFAULT,
	TRANSIT_TO_GOAL,
	TRANSIT_TO_KTURN,
	TRANSIT_TO_INTERSECTION,
	TRANSIT_TO_ZONE,
	TRANSIT_TO_LANECHANGE,
  TRANSIT_TO_STOPLINE,
  TRANSIT_TO_TRAFFIC_LIGHT,
  TRANSIT_TO_CROSSWALK
};

//! Base for all states.
/*! Using the curiously recurring template pattern to get the ChsmPlanner context in this base class.
 */
template < class DerivedState >
class StBase
{
public:
	// on enter
	StBase(const std::string& n) : recovery_time(RECOVERY_TIMEOUT), name_(n) {
		// entry
		ChsmPlanner& planner = static_cast< DerivedState* >(this)->template context<ChsmPlanner>();
		planner.addMessage("Entering " + name());
		std::cout << "[CHSM] Entering " << name() << std::endl;
		clearRecoveryIndicator();
		savedTransitionCounter = planner.transitionCounter;
	};
	// on exit
	virtual ~StBase() {
		ChsmPlanner& planner = static_cast< DerivedState* >(this)->template context<ChsmPlanner>();
		planner.addMessage("Exiting " + name());
		std::cout << "[CHSM] Exiting " << name() << std::endl;
		//assert(savedTransitionCounter != planner.transitionCounter && "you have to call detectedErrornousTransitions() in EVERY react() method");
	};
	// returns the name of this state
	std::string name() const { return name_; };

	bool checkRecovery() {
	  // measure progress
	  ChsmPlanner& planner = static_cast< DerivedState* >(this)->template context<ChsmPlanner>();
	  if (!planner.params().enable_recovery) {
	  	clearRecoveryIndicator();
	  	return false;
	  }
	  if (planner.distance(recovery_last_pose_) > RECOVERY_PROGRESS_INDICATOR) {
	  	clearRecoveryIndicator();
	  	return false;
	  }

	  // check expiration
	  //std::cout << "recovery_timeout for " << name() << " is " << recovery_timeout << std::endl;
	  return driving_common::Time::current() > recovery_timeout;
	}
	virtual void setRecoveryTime(double time) {
		recovery_time = time;
		clearRecoveryIndicator();
	}
	//! resets the recovery indicator so that recovery timer starts over again
	/*! use with care because it can easyly countermeasure recover mode if called too often */
	void clearRecoveryIndicator() {
		recovery_timeout = driving_common::Time::current();
		recovery_timeout += recovery_time;
		recovery_last_pose_ = static_cast< DerivedState* >(this)->template context< ChsmPlanner>().currentPose(); // TODO: causes mutex lock
	}
protected:
	bool isExpired(double t) {
		return driving_common::Time::current() > t;
	}

	bool detectedErrornousTransitions() {

		ChsmPlanner& planner = static_cast< DerivedState* >(this)->template context< ChsmPlanner>();

		planner.transitionCounter = planner.transitionCounter+1;
		if (planner.transitionCounter > 16) { // doesn't work, don"t know why
			planner.addMessage("errornous transitions detected");
			return true;
		} else {
			return false;
		}

	}
  double recovery_timeout;
  driving_common::GlobalPose recovery_last_pose_;
  double recovery_time;
private:
	std::string name_;
	uint savedTransitionCounter;
};

} // namespace vlr

#endif // AW_STBASE_HPP
