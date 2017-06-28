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


#ifndef PASSAT_H_
#define PASSAT_H_

#include <ros/ros.h>
#include <driving_common/CanStatus.h>
#include <driving_common/Actuator.h>
#include <driving_common/EStopStatus.h>
#include <driving_common/Heartbeat.h>
#include <passat/PassatState.h>
#include <passatcore.h>

namespace vlr {
class Passat : public PassatCore {
public:
  typedef enum {
    INITIALIZING_WAIT_FOR_DATA,

    PAUSEFOR_STOP_VEHICLE,
    PAUSEFOR_PRESHIFT_WAIT,
    PAUSEFOR_SHIFT_TO_PARK,
    PAUSEFOR_WAIT_WITH_BRAKE,
    PAUSEFOR_ENABLE_PARKING_BRAKE,
    PAUSEFOR_WAIT,
    PAUSEFOR_ENABLE_BRAKE,
    PAUSEFOR_DISABLE_PARKING_BRAKE,
    PAUSEFOR_SHIFT_TO_DRIVE,
    PAUSEFOR_WAIT_5SEC,

    PAUSEREV_STOP_VEHICLE,
    PAUSEREV_PRESHIFT_WAIT,
    PAUSEREV_SHIFT_TO_PARK,
    PAUSEREV_WAIT_WITH_BRAKE,
    PAUSEREV_ENABLE_PARKING_BRAKE,
    PAUSEREV_WAIT,
    PAUSEREV_ENABLE_BRAKE,
    PAUSEREV_DISABLE_PARKING_BRAKE,
    PAUSEREV_SHIFT_TO_REVERSE,
    PAUSEREV_WAIT_5SEC,

    FORWARD_STOP_VEHICLE,
    FORWARD_PRESHIFT_WAIT,
    FORWARD_SHIFT_TO_DRIVE,
    FORWARD_GO,

    REVERSE_STOP_VEHICLE,
    REVERSE_PRESHIFT_WAIT,
    REVERSE_SHIFT_TO_REVERSE,
    REVERSE_GO,

    FINISH_STOP_VEHICLE,
    FINISH_PRESHIFT_WAIT,
    FINISH_SHIFT_TO_PARK,
    FINISH_WAIT
  } FsmState;

public:
  Passat();
  virtual ~Passat();

  void run();

private:
  void readParameters();
  void actuatorHandler(const driving_common::Actuator& actuator);
  void turnSignalHandler(const driving_common::TurnSignal& signal);
  void canStatusHandler(const driving_common::CanStatus& status);
  void estopStatusHandler(const driving_common::EStopStatus& status);

  int forwardGear(int target_gear);
  int reverseGear(int target_gear);

  void update();

private:
  ros::Subscriber can_status_sub_;
  ros::Subscriber estop_status_sub_;
  ros::Subscriber turn_signal_sub_;
  ros::Subscriber actuator_sub_;

  ros::Publisher heartbeat_pub_;
  ros::Publisher fsm_state_pub_;

  driving_common::CanStatus can_status_;
  driving_common::Heartbeat heartbeat_;
  passat::PassatState fsm_state_;

  bool use_fsm_;
  double estop_brake_;
  bool received_can_status_, received_estop_status_;
  bool estop_run_status_, estop_enable_status_;
  double can_velocity_;
  FsmState state_;
  double shift_time_, siren_time_, brake_time_;
  double first_ts_;
  double start_wait_with_brake_;
  bool startup_test_;
  double first_startup_test_ts_;
};

} // namespace vlr
#endif
