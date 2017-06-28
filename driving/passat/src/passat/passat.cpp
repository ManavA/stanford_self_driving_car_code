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


#include <global.h>
#include <driving_common/TurnSignal.h>
#include <driving_common/EStopStatus.h>
#include <passat/PassatStatus.h>
#include <passat/PassatState.h>
#include <passatcore.h>
#include <passat_fsm.h>

#define      WAIT_BEFORE_SHIFT         0.5

namespace drc = driving_common;

namespace vlr {

Passat::Passat() : use_fsm_(false), received_can_status_(false), received_estop_status_(false),
                   estop_run_status_(false), estop_enable_status_(true), can_velocity_(0.0),
                   state_(INITIALIZING_WAIT_FOR_DATA), startup_test_(true), first_startup_test_ts_(-1.0) {

  readParameters();

  actuator_sub_ = nh_.subscribe("Actuator", 5, &Passat::actuatorHandler, this);
  turn_signal_sub_ = nh_.subscribe("TurnSignal", 5, &Passat::turnSignalHandler, this);
  can_status_sub_ = nh_.subscribe("CanStatus", 5, &Passat::canStatusHandler, this);
  estop_status_sub_ = nh_.subscribe("EStopStatus", 5, &Passat::estopStatusHandler, this);

  heartbeat_pub_ = nh_.advertise<driving_common::Heartbeat>("Heartbeat", 1);
  fsm_state_pub_ = nh_.advertise<passat::PassatState>("PassatState", 1);

  heartbeat_.modulename = "Passat";

  first_ts_ = drc::Time::current();

  if(use_fsm_) {
    steering_commands_allowed_ = false;
    engine_commands_allowed_ = false;
  }
}

Passat::~Passat() {

}

void Passat::readParameters() {
  getParam("vehicle/estop_brake", Passat::estop_brake_);
  getParam("vehicle/enable_fsm", use_fsm_);
}

void Passat::actuatorHandler(const driving_common::Actuator& actuator) {
#ifdef DELAY_WARNING
  double delta_s = 0;
#endif

  requested_direction_ = actuator.direction;
  if(actuator.steering_mode == driving_common::Actuator::TORQUE_CONTROL && !torque_control_) {
    std::cout << "WARNING: received steering torque command in angle control mode.\n";
    //return;
  }
  else if(actuator.steering_mode == driving_common::Actuator::ANGLE_CONTROL && torque_control_) {
    std::cout << "WARNING: received steering angle command in torque control mode.\n";
    //return;
  }

  if (torque_control_) {
    sendTorqueControlCommand(actuator.steering_value, actuator.brake_pressure, actuator.throttle_fraction, requested_gear_);
#ifdef DELAY_WARNING
    delta_s = drc::Time::current() - actuator.timestamp;
    if(delta_s>MAX_TIME_DELAY) {
      fprintf(stderr, "WARNING: delay between PASSAT and CONTROLLER is %.3f sec\n", delta_s);
      dgc_error_send_comment("PASSAT: controller delay for %.3f seconds\n",
          delta_s);
      dgc_error_send_status("PASSAT: controller delay for %.3f seconds\n",
          delta_s);
    }
#endif
  }
  else {
      throw VLRException("Angle control has to be debugged and is therefore disabled.");
      sendAngleControlCommand(actuator.steering_value, actuator.brake_pressure, actuator.throttle_fraction, requested_gear_);
  }

  // TODO: Check if this should be enabled
  //  sendExtendedCommand(requested_signal_, 0, 0, requested_ebrake_);
  }

void Passat::turnSignalHandler(const driving_common::TurnSignal& signal) {
  requested_signal_ = signal.signal;
  sendExtendedCommand(requested_signal_, 0, 0, requested_ebrake_);
}

void Passat::canStatusHandler(const driving_common::CanStatus& status) {
  can_status_ = status;
  can_velocity_ = 0.5 * dgc::dgc_kph2ms(status.wheel_speed_rl + status.wheel_speed_rr);
  received_can_status_ = true;
}

void Passat::estopStatusHandler(const driving_common::EStopStatus& estop) {
  if(estop.estop_code == driving_common::EStopStatus::ESTOP_DISABLE) {
    estop_enable_status_ = false;
  }
  else if(estop.estop_code == driving_common::EStopStatus::ESTOP_PAUSE) {
    estop_run_status_ = false;
  }
  else if(estop.estop_code == driving_common::EStopStatus::ESTOP_RUN) {
    estop_run_status_ = true;
  }

  received_estop_status_ = true;
}

//void Passat::paramChangeHandler() {
//  updateControlLimits(max_steering, max_torque, max_throttle, max_brake);
//  torque_control_ = torque_control;
//  belt_steering_ = belt_steering;
//}

int Passat::forwardGear(int target_gear) {
  if(target_gear == driving_common::CanStatus::TARGET_GEAR_PARK_NEUTRAL)
    return 0;
  else if(target_gear == driving_common::CanStatus::TARGET_GEAR_REVERSE)
    return 0;
  else if(target_gear == driving_common::CanStatus::TARGET_GEAR_ERROR)
    return 0;
  return 1;
}

int Passat::reverseGear(int target_gear) {
  if(target_gear == driving_common::CanStatus::TARGET_GEAR_REVERSE) {
    return 1;
  }
  return 0;
}

void Passat::run() {
  ros::Rate r(50); // 50 hz ; was 10

  double last_heartbeat_time=0;

  while(ros::ok()) {

    readStatus();

    if(use_fsm_) {
      update();
      fsm_state_.fsm_state = state_;
      fsm_state_pub_.publish(fsm_state_);
    }

    double current_time = drc::Time::current();

    if(current_time - last_heartbeat_time > 1.0) {
      heartbeat_pub_.publish(heartbeat_);
      last_heartbeat_time = current_time;
    }

    ros::spinOnce();
    r.sleep();
  }
}

void Passat::update() {
    // no matter what state we are in, if a disable or course complete message comes in, stop the robot
  if(!estop_enable_status_) {
    if (state_ != FINISH_STOP_VEHICLE && state_ != FINISH_PRESHIFT_WAIT && state_ != FINISH_SHIFT_TO_PARK && state_ != FINISH_WAIT) {
      std::cout << "SWITCHING TO FINISH_STOP_VEHICLE.\n";
      state_ = FINISH_STOP_VEHICLE;
    }
  }

  switch(state_) {
    /* INITIALIZING_WAIT_FOR_DATA: waiting to find out state of system */
    case INITIALIZING_WAIT_FOR_DATA:
      /* no steering or engine commands */
      engine_commands_allowed_ = false;
      steering_commands_allowed_ = false;
      sendEngineCommand(0, 0);

      /* wait for estop and can status */
      if(received_estop_status_ && received_can_status_ &&
          drc::Time::current() - first_ts_ > 0.0) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE\n");
        state_ = PAUSEFOR_STOP_VEHICLE;
      }
      break;




      /* PAUSEFOR_STOP_VEHICLE: bring vehicle to zero velocity */
    case PAUSEFOR_STOP_VEHICLE:
      /* OK to steer while braking to pause */
      steering_commands_allowed_ = true;
      engine_commands_allowed_ = false;
      sendEngineCommand(0, estop_brake_);

      /* wait until the vehicle stops */
      if(can_velocity_ == 0) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_PRESHIFT_WAIT\n");
        state_ = PAUSEFOR_PRESHIFT_WAIT;
        shift_time_ = drc::Time::current();
      }
      break;

      /* PAUSEFOR_PRESHIFT_WAIT: wait after stopping before shift */
    case PAUSEFOR_PRESHIFT_WAIT:
      sendCompleteCommand(0, 0, estop_brake_);
      steering_commands_allowed_ = false;
      engine_commands_allowed_ = false;
      if(drc::Time::current() - shift_time_ > WAIT_BEFORE_SHIFT) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_SHIFT_TO_PARK\n");
        state_ = PAUSEFOR_SHIFT_TO_PARK;
      }
      break;

      /* PAUSEFOR_SHIFT_TO_PARK: shift the vehicle into park */
    case PAUSEFOR_SHIFT_TO_PARK:
      if(can_status_.parking_brake) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT\n");
        state_ = PAUSEFOR_WAIT;
      }
      else if(can_status_.target_gear == driving_common::CanStatus::TARGET_GEAR_PARK_NEUTRAL) {
        start_wait_with_brake_ = drc::Time::current();
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT_WITH_BRAKE.\n");
        state_ = PAUSEFOR_WAIT_WITH_BRAKE;
      }
      else
        sendGearShiftCommand(GEAR_NEUTRAL);
      break;

      /* PAUSEFOR_WAIT_WITH_BRAKE: wait with brake on */
    case PAUSEFOR_WAIT_WITH_BRAKE:
      sendCompleteCommand(0, 0, estop_brake_);

      /* if we get run command, start unpausing */
      if(estop_run_status_) {
        siren_time_ = drc::Time::current();
        fprintf(stderr, "SWITCHING TO PAUSEFOR_SHIFT_TO_DRIVE.\n");
        state_ = PAUSEFOR_SHIFT_TO_DRIVE;
      }

      /* if we wait too long in this state, put the parking brake on */
      if(drc::Time::current() - start_wait_with_brake_ > 20) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_ENABLE_PARKING_BRAKE\n");
        state_ = PAUSEFOR_ENABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEFOR_ENABLE_PARKING_BRAKE: turn on the parking brake */
    case PAUSEFOR_ENABLE_PARKING_BRAKE:
      // Temporary May 28 2010 - Remove with working parking brake
      state_ = PAUSEFOR_WAIT;
      if(can_status_.parking_brake) {
        sendParkingBrakeCommand(false);
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT\n");
        state_ = PAUSEFOR_WAIT;
      }
      else 
        sendParkingBrakeCommand(true);
      break;

      /* PAUSEFOR_WAIT: wait for command from remote to start trial */
    case PAUSEFOR_WAIT:
      engine_commands_allowed_ = false;
      steering_commands_allowed_ = false;

      if(startup_test_) {
        if(first_startup_test_ts_ < 0) {
          first_startup_test_ts_ = drc::Time::current();
        }
        if(drc::Time::current() - first_startup_test_ts_ > 0.5)
          startup_test_ = 0;
        sendCompleteCommand(0, 0.2, 0);
        sendParkingBrakeCommand(false);
      }
      else {
        sendCompleteCommand(0, 0, 0);
        sendParkingBrakeCommand(false);
      }

      /* if we get run command, start unpausing */
      if(estop_run_status_) {
        sendCompleteCommand(0, 0, 0);
        sendParkingBrakeCommand(false);

        siren_time_ = drc::Time::current();
        fprintf(stderr, "SWITCHING TO PAUSEFOR_ENABLE_BRAKE.\n");
        state_ = PAUSEFOR_ENABLE_BRAKE;
      }
      break;

      /* PAUSEFOR_ENABLE_BRAKE: put on the brake */
    case PAUSEFOR_ENABLE_BRAKE:
      sendEngineCommand(0, estop_brake_);
      if(can_status_.brake_pressure > estop_brake_ / 2.0) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_DISABLE_PARKING_BRAKE\n");
        state_ = PAUSEFOR_DISABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEFOR_DISABLE_PARKING_BRAKE */
    case PAUSEFOR_DISABLE_PARKING_BRAKE:
      // Temporary May 28 2010 - Remove with working Parking Brake
      state_ = PAUSEFOR_SHIFT_TO_DRIVE;
      break;
      if(!can_status_.parking_brake) {
        sendParkingBrakeCommand(false);
        fprintf(stderr, "SWITCHING TO PAUSEFOR_SHIFT_TO_DRIVE\n");
        state_ = PAUSEFOR_SHIFT_TO_DRIVE;
      }
      else 
        sendParkingBrakeCommand(true);
      break;

      /* PAUSEFOR_SHIFT_TO_DRIVE: shift into drive */
    case PAUSEFOR_SHIFT_TO_DRIVE:
      if(forwardGear(can_status_.target_gear)) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT_5SEC.\n");
        state_ = PAUSEFOR_WAIT_5SEC;
      }
      else 
        sendGearShiftCommand(GEAR_DRIVE);
      break;

      /* PAUSEFOR_WAIT_5SEC: normal operation of vehicle controller */
    case PAUSEFOR_WAIT_5SEC:
      if(drc::Time::current() - siren_time_ > 5.0) {
        sendEngineCommand(0.0, 0.0);
        steering_commands_allowed_ = true;
        engine_commands_allowed_ = true;
        fprintf(stderr, "SWITCHING TO FORWARD_GO.\n");
        state_ = FORWARD_GO;
      }
      break;




      /* PAUSEREV_STOP_VEHICLE: bring vehicle to zero velocity */
    case PAUSEREV_STOP_VEHICLE:
      steering_commands_allowed_ = true;
      engine_commands_allowed_ = false;
      sendEngineCommand(0.0, estop_brake_);

      /* wait until the vehicle stops */
      if(can_velocity_ == 0) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_PRESHIFT_WAIT\n");
        state_ = PAUSEREV_PRESHIFT_WAIT;
        shift_time_ = drc::Time::current();
      }
      break;

      /* PAUSEREV_PRESHIFT_WAIT: wait after stopping before shift */
    case PAUSEREV_PRESHIFT_WAIT:
      sendCompleteCommand(0, 0, estop_brake_);
      steering_commands_allowed_ = false;
      engine_commands_allowed_ = false;
      if(drc::Time::current() - shift_time_ > WAIT_BEFORE_SHIFT) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_SHIFT_TO_PARK\n");
        state_ = PAUSEREV_SHIFT_TO_PARK;
      }
      break;

      /* PAUSEREV_SHIFT_TO_PARK: shift the vehicle into park */
    case PAUSEREV_SHIFT_TO_PARK:
      if(can_status_.target_gear == driving_common::CanStatus::TARGET_GEAR_PARK_NEUTRAL) {
        start_wait_with_brake_ = drc::Time::current();
        fprintf(stderr, "SWITCHING TO PAUSEREV_WAIT_WITH_BRAKE\n");
        state_ = PAUSEREV_WAIT_WITH_BRAKE;
      }
      else 
        sendGearShiftCommand(GEAR_NEUTRAL);
      break;

      /* PAUSEREV_WAIT_WITH_BRAKE: wait with the brake on */
    case PAUSEREV_WAIT_WITH_BRAKE:
      sendCompleteCommand(0, 0, estop_brake_);

      if(estop_run_status_) {
        siren_time_ = drc::Time::current();
        fprintf(stderr, "SWITCHING TO PAUSEREV_SHIFT_TO_REVERSE.\n");
        state_ = PAUSEREV_SHIFT_TO_REVERSE;
      }

      /* if we wait too long in this state, put the parking brake on */
      if(drc::Time::current() - start_wait_with_brake_ > 20) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_ENABLE_PARKING_BRAKE\n");
        state_ = PAUSEREV_ENABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEREV_ENABLE_PARKING_BRAKE: turn on the parking brake */
    case PAUSEREV_ENABLE_PARKING_BRAKE:
      // Temporary May 28 2010 - Remove with working parking brake
      state_ = PAUSEREV_WAIT;
      if(can_status_.parking_brake) {
        sendParkingBrakeCommand(false);
        fprintf(stderr, "SWITCHING TO PAUSEREV_WAIT\n");
        state_ = PAUSEREV_WAIT;
      }
      else 
        sendParkingBrakeCommand(true);
      break;

      /* PAUSEREV_WAIT: wait for command from remote to start trial */
    case PAUSEREV_WAIT:
      steering_commands_allowed_ = false;
      engine_commands_allowed_ = false;
      sendCompleteCommand(0, 0, 0);
      sendParkingBrakeCommand(false);

      if(estop_run_status_) {
        siren_time_ = drc::Time::current();
        fprintf(stderr, "SWITCHING TO PAUSEREV_ENABLE_BRAKE\n");
        state_ = PAUSEREV_ENABLE_BRAKE;
      }
      break;

      /* PAUSEREV_ENABLE_BRAKE: put on the brake */
    case PAUSEREV_ENABLE_BRAKE:
      sendEngineCommand(0, estop_brake_);
      if(can_status_.brake_pressure > estop_brake_ / 2.0) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_DISABLE_PARKING_BRAKE\n");
        state_ = PAUSEREV_DISABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEREV_DISABLE_PARKING_BRAKE */
    case PAUSEREV_DISABLE_PARKING_BRAKE:
      // Temporary
      state_ = PAUSEREV_SHIFT_TO_REVERSE;
      if(!can_status_.parking_brake) {
        sendParkingBrakeCommand(false);
        fprintf(stderr, "SWITCHING TO PAUSEREV_SHIFT_TO_REVERSE\n");
        state_ = PAUSEREV_SHIFT_TO_REVERSE;
      }
      else 
        sendParkingBrakeCommand(true);
      break;

      /* PAUSEREV_SHIFT_TO_REVERSE: shift into reverse */
    case PAUSEREV_SHIFT_TO_REVERSE:
      if(reverseGear(can_status_.target_gear)) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_WAIT_5SEC.\n");
        state_ = PAUSEREV_WAIT_5SEC;
      }
      else
        sendGearShiftCommand(GEAR_REVERSE);
      break;

      /* PAUSEREV_WAIT_5SEC: normal operation of vehicle controller */
    case PAUSEREV_WAIT_5SEC:
      if(drc::Time::current() - siren_time_ > 5.0) {
        /* release the brake */
        sendEngineCommand(0.0, 0.0);
        steering_commands_allowed_ = true;
        engine_commands_allowed_ = true;
        fprintf(stderr, "SWITCHING TO REVERSE_GO.\n");
        state_ = REVERSE_GO;
      }
      break;



      /* FORWARD_STOP_VEHICLE: bring vehicle to stop before shifting gears */
    case FORWARD_STOP_VEHICLE:
      steering_commands_allowed_ = false;
      engine_commands_allowed_ = false;

      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        state_ = PAUSEFOR_STOP_VEHICLE;
      }
      else {
        sendEngineCommand(0, estop_brake_);
        /* wait until the vehicle stops */
        if(can_velocity_ == 0) {
          fprintf(stderr, "SWITCHING TO FORWARD_PRESHIFT_WAIT\n");
          state_ = FORWARD_PRESHIFT_WAIT;
          shift_time_ = drc::Time::current();
        }
      }
      break;

      /* FORWARD_PRESHIFT_WAIT: wait after stopping before shift */
    case FORWARD_PRESHIFT_WAIT:
      engine_commands_allowed_ = false;
      steering_commands_allowed_ = false;

      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        state_ = PAUSEFOR_STOP_VEHICLE;
      }
      else {
        if(drc::Time::current() - shift_time_ > WAIT_BEFORE_SHIFT) {
          fprintf(stderr, "SWITCHING TO FORWARD_SHIFT_TO_DRIVE\n");
          state_ = FORWARD_SHIFT_TO_DRIVE;
        }
      }
      break;

      /* FORWARD_SHIFT_TO_DRIVE: shift into drive */
    case FORWARD_SHIFT_TO_DRIVE:
      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        state_ = PAUSEFOR_STOP_VEHICLE;
      }

      if(forwardGear(can_status_.target_gear)) {
        sendEngineCommand(0.0, 0.0);
        steering_commands_allowed_ = true;
        engine_commands_allowed_ = true;
        fprintf(stderr, "SWITCHING TO FORWARD_GO.\n");
        state_ = FORWARD_GO;
      }
      else 
        sendGearShiftCommand(GEAR_DRIVE);
      break;

    case FORWARD_GO:
      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        state_ = PAUSEFOR_STOP_VEHICLE;
      }
      else {
        engine_commands_allowed_ = true;
        steering_commands_allowed_ = true;
        if(requested_direction_ != driving_common::Actuator::DIRECTION_FORWARD) {
          fprintf(stderr, "SWITCHING TO REVERSE_SET_BRAKE.\n");
          steering_commands_allowed_ = false;
          state_ = REVERSE_STOP_VEHICLE;
        }
      }
      break;


    case REVERSE_STOP_VEHICLE:
      steering_commands_allowed_ = false;
      engine_commands_allowed_ = false;

      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        state_ = PAUSEREV_STOP_VEHICLE;
      }
      else {
        sendEngineCommand(0, estop_brake_);
        /* wait until the vehicle stops */
        if(can_velocity_ == 0) {
          fprintf(stderr, "SWITCHING TO REVERSE_PRESHIFT_WAIT\n");
          state_ = REVERSE_PRESHIFT_WAIT;
          shift_time_ = drc::Time::current();
        }
      }

      break;

      /* REVERSE_PRESHIFT_WAIT: wait after stopping before shift */
    case REVERSE_PRESHIFT_WAIT:
      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        state_ = PAUSEREV_STOP_VEHICLE;
      }
      else {
        if(drc::Time::current() - shift_time_ > WAIT_BEFORE_SHIFT) {
          fprintf(stderr, "SWITCHING TO REVERSE_SHIFT_TO_REVERSE\n");
          state_ = REVERSE_SHIFT_TO_REVERSE;
        }
      }
      break;

      /* REVERSE_SHIFT_TO_REVERSE: shift into reverse */
    case REVERSE_SHIFT_TO_REVERSE:
      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        state_ = PAUSEREV_STOP_VEHICLE;
      }

      if(reverseGear(can_status_.target_gear)) {
        sendEngineCommand(0.0, 0.0);
        steering_commands_allowed_ = true;
        engine_commands_allowed_ = true;
        fprintf(stderr, "SWITCHING TO REVERSE_GO.\n");
        state_ = REVERSE_GO;
      }
      else 
        sendGearShiftCommand(GEAR_REVERSE);
      break;

    case REVERSE_GO:
      if(!estop_run_status_) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        state_ = PAUSEREV_STOP_VEHICLE;
      }
      else {
        steering_commands_allowed_ = true;
        engine_commands_allowed_ = true;

        if(requested_direction_ != driving_common::Actuator::DIRECTION_REVERSE) {
          fprintf(stderr, "SWITCHING TO FORWARD_STOP_VEHICLE.\n");
          steering_commands_allowed_ = false;
          state_ = FORWARD_STOP_VEHICLE;
        }
      }
      break;





      /* FINISH_STOP_VEHICLE: bring the vehicle to prompt stop */
    case FINISH_STOP_VEHICLE:
      engine_commands_allowed_ = false;
      sendEngineCommand(0.0, estop_brake_);

      /* wait until the vehicle stops */
      if(can_velocity_ == 0) {
        steering_commands_allowed_ = false;
        fprintf(stderr, "SWITCHING TO FINISH_PRESHIFT_WAIT\n");
        state_ = FINISH_PRESHIFT_WAIT;
        shift_time_ = drc::Time::current();
      }    
      break;

      /* FINISH_PRESHIFT_WAIT: wait after stopping to shift */
    case FINISH_PRESHIFT_WAIT:
      if(drc::Time::current() - shift_time_ > WAIT_BEFORE_SHIFT) {
        fprintf(stderr, "SWITCHING TO FINISH_SHIFT_TO_PARK\n");
        state_ = FINISH_SHIFT_TO_PARK;
      }
      break;

      /* FINISH_SHIFT_TO_PARK: shift the vehicle into park */
    case FINISH_SHIFT_TO_PARK:
      if(can_status_.target_gear == driving_common::CanStatus::TARGET_GEAR_PARK_NEUTRAL) {
        fprintf(stderr, "SWITCHING TO FINISH_WAIT.\n");
        state_ = FINISH_WAIT;
      }
      else
        sendGearShiftCommand(GEAR_NEUTRAL);
      break;

      /* FINISH_WAIT: send 0 brake 0 throttle, do nothing */
    case FINISH_WAIT:
      engine_commands_allowed_ = false;
      steering_commands_allowed_ = false;
      sendEngineCommand(0, estop_brake_);
      break;
  }
}

} // namespace vlr
