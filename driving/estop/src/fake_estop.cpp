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


#include <ros/ros.h>
#include <global.h>

#include <driving_common/EStopRequest.h>
#include <driving_common/EStopStatus.h>
#include <driving_common/Heartbeat.h>
//#include <power_interface.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <termios.h>

namespace drc = driving_common;

namespace vlr {

ros::NodeHandle* nh_ = NULL;
ros::Subscriber estop_request_sub_;
ros::Publisher estop_status_pub_, heartbeat_pub_;

driving_common::EStopStatus estop_status_msg_;
driving_common::Heartbeat heartbeat_msg_;

bool enable_siren_=false;
bool start_run_=false;

struct termios old_term_;
int old_flags_;

void initializeKeyboard() {
  struct termios term_struct;
  int flags;
  tcflag_t oflags;

  flags = fcntl((intptr_t)stdin, F_GETFL);
  old_flags_ = flags;
  fcntl((intptr_t)stdin, F_SETFL, flags | O_NONBLOCK);
  tcgetattr(0, &term_struct);
  memcpy(&old_term_, &term_struct, sizeof(struct termios));
  oflags = term_struct.c_oflag;
  cfmakeraw(&term_struct);
  term_struct.c_oflag = oflags;
  term_struct.c_lflag |= ISIG;
  tcsetattr(0, TCSANOW, &term_struct);
}

void releaseKeyboard() {
  fcntl((intptr_t)stdin, F_SETFL, old_flags_);
  tcsetattr(0, TCSANOW, &old_term_);
}

int readKeyboardCharacter(char *c)
{
  int32_t available;
  int i;

  ioctl(0, FIONREAD, &available);
  if(available > 0) {
    for(i = 0; i < available; i++)
      if(read(0, c, 1) != 1)
        throw VLRException("Trouble reading from keyboard");
    return 1;
  }
  else
    return 0;
}

std::string modeString(uint8_t mode) {
  if(mode == driving_common::EStopStatus::ESTOP_PAUSE) {
    return "PAUSE";
  }
  else if(mode == driving_common::EStopStatus::ESTOP_DISABLE) {
    return "DISABLE";
  }
  else if(mode == driving_common::EStopStatus::ESTOP_RUN) {
    return "RUN";
  }
  return "ERROR";
}

void estopRequestHandler(const driving_common::EStopRequest& estop_request) {
  if(estop_request.estop_code == driving_common::EStopStatus::ESTOP_PAUSE) {
    //    PowerSetNamedCommand(ipc, "LIGHT", 0);
    //    PowerSetNamedCommand(ipc, "SIREN", 0);
  }
  else if(estop_request.estop_code == driving_common::EStopStatus::ESTOP_DISABLE) {
    //    PowerSetNamedCommand(ipc, "LIGHT", 0);
    //    PowerSetNamedCommand(ipc, "SIREN", 0);
  }
  else if(estop_request.estop_code == driving_common::EStopStatus::ESTOP_RUN) {
    //    PowerSetNamedCommand(ipc, "LIGHT", 1);
    if(enable_siren_) {
      //      PowerSetNamedCommand(ipc, "SIREN", 1);
    }
  }

  estop_status_msg_.timestamp = drc::Time::current();
  estop_status_msg_.estop_code = estop_request.estop_code;
  estop_status_pub_.publish(estop_status_msg_);

  fprintf(stderr, "\rMODE:   %s    ", vlr::modeString(estop_status_msg_.estop_code).c_str());
}

template <class T> void getParam(std::string key, T& var) {
  if(!nh_->getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void readParameters() {
  getParam("estop/enable_siren", enable_siren_);
  getParam("estop/start_in_run_mode", start_run_);
}

} // namespace vlr

int main(int argc, char **argv) {

  ros::init(argc, argv, "fake_estop");
  vlr::nh_ = new ros::NodeHandle("/driving");

  double last_publish_time = 0;
  double last_heartbeat_time = 0;
  vlr::heartbeat_msg_.modulename = "EStop";

  vlr::estop_request_sub_ = vlr::nh_->subscribe("EStopRequest", 5, &vlr::estopRequestHandler);
  vlr::estop_status_pub_ = vlr::nh_->advertise<driving_common::EStopStatus>("EStopStatus", 1);
  vlr::heartbeat_pub_ = vlr::nh_->advertise<driving_common::Heartbeat>("Heartbeat", 1);

  try {
    vlr::readParameters();
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
    exit(-5);
  }

  bool interactive = false;

  if(argc > 1) {
    for(int i = 1; i < argc; i++) {
      if(strcmp(argv[i],"-i") == 0)
        interactive = true;
    }
  }
  if(!interactive) {
    std::cout << "Non-interactive mode.  Use -i to enable keyboard input\n";
  }
  if(vlr::start_run_) {
    vlr::estop_status_msg_.estop_code = driving_common::EStopStatus::ESTOP_RUN;
  }
  else {
    vlr::estop_status_msg_.estop_code = driving_common::EStopStatus::ESTOP_PAUSE;
  }

  fprintf(stderr, "RUN MODE %d\n", vlr::start_run_);

  if(interactive) {
    vlr::initializeKeyboard();
    fprintf(stderr, "\rMODE:   %s    ", vlr::modeString(vlr::estop_status_msg_.estop_code).c_str());
  }

  //  PowerSetNamedCommand(ipc, "LIGHT", 1);
  //  PowerSetNamedCommand(ipc, "SIREN", 0);

  ros::Rate r(50); // 50 hz ; was 10

  while(ros::ok()) {
    bool read_one = false;
    char c = 'p';
    if(interactive) {
      while(vlr::readKeyboardCharacter(&c)) {
        read_one = true;
      }
    }

    if(read_one) {
      switch(c) {
        case 'p': case 'P':
          vlr::estop_status_msg_.estop_code = driving_common::EStopStatus::ESTOP_PAUSE;
          //	PowerSetNamedCommand(ipc, "LIGHT", 1);
          //	PowerSetNamedCommand(ipc, "SIREN", 0);
          break;
        case 'k': case 'K': case 'd': case 'D':
          vlr::estop_status_msg_.estop_code = driving_common::EStopStatus::ESTOP_DISABLE;
          //	PowerSetNamedCommand(ipc, "LIGHT", 0);
          //	PowerSetNamedCommand(ipc, "SIREN", 0);
          break;
        case 'r': case 'R':
          vlr::estop_status_msg_.estop_code = driving_common::EStopStatus::ESTOP_RUN;
          // 	PowerSetNamedCommand(ipc, "LIGHT", 1);
          if(vlr::enable_siren_) {
            //	  PowerSetNamedCommand(ipc, "SIREN", 1);
          }
          break;
      }
      vlr::estop_status_msg_.timestamp = drc::Time::current();
      vlr::estop_status_pub_.publish(vlr::estop_status_msg_);
      fprintf(stderr, "\rMODE:   %s    ", vlr::modeString(vlr::estop_status_msg_.estop_code).c_str());
    }

    double current_time = drc::Time::current();
    if(current_time - last_publish_time > 1.0) {
      vlr::estop_status_msg_.timestamp = drc::Time::current();
      vlr::estop_status_pub_.publish(vlr::estop_status_msg_);
      fprintf(stderr, "\rMODE:   %s    ", vlr::modeString(vlr::estop_status_msg_.estop_code).c_str());
      last_publish_time = current_time;
    }

    if(current_time - last_heartbeat_time > 1.0) {
      vlr::heartbeat_pub_.publish(vlr::heartbeat_msg_);
      last_heartbeat_time = current_time;
    }

    ros::spinOnce();
    r.sleep();
  }

  fprintf(stderr, "\n");

  if(interactive) {
    vlr::releaseKeyboard();
  }

  delete vlr::nh_;
  return 0;
}
