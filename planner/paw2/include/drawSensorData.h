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


#ifndef PAW2_DRAW_SENSOR_DATA_H_
#define PAW2_DRAW_SENSOR_DATA_H_
#include <ros/ros.h>
#include <driving_common/CanStatus.h>
#include <applanix/ApplanixPose.h>
#include <applanix/ApplanixRMS.h>
#include <applanix/ApplanixDMI.h>
#include <applanix/ApplanixGPS.h>
#include <controller/ControllerTarget.h>
#include <paw2_gui.h>
#include <paw2GlView.h>

namespace vlr {
class Paw2Subscriber {
public:
  Paw2Subscriber(ros::NodeHandle& nh, Paw2Gui& gui) : nh_(nh), subscribed_(false), gui_(gui) {}
  virtual ~Paw2Subscriber() {unsubscribe();}
  void unsubscribe();
  boost::mutex& mutex() {return mutex_;}

protected:
  template <class HC>
  void subscribe(const std::string& topic, uint32_t queue_size=5) {
    if(!subscribed_) {
      HC* handler_class = dynamic_cast<HC*>(this);
      sub_ = nh_.subscribe(topic, queue_size, &HC::handler, handler_class);
      subscribed_=true;
    }
  }
protected:
  ros::NodeHandle& nh_;
  ros::Subscriber sub_;
  bool subscribed_;
  boost::mutex mutex_;
  Paw2Gui& gui_;
};

class Paw2Can : public Paw2Subscriber {
 public:
  Paw2Can(ros::NodeHandle& nh, Paw2Gui& gui) : Paw2Subscriber(nh, gui) {}
  virtual ~Paw2Can() {}

  void handler(const driving_common::CanStatus& status);
  void subscribe() {Paw2Subscriber::subscribe<Paw2Can>(std::string("CanStatus"));}

private:
  driving_common::CanStatus status_;
};


class Paw2ApplanixPose : public Paw2Subscriber {
 public:
  Paw2ApplanixPose(ros::NodeHandle& nh, Paw2Gui& gui) : Paw2Subscriber(nh, gui), applanix_hz_(0) {}
  virtual ~Paw2ApplanixPose() {}

  void handler(const applanix::ApplanixPose& status);
  void subscribe() {Paw2Subscriber::subscribe<Paw2ApplanixPose>(std::string("ApplanixPose"));}

private:
  applanix::ApplanixPose pose_;
  double applanix_hz_;
};

class Paw2ApplanixRMS : public Paw2Subscriber {
 public:
  Paw2ApplanixRMS(ros::NodeHandle& nh, Paw2Gui& gui) : Paw2Subscriber(nh, gui) {}
  virtual ~Paw2ApplanixRMS() {}

  void handler(const applanix::ApplanixRMS& rms);
  void subscribe() {Paw2Subscriber::subscribe<Paw2ApplanixRMS>(std::string("ApplanixRMS"));}

private:
  applanix::ApplanixRMS rms_;
};

class Paw2ApplanixDMI : public Paw2Subscriber {
 public:
  Paw2ApplanixDMI(ros::NodeHandle& nh, Paw2Gui& gui) : Paw2Subscriber(nh, gui) {}
  virtual ~Paw2ApplanixDMI() {}

  void handler(const applanix::ApplanixDMI& dmi);
  void subscribe() {Paw2Subscriber::subscribe<Paw2ApplanixDMI>(std::string("ApplanixDMI"));}

private:
  applanix::ApplanixDMI dmi_;
};

class Paw2ApplanixGPS : public Paw2Subscriber {
 public:
  Paw2ApplanixGPS(ros::NodeHandle& nh, Paw2Gui& gui) : Paw2Subscriber(nh, gui) {
    gams_solution_ = {
        "no solution",
        "solution from nav. & install",
        "solution without install data",
        "degraded floated ambiguity",
        "floated ambiguity",
        "degraded fixed integer",
        "fixed integer test install data",
        "fixed integer",
    };
  }
  virtual ~Paw2ApplanixGPS() {}

  void handler(const applanix::ApplanixGPS& gps);
  void subscribe() {Paw2Subscriber::subscribe<Paw2ApplanixGPS>(std::string("ApplanixGPS"));}

private:
  applanix::ApplanixGPS gps_;
  static std::vector<std::string> gams_solution_;
};

} // namespace vlr

#endif // PAW2_DRAW_SENSOR_DATA_H_

