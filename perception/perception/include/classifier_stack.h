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


#include <assert.h>
#include <lltransform.h>
#include <aw_roadNetworkSearch.h>

#include "perception.h"
#include "utils.h"
#include "obstacle.h"
#include "segment.h"
#include "laser_segment.h"
#include "box.h"
//REMOVE: #include "kalman_tracker.h"
#include "kalman_multitracker.h"


#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <queue>

using namespace std;
using namespace std::tr1;
using namespace dgc;

using namespace vlr::rndf;

using namespace Eigen;

namespace perception {

//TODO: turn into param?
//#define MIN_CONFIDENCE_CLUSTER      1.0

//extern dgc::VelodyneRings* rings;
//REMOVE: TrackerKF* tracker_ = NULL;
//KalmanMultiTracker* tracker_ = NULL;

/*
char* obstacle_type_str[] =
{
    "unknown", "car", "pedestrian", "bicyclist"
};

char* obstacle_type2str(dgc_obstacle_type type) {
  if (type <= OBSTACLE_BICYCLIST )
    return obstacle_type_str[type];
  else
    return obstacle_type_str[0];
}

*/

class ClassifierStack {
public:
  ClassifierStack(){}

  void push_inbox(boost::shared_ptr<Obstacle> obstacle) {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    inbox_.push_back(obstacle);
  }
  boost::shared_ptr<Obstacle> pop_inbox() {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    boost::shared_ptr<Obstacle> obstacle = inbox_.back();
    inbox_.pop_back();
    return obstacle;
  }
  int inbox_empty() {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    return inbox_.empty();
  }
  int clear_inbox() {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    int s = inbox_.size();
    inbox_.clear();
    return s;
  }

  void push_outbox(boost::shared_ptr<Obstacle> obstacle) {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    outbox_.push_back(obstacle);
  }
  boost::shared_ptr<Obstacle> pop_outbox() {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    boost::shared_ptr<Obstacle> obstacle = outbox_.back();
    outbox_.pop_back();
    return obstacle;
  }
  int outbox_empty() {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    return outbox_.empty();
  }

private:
  boost::mutex inbox_mutex_;
  boost::mutex outbox_mutex_;

  vector< boost::shared_ptr<Obstacle> > inbox_;
  vector< boost::shared_ptr<Obstacle> > outbox_;
};


} // namespace perception

