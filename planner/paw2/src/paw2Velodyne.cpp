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


#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <global.h>
//#include <velodyne_interface.h>
//#include <velodyne_shm_interface.h>
//#include <velocore.h>
#include <velodyne.h>
#include <paw2Velodyne.h>

#define MAX_NUM_SCANS            10000
#define MAX_NUM_POINTS_PER_BEAM  20000
#define NUM_LASER_BEAMS          VELO_NUM_LASERS

using namespace dgc;

namespace drc = driving_common;

namespace vlr {

int                 num_pts[NUM_LASER_BEAMS];
velodyne_pt_t       pts[NUM_LASER_BEAMS][MAX_NUM_POINTS_PER_BEAM];

#define    VELO_BLIND_SPOT_START    17000
#define    VELO_BLIND_SPOT_STOP     19000
#define    VELO_NUM_TICKS           36000
#define    BINS_PER_REV             720

//static short beam_active_[NUM_LASER_BEAMS] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

Paw2Velodyne::Paw2Velodyne(Paw2Gui& gui) : gui_(gui), nh_("/driving"), veloconfig_(NULL), scans_(NULL), //velo_interface_(NULL),
                                  subscribed_(false), color_mode_(intensity), display_style_(points),
                                  point_size_(1), show_upper_block_(true), show_lower_block_(true),
                                  mark_single_beam_(false), marked_beam_(0), update_data_(true), num_scans_(0),
                                  update_thread_(NULL) {

//  dgc::dgc_velodyne_get_config(&veloconfig_);
  veloconfig_ = new velodyne::Config;
//  scans_ = new dgc_velodyne_scan_t[MAX_NUM_SCANS];
//  velo_interface_ = new dgc::VelodyneShmInterface;
}

Paw2Velodyne::~Paw2Velodyne() {
unsubscribe();
//delete[] scans_;
}

void Paw2Velodyne::subscribe() {
  packet_sub_ = nh_.subscribe("projected", 5, &Paw2Velodyne::update, this);
//    if(velo_interface_->CreateClient() < 0) {
//    throw Exception("Could not create client for velodyne interface.");
//  }

subscribed_=true;
//update_thread_ = new boost::thread(boost::bind(&Paw2Velodyne::update, this));
spin_thread_ = new boost::thread(boost::bind(&Paw2Velodyne::spinThread, this));
}

void Paw2Velodyne::unsubscribe() {
  if(!subscribed_) {return;}
  subscribed_=false;
//  if(update_thread_) {
//    update_thread_->join();
//    update_thread_=NULL;
//  }

  if(spin_thread_) {
    spin_thread_->join();
    spin_thread_=NULL;
  }
  // TODO: DeleteClient() doesn't exist :-(
  //  subscribed_=false;
}

//void Paw2Velodyne::update() {
//  while (subscribed_) {
//    if (update_data_) {
//      vlr::Lock lock(mutex_);
//      while (velo_interface_->ScanDataWaiting()) {
//        num_scans_ = velo_interface_->ReadCurrentScans(scans_, MAX_NUM_SCANS);
//      }
//    }
//  usleep(1000); // TODO:...
//  }
//}

void Paw2Velodyne::spinThread() {
  ros::spin();
}

void Paw2Velodyne::update(const velodyne::Projected& scan) {
  if(subscribed_) {
    if(update_data_) {
      if(!scans_.empty()) {
//        const velodyne::Projected& last_scan = *(scans_.rbegin());
//        if(scan.encoder < last_scan.encoder) {scans_.clear();}
        scans_.push_back(scan);
      }
      else {
        scans_.push_back(scan);
      }
    }
  }
}

void Paw2Velodyne::draw(const drc::GlobalPose& cur_pose) {
}

float Paw2Velodyne::pointDist(velodyne_pt_t& pt1, velodyne_pt_t& pt2) {
  float dx = (pt1.x - pt2.x);
  float dy = (pt1.y - pt2.y);
  float dz = (pt1.z - pt2.z);
  return sqrt(dx*dx+dy*dy+dz*dz);
}

RGB Paw2Velodyne::hsv2Rgb(HSV color) {
  RGB ret;
  int i;
  double aa, bb, cc, f;

  if (color.s == 0) ret.r = ret.g = ret.b = color.v;
  else {
    if (color.h == 1.0) color.h = 0;
    color.h *= 6.0;
    i = (int) floor(color.h);
    f = color.h - i;
    aa = color.v * (1 - color.s);
    bb = color.v * (1 - (color.s * f));
    cc = color.v * (1 - (color.s * (1 - f)));
    switch (i) {
    case 0:
      ret.r = color.v;
      ret.g = cc;
      ret.b = aa;
      break;
    case 1:
      ret.r = bb;
      ret.g = color.v;
      ret.b = aa;
      break;
    case 2:
      ret.r = aa;
      ret.g = color.v;
      ret.b = cc;
      break;
    case 3:
      ret.r = aa;
      ret.g = bb;
      ret.b = color.v;
      break;
    case 4:
      ret.r = cc;
      ret.g = aa;
      ret.b = color.v;
      break;
    case 5:
      ret.r = color.v;
      ret.g = aa;
      ret.b = bb;
      break;
    }
  }
  return (ret);
}

RGB Paw2Velodyne::valToRgb(double val) {
  HSV color = {1.0, 1.0, 1.0};

  /* cut to range [0.0,1.0] */
  val = std::min( 1.0, std::max( 0.0, val ) );

  /* the gradient is done by changing hue between blue and yellow */
  if (val>0.1) {
    color.h = fmod(0.555*val+.66666666,1.0);
  } else {
    /* if the val is smaller than 10% */
    color.h = .66666666;
    color.s = 1.0;
    color.v = val * 10.0;
  }

  return( hsv2Rgb( color ) );
}

} // namespace vlr
