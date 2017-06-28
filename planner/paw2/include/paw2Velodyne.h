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


#ifndef PAW2_VELODYNE_H_
#define PAW2_VELODYNE_H_
#include <boost/thread.hpp>
#include <GlobalPose.h>
#include <velodyne.h>
#include <velodyne/Projected.h>
//#include <velodyne_interface.h>

#include <graphics.h>
#include <paw2_gui.h>

namespace vlr {

typedef struct {
  float   x, y, z;
  char    i, v;
} velodyne_pt_t;

class Paw2Velodyne {
public:
  typedef enum {white=0, intensity, color1, color2} ColorMode_t;
  typedef enum {points=0, lines} DisplayStyle_t;

public:
  Paw2Velodyne(Paw2Gui& gui);
  virtual ~Paw2Velodyne();
  void subscribe();
  void unsubscribe();
  boost::mutex& mutex() {return mutex_;}
  void draw(const driving_common::GlobalPose& cur_pose);
  inline bool updateData() const {return update_data_;}
  inline void updateData(bool update) {update_data_=update;}
  inline ColorMode_t colorMode() const {return color_mode_;}
  inline void colorMode(ColorMode_t mode) {color_mode_=mode;}
  inline DisplayStyle_t displayStyle() const {return display_style_;}
  inline void displayStyle(DisplayStyle_t style) {display_style_=style;}
  inline uint32_t pointSize() const {return point_size_;}
  inline void pointSize(uint32_t size) {point_size_=size;}
  inline bool showUpperBlock() const {return show_upper_block_;}
  inline void showUpperBlock(bool show) {show_upper_block_=show;}
  inline bool showLowerBlock() const {return show_lower_block_;}
  inline void showLowerBlock(bool show) {show_lower_block_=show;}

private:
  void update(const velodyne::Projected& scan);
  void spinThread();

//  void update();
  RGB valToRgb(double val);
  RGB hsv2Rgb(HSV color);
  float pointDist(velodyne_pt_t& pt1, velodyne_pt_t& pt2);

private:
  Paw2Gui& gui_;
  ros::NodeHandle nh_;
  ros::Subscriber packet_sub_;
  boost::mutex mutex_;
  velodyne::Config* veloconfig_;
  std::vector<velodyne::Projected> scans_;
//  dgc::VelodyneInterface* velo_interface_;
  bool subscribed_;
  ColorMode_t color_mode_;
  DisplayStyle_t display_style_;
  uint32_t point_size_;
  bool show_upper_block_;
  bool show_lower_block_;
  bool mark_single_beam_;
  int32_t marked_beam_;

  bool update_data_;

  int32_t num_scans_;
  boost::thread* spin_thread_;
  boost::thread* update_thread_;
};

} // namespace vlr

#endif // PAW2_VELODYNE_H_

