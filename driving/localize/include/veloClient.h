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


#ifndef VLR_VELOCLIENT_H_
#define VLR_VELOCLIENT_H_

#include <string>
#include <global.h>
#include <transform.h>
#include <velodyne.h>
#include <velo_support.h>

namespace vlr {

class VelodyneClient {
 public:
  VelodyneClient(const std::string& cal_filename, const std::string& int_filename, dgc::dgc_transform_t velodyne_offset);
  ~VelodyneClient();

  bool blocksAvailable() {
    return !blocks_.empty();
  }
  const velodyne::Config& config() {return config_;}
  void readSpin(velodyne::Spin* spin);

private:
  void update(const velodyne::Projected& packet);
  void spinThread();

private:
  velodyne::Config config_;
  ros::NodeHandle nh_;
  ros::Subscriber velo_packet_sub_;
  std::vector<velodyne::Block> blocks_;
  velodyne::Projected last_packet_;
  boost::thread* spin_thread_;
  boost::mutex mutex_;
  boost::condition_variable cond_spin_ready_;
  boost::condition_variable cond_spin_read_;
  bool spin_ready_;
  bool spin_read_;
};

} // namespace vlr

#endif
