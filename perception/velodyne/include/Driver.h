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


#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdint.h>
#include <velodyne/Packet.h>

#define VELO_BEAMS_IN_SCAN       32
#define VELO_NUM_LASERS          64
#define VELO_SCANS_IN_PACKET     12
#define VELO_PACKET_SIZE         1206
#define VLF_START_BYTE           0x1b
#define MAXRECVLEN               4096

namespace velodyne
{

  class Driver : public nodelet::Nodelet
  {
    public:
      Driver();
      ~Driver();

    private:
      virtual void onInit();

      void udpthread();

      int openSocket();
      PacketPtr parsePacket(uint8_t* data, uint16_t len);

      boost::shared_ptr<boost::thread> udpThread_;
      ros::Publisher packet_pub_;
      double value_;
      bool shutdown_;
      int port_;

  };
} // namespace velodyne
