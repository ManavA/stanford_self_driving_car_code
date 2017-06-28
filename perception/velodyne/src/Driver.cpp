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


#include <pluginlib/class_list_macros.h>
#include <Driver.h>
#include <boost/thread.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <global.h>

namespace velodyne
{

  PLUGINLIB_DECLARE_CLASS(velodyne, Driver, velodyne::Driver, nodelet::Nodelet);

  Driver::Driver() :
    shutdown_(false)
  {

  }

  Driver::~Driver()
  {
    shutdown_ = true;
    udpThread_->join();
  }

  void Driver::onInit()
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    private_nh.getParam("/driving/velodyne/port", port_);
    packet_pub_ = private_nh.advertise<velodyne::Packet>("/driving/velodyne/packet", 10);
    udpThread_ = boost::shared_ptr<boost::thread> (
        new boost::thread(boost::bind(&Driver::udpthread, this)));
  }

  void Driver::udpthread()
  {
    unsigned char data[MAXRECVLEN];

    int sock = openSocket();
    NODELET_INFO_STREAM("Opened UDP socket " << port_ << " to velodyne");
    //double last_spin_publish_time = ros::Time::now().toSec();
    //uint64_t last_spin_num = 0;
    //uint64_t last_byte_count = 0;
    //uint64_t bytes = 0;

    PacketPtr packet;
    while (!shutdown_) {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(sock, &set);
      struct timeval t;
      t.tv_sec = 0;
      t.tv_usec = 100000;
      int err = select(sock + 1, &set, NULL, NULL, &t);

      if (err == 1) {
        int len = recv(sock, data, MAXRECVLEN, MSG_WAITALL);
        if (len < 0) {
          NODELET_ERROR("recv from socket failed with return %d", len);
          throw VLRException("recvfrom() failed");
        }
        ros::Time packet_time = ros::Time::now();
        try 
        {
          packet = parsePacket(data, len);
          packet->header.stamp = packet_time;
          packet_pub_.publish(packet);
        }
        catch(vlr::Ex<>)
        {
          // Nothing to do, if we get a bad packet keep trying
          // Errors have already been published over ROS
        }
        //bytes += len;
      }
      /*if(num_spins_ > last_spin_num ) {
        double  time = ros::Time::now().toSec();
        stat_.turn_rate = (time != last_spin_publish_time ? 1/(time-last_spin_publish_time) : 0);
        stat_.byte_rate = bytes - last_byte_count;
        stat_pub_.publish(stat_);
        last_spin_num = packet->spin_count;
        last_spin_publish_time = time;
        last_byte_count = bytes;
      }*/
    }

    close(sock);
    NODELET_INFO_STREAM("Closed UDP socket " << port_ << " to velodyne");
  }

  /**
   * Reads a UDP Packet from Velodyne and puts it in a ROS message format
   */
  PacketPtr Driver::parsePacket(uint8_t* data, uint16_t len)
  {
    PacketPtr p(new Packet);
    int ptr = 0;
    if (len != VELO_PACKET_SIZE) {
      NODELET_ERROR("Bad velodyne packet size.  Possibly because multiple packet have arrived");
      throw VLRException("");
    }

    for (int b = 0; b < velodyne::Packet::NUM_BLOCKS; b++)
    {
      uint16_t blockid;
      memcpy(&blockid, &(data[ptr]), sizeof(uint16_t));
      ptr += sizeof(uint16_t);
      switch (blockid) 
      {
        case 0xeeff: // upper block
          p->block[b].block = 0;
          break;
        case 0xddff: // lower block
          p->block[b].block = 1;
          break;
        default:
          NODELET_ERROR("Unknown block id in velodyne packet.  Possibly misaligned data.");
          throw VLRException("");
      }

      memcpy(&(p->block[b].encoder), &(data[ptr]), sizeof(uint16_t));
      ptr += sizeof(uint16_t);

      for (int l = 0; l < velodyne::BlockRaw::NUM_BEAMS; l++)
      {
        memcpy(&(p->block[b].laser[l].distance), 
            &(data[ptr]), sizeof(uint16_t));
        ptr += sizeof(uint16_t);
        memcpy(&(p->block[b].laser[l].intensity), 
            &(data[ptr]), sizeof(uint8_t));
        ptr += sizeof(uint8_t);
      }
    }
    memcpy(&(p->spin_count), &(data[ptr]), sizeof(uint16_t));

    return p;
  }

  /**
   * Opens UDP socket to get data from Velodyne
   */
  int Driver::openSocket()
  {
    int sock_rmem_size = 2097152;
    int sock;
    struct sockaddr_in broadcastAddr; /* Broadcast Address */

    /* Create a best-effort datagram socket using UDP */
    if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
      NODELET_ERROR("Failed to open socket"); 
      throw VLRException("socket() failed");
    }

    /* Request a larger receive buffer window */
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &sock_rmem_size, sizeof(sock_rmem_size)) == -1) {
      NODELET_WARN_STREAM("Could not increase socket receive buffer to " << sock_rmem_size << ". This may cause dropped veloydne data\n");
    }

    /* Zero out structure */
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));

    /* Internet address family */
    broadcastAddr.sin_family = AF_INET;

    /* Any incoming interface */
    broadcastAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    /* Broadcast port */
    broadcastAddr.sin_port = htons(port_);

    /* Bind to the broadcast port */
    if (bind(sock, (struct sockaddr *) &broadcastAddr, sizeof(broadcastAddr)) < 0) {
      NODELET_ERROR("Could not bind to port %d", port_);
      throw VLRException("bind() failed");
    }

    return sock;
  }
}

