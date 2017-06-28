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


#ifndef VELODYNE_INTERFACE_H
#define VELODYNE_INTERFACE_H

#include <stdint.h>
#include <global.h>
#include <transform.h>

namespace vlr {

typedef enum { UNKNOWN, PCAP, VLF } VELODYNE_FILE_TYPE;

#define VELODYNE_TICKS_TO_METER     0.002

#define VELO_NUM_LASERS          64
#define VELO_NUM_TICKS           36000
#define VELO_SPIN_START          18000

#define VELO_SCANS_IN_PACKET     12
#define VELO_BEAMS_IN_SCAN       32

#define VELO_PACKET_SIZE         1206

#define VLF_START_BYTE           0x1b
#define VELO_PACKET_FSIZE        1226         /* 1 + 16 + 2 + 1206 + 1 */

/* 
   one packet is 

   one scan =  VELO_BEAMS_IN_SCAN * 3 + 4 [enc+block+n*3] = 100 bytes
   packet = VELO_SCANS_IN_PACKET * 100 + status (6 bytes) = 1206 bytes
   
   file:
   
   startbyte  timestamp length  [... DATA ...] checksum
   
   => additional 1+16+2+1 = 20 bytes
   
     startbyte = 0x1b
     timestamp = 2 x <unsigned long>  [tv_sec and tv_usec]
     length    = <unsigned short>
     ...
     checksum  = <unsigned char>
*/

/************************************************************************
 *
 ************************************************************************/


/************************************************************************
 *
 *  STRUCTURE WITH RAW DATA FROM SCANNER
 *
 ************************************************************************/

typedef struct {

  unsigned short                encoder;
  unsigned short                block;
  unsigned short                range[VELO_BEAMS_IN_SCAN];
  unsigned char                 intensity[VELO_BEAMS_IN_SCAN];

} dgc_velodyne_measurement_t, *dgc_velodyne_measurement_p;

typedef struct {

  double                        timestamp;
  dgc_velodyne_measurement_t    scan[VELO_SCANS_IN_PACKET];
  unsigned char                 status[6];

} dgc_velodyne_packet_t, *dgc_velodyne_packet_p;


/************************************************************************
 *
 *  STRUCTURE WITH LOG FILE DATA
 *
 ************************************************************************/

/*typedef struct {

  VELODYNE_FILE_TYPE           format;

  dgc_FILE                    *fp;
  char                        *filename;

  int                          buffer_len;
  unsigned char               *msg_buffer;

  // variables that change infrequently
  int                           sweep_number;

} dgc_velodyne_file_t, * dgc_velodyne_file_p;*/

/************************************************************************
 *
 ************************************************************************/

/*typedef struct {

  // variables that never change 
  dgc_transform_t           offset;
  double                    range_offset[VELO_NUM_LASERS];
  double                    range_offsetX[VELO_NUM_LASERS];
  double                    range_offsetY[VELO_NUM_LASERS];
  char                      laser_enabled[VELO_NUM_LASERS];
  double                    global_range_offset;
  double                    vert_angle[VELO_NUM_LASERS];
  double                    rot_angle[VELO_NUM_LASERS];
  double                    h_offset[VELO_NUM_LASERS];
  double                    v_offset[VELO_NUM_LASERS];
  double                    sin_vert_angle[VELO_NUM_LASERS];
  double                    cos_vert_angle[VELO_NUM_LASERS];
  double                    enc_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    sin_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    cos_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    sin_enc_angle[VELO_NUM_TICKS];
  double                    cos_enc_angle[VELO_NUM_TICKS];
  double                    enc_angle[VELO_NUM_TICKS];
  double                    range_multiplier;
  int                       min_intensity;
  int                       max_intensity;
  double                    intensity_map[VELO_NUM_LASERS][256];
  int                       beam_order[VELO_NUM_LASERS];
  int                       inv_beam_order[VELO_NUM_LASERS];
  int                       spin_start;

} dgc_velodyne_config_t, *dgc_velodyne_config_p;*/

/************************************************************************
 *
 ************************************************************************/

typedef struct {

  /* the unit here is 5mm(!). Do x * 0.005 to convert
     to m. Therefore the maximum range is +/- 163.84m  */
  short                     x;
  short                     y;
  short                     z;
  /* use the units here are 5mm (1). Do x * 0.005 to convert to m */
  unsigned short            range;
  unsigned char             intensity;

} dgc_velodyne_point_t, * dgc_velodyne_point_p;

typedef struct {

  // global position of the measurement
  dgc_velodyne_point_t      p[VELO_BEAMS_IN_SCAN];
  //
  double                    timestamp;
  //
  dgc::dgc_pose_t           robot;
  // which block (upper/lower lasers) is firing
  unsigned char             block;
  unsigned short            encoder;
  // counter that indicates which scans belongs to the same revolution
  unsigned short            counter;

} dgc_velodyne_scan_t, *dgc_velodyne_scan_p;


class VelodyneInterface {
 public:
  virtual ~VelodyneInterface() {};

  virtual int CreateServer(void) = 0;
  virtual int CreateClient(void) = 0;
  virtual void SetKey(uint32_t) = 0;
  virtual int ReadRaw(unsigned char *data) = 0;
  virtual int WriteRaw(int len, unsigned char *data) = 0;
  virtual int ReadScans(dgc_velodyne_scan_p scans, int max_num_scans) = 0;
  virtual int WriteScans(int num, dgc_velodyne_scan_p scans) = 0;
  virtual int ReadCurrentScans(dgc_velodyne_scan_p scans, 
			       int max_num_scans) = 0;

  virtual int RawDataWaiting(void) = 0;
  virtual int ScanDataWaiting(void) = 0;
};

}

#endif
