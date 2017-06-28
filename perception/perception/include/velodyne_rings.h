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


#ifndef VELODYNERINGS_H_
#define VELODYNERINGS_H_

#include <perception_defines.h>
#include <perception_types.h>

namespace perception {

/*
 * Beam and index have specific meanings:
 *   beam: index into beams sorted by vertical angle of beams, 0 maps to nearest beam
 *   index: index into beams as sorted by incoming data
 */

typedef struct {
  int     idx;      /* velodyne index beam */
  int     pb;       /* partner beam for the comparison */
  float   v_angle;  /* vertical angle of the laser beam */
  float   h_angle;  /* horizontal angle of the laser beam */
  int     h_offset; /* horizontal offset of the beam in velodyne ticks */
  float   fac;      /* approximation factor of function with pb */
} velodyne_ring_settings_t;


typedef struct {
  float  x;
  float  y;
} sample_t;

class VelodyneRings {
private:
  velodyne_ring_settings_t    ring[NUM_LASER_BEAMS];
  int                         ridx[NUM_LASER_BEAMS];

  int     min_h_offset;

  double  velodyne_min_beam_diff_;

public:
  VelodyneRings(const velodyne::Config& config, double velodyne_min_beam_diff);
  virtual ~VelodyneRings();

  int beamToIndex(int beam); // go from index to sorted index
  int indexToBeam(int index);  // go from sorted index to index

//  int beam(int index);
//  int index(int idx);
  int nextBeam(int index, int step=1); // returns the index of the next farther ring
  int prevBeam(int index, int step=1); // returns the index of the next closer beam

  int partnerBeam(int index);
  int partnerIndex(int index);

  int minHorizontalOffset();

  int horizontalOffset(int index);
  float factor(int beam);
};

} // namespace perception

#endif // VELODYNERINGS_H_
