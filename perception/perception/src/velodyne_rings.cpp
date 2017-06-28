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


#include <velodyne_rings.h>
#include <passat_constants.h>

#define NUM_SAMPLES    1000
#define EPSILON        0.000000001
#define MAX_RANGE      70.0

namespace perception {

float eval_function(float factor, sample_t* samples, int num_samples) {
  int i;
  double v, sum = 0.0;

  for (i = 0; i < num_samples; i++) {
    v = factor * samples[i].x * samples[i].x;
    sum += fabs(v - samples[i].y);
  }

  return (sum / (float) num_samples);
}

float approx_with_square_function(sample_t* samples, int num_samples) {
  int loop;
  double stepsize, best, test_plus, test_minus, fac;

  loop = 1;
  stepsize = 1.0;
  fac = 10.0;

  best = eval_function(fac, samples, num_samples);

  while (loop && stepsize > EPSILON) {
    test_plus = eval_function(fac + stepsize, samples, num_samples);
    test_minus = eval_function(fac - stepsize, samples, num_samples);
    if (test_plus < best) {
      fac += stepsize;
      best = test_plus;
    }
    else if (test_minus < best) {
      best = test_minus;
      fac -= stepsize;
    }
    else {
      stepsize /= 2.0;
    }
  }

  return (fac);

}

int ringCompare(const void *a, const void *b) {
  static velodyne_ring_settings_t v1, v2;
  v1 = *(velodyne_ring_settings_t *) a;
  v2 = *(velodyne_ring_settings_t *) b;
  if (v1.v_angle < v2.v_angle) {
    return (1);
  }
  else if (v1.v_angle == v2.v_angle) {
    return (0);
  }
  else {
    return (-1);
  }
}

VelodyneRings::VelodyneRings(const velodyne::Config& config, double velodyne_min_beam_diff) :
  min_h_offset(0), velodyne_min_beam_diff_(velodyne_min_beam_diff) {
  printf("velodyne_min_beam_diff: %f\n", velodyne_min_beam_diff);
  for (int i = 0; i < NUM_LASER_BEAMS; i++) {
    ring[i].idx = i;
    ring[i].pb = i;
    ring[i].v_angle = config.vert_angle[i];
    ring[i].h_angle = config.rot_angle[i];
    ring[i].h_offset = (int) (VELO_NUM_TICKS * (-ring[i].h_angle / (2 * M_PI)));
    ring[i].fac = 0;

    min_h_offset = std::min(min_h_offset, ring[i].h_offset);
  }

  qsort(ring, NUM_LASER_BEAMS, sizeof(velodyne_ring_settings_t), ringCompare);

  sample_t sample[NUM_SAMPLES];

  /* beam 0 is highest laser - beam 63 is lowest laser */
  for (int i = 0; i < NUM_LASER_BEAMS; i++) {
    ridx[ring[i].idx] = i;
    /* find partner beam */
    ring[i].pb = i;
    float dist = 0.0;
    float a1, a2;
    a2 = a1 = M_PI_2 + ring[i].v_angle;
    while (ring[i].pb > 0 && dist < velodyne_min_beam_diff_) {
      ring[i].pb--;
      a2 = M_PI_2 + ring[ring[i].pb].v_angle;
      if (a2 < 0.0) {
        dist = velodyne_min_beam_diff_;
      }
      else {
        dist = (DGC_PASSAT_HEIGHT + 0.8) * (tan(a2) - tan(a1));
      }
    }

    /* compute samples for distance difference function
     between the two beams */
    if (ring[i].pb != i) {
      float a_dist = ring[i].v_angle - ring[ring[i].pb].v_angle;
      static float h = DGC_PASSAT_HEIGHT + 0.7;
      for (int j = 0; j < NUM_SAMPLES; j++) {
        float d = j * MAX_RANGE / (float) NUM_SAMPLES;
        sample[j].x = d;
        sample[j].y = (d - h) / sin(asin(h / d) + a_dist);
        if (isnan(sample[j].y)) sample[j].y = 0;
      }

      /* compute best factor of square function to approximate the
       distance difference function */
      ring[i].fac = approx_with_square_function(sample, NUM_SAMPLES);
    }
  }
}

VelodyneRings::~VelodyneRings() {

}

int VelodyneRings::beamToIndex(int beam) {
  return ring[beam].idx;
}

int VelodyneRings::indexToBeam(int index) {
  return ridx[index];
}

int VelodyneRings::nextBeam(int beam, int step) {
  return ring[beam + step].idx;
}

int VelodyneRings::prevBeam(int beam, int step) {
  return ring[beam - step].idx;
}

int VelodyneRings::partnerIndex(int beam) {
  return ring[ring[beam].pb].idx;
}

int VelodyneRings::partnerBeam(int beam) {
  return ring[beam].pb;
}

int VelodyneRings::minHorizontalOffset() {
  return min_h_offset;
}

//int VelodyneRings::horizontalOffset(int beam) {
//  return ring[beam].h_offset;
//}

float VelodyneRings::factor(int beam) {
  return ring[beam].fac;
}

int VelodyneRings::horizontalOffset(int index) {
  int beam = indexToBeam(index);
  return ring[beam].h_offset;
}

}
