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


#ifndef VEHICLE_CONSTANTS_H_
#define VEHICLE_CONSTANTS_H_

namespace vlr {

#define       DGC_PASSAT_LENGTH            4.78
#define       DGC_PASSAT_WIDTH             2.10
#define       DGC_PASSAT_HEIGHT            1.51        // car only
#define       DGC_PASSAT_WHEEL_BASE        2.71
#define       DGC_PASSAT_STEERING_RATIO    14.3        // FROM WEB
#define       DGC_PASSAT_IMU_TO_FA_DIST    3.15        // DON'T KNOW YET
#define       DGC_PASSAT_IMU_TO_CG_DIST    1.37
#define       DGC_PASSAT_IMU_TO_R_BUMPER   1.65    // DISTANCE TO REAR BUMPER

#define       DGC_PASSAT_TRACK_WIDTH       1.65    // DON'T KNOW
#define       DGC_PASSAT_WHEEL_RADIUS      0.40
#define       DGC_PASSAT_FA_TO_BUMPER_DIST 1.02    // DON'T KNOW

#define       DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT    1.7

} // namespace vlr

#endif
