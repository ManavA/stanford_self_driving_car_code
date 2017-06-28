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


#ifndef _MDFTOKENS_H_
#define _MDFTOKENS_H_

namespace vlr {

#define MDF_MISSION_NAME "MDF_name"
#define MDF_MISSION_FORMAT_VERSION "format_version"
#define MDF_MISSION_RNDF_REF "RNDF"
#define MDF_MISSION_CREATION_DATE "creation_date"

#define MDF_CHECKPOINTLIST_BEGIN "checkpoints"
#define MDF_CHECKPOINTLIST_NUM_CHECKPOINTS "num_checkpoints"
#define MDF_CHECKPOINTLIST_END "end_checkpoints"

#define MDF_SPEEDLIMITLIST_BEGIN "speed_limits"
#define MDF_SPEEDLIMITLIST_NUM_SPEEDLIMITS "num_speed_limits"
#define MDF_SPEEDLIMITLIST_END "end_speed_limits"

#define MDF_MISSION_END "end_file"

} // namespace vlr

#endif
