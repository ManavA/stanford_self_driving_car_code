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


#ifndef DGC_DEFAULTCAL_H
#define DGC_DEFAULTCAL_H

namespace velodyne {

double DEFAULT_VELO_ROT_ANGLE[64]  = {   // RCF
  // UPPER LASER
  -4.95423980,               // laser number  01
  -2.81469990,               // laser number  02
  +2.81473990,               // laser number  03
  +4.95422980,               // laser number  04
  -0.67916203,               // laser number  05
  +1.45547000,               // laser number  06
  -1.45547000,               // laser number  07
  +0.67916203,               // laser number  08
  +3.59211990,               // laser number  09
  +5.73379990,               // laser number  10
  +2.81473990,               // laser number  11
  +4.95419980,               // laser number  12
  -4.95423980,               // laser number  13
  -2.81469990,               // laser number  14
  -5.73379990,               // laser number  15
  -3.59211990,               // laser number  16
  -0.67916203,               // laser number  17
  +1.45547000,               // laser number  18
  -1.45547000,               // laser number  19
  +0.67916203,               // laser number  20
  +3.59211990,               // laser number  21
  +5.73379990,               // laser number  22
  +2.81473990,               // laser number  23
  +4.95423980,               // laser number  24
  -4.95423980,               // laser number  25
  -2.81473990,               // laser number  26
  -5.73379990,               // laser number  27
  -3.59211990,               // laser number  28
  -0.67916203,               // laser number  29
  +1.45547000,               // laser number  30
  -1.45547000,               // laser number  31
  +0.67916203,               // laser number  32

  // LOWER LASER
  -7.44301080,               // laser number  01
  -4.22423320,               // laser number  02
  +4.22423320,               // laser number  03
  +7.44301080,               // laser number  04
  -1.01877260,               // laser number  05
  +2.18349790,               // laser number  06
  -2.18349790,               // laser number  07
  +1.01877300,               // laser number  08
  +5.39261480,               // laser number  09
  +8.61881260,               // laser number  10
  +4.22423320,               // laser number  11
  +7.44301080,               // laser number  12
  -7.44301080,               // laser number  13
  -4.22423320,               // laser number  14
  -8.61881260,               // laser number  15
  -5.39261480,               // laser number  16
  -1.01877300,               // laser number  17
  +2.18349790,               // laser number  18
  -2.18349790,               // laser number  19
  +1.01877300,               // laser number  20
  +5.39261480,               // laser number  21
  +8.61881260,               // laser number  22
  +4.22423320,               // laser number  23
  +7.44301080,               // laser number  24
  -7.44301080,               // laser number  25
  -4.22423320,               // laser number  26
  -8.61881260,               // laser number  27
  -5.39261480,               // laser number  28
  -1.01877300,               // laser number  29
  +2.18349790,               // laser number  30
  -2.18349790,               // laser number  31
  +1.01877300,               // laser number  32
};

double DEFAULT_VELO_VERT_ANGLE[64] = {   // RCF
  // UPPER LASER
  -07.15812020,               // laser number  01 
  -06.81782010,               // laser number  02
  +00.31782201,               // laser number  03
  +00.65811902,               // laser number  04
  -06.47765020,               // laser number  05
  -06.13758990,               // laser number  06
  -08.52081010,               // laser number  07
  -08.17988970,               // laser number  08
  -05.79763980,               // laser number  09
  -05.45776990,               // laser number  10
  -07.83913990,               // laser number  11
  -07.49856000,               // laser number  12
  -03.08021000,               // laser number  13
  -02.74062990,               // laser number  14
  -05.11798000,               // laser number  15
  -04.77826020,               // laser number  16
  -02.40104010,               // laser number  17
  -02.06141000,               // laser number  18
  -04.43859000,               // laser number  19
  -04.09895990,               // laser number  20
  -01.72174000,               // laser number  21
  -01.38202000,               // laser number  22
  -03.75937010,               // laser number  23
  -03.41979000,               // laser number  24
  +00.99855500,               // laser number  25
  +01.33914010,               // laser number  26
  -01.04223000,               // laser number  27
  -00.70236301,               // laser number  28
  +01.67989000,               // laser number  29
  +02.02080990,               // laser number  30
  -00.36240700,               // laser number  31
  -00.02235000,               // laser number  32

  // LOWER LASER
  -22.73788600,               // laser number  01
  -22.22607200,               // laser number  02
  -11.51392800,               // laser number  03
  -11.00211400,               // laser number  04
  -21.71468500,               // laser number  05
  -21.20368800,               // laser number  06
  -24.79027200,               // laser number  07
  -24.27632100,               // laser number  08
  -20.69303100,               // laser number  09
  -20.18268200,               // laser number  10
  -23.76296800,               // laser number  11
  -23.25017200,               // laser number  12
  -16.61531800,               // laser number  13
  -16.10593800,               // laser number  14
  -19.67259400,               // laser number  15
  -19.16272900,               // laser number  16
  -15.59649600,               // laser number  17
  -15.08695400,               // laser number  18
  -18.65304600,               // laser number  19
  -18.14350300,               // laser number  20
  -14.57727100,               // laser number  21
  -14.06740500,               // laser number  22
  -17.63406200,               // laser number  23
  -17.12468100,               // laser number  24
  -10.48982900,               // laser number  25
  -09.97703170,               // laser number  26
  -13.55731800,               // laser number  27
  -13.04696800,               // laser number  28
  -09.46367930,               // laser number  29
  -08.94972800,               // laser number  30
  -12.53631300,               // laser number  31
  -12.02531400,               // laser number  32
};

} // namespace velodyne
#endif
