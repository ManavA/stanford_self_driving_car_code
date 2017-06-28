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


#ifndef RE_ZONE_H_
#define RE_ZONE_H_

#include <REElement.h>
#include <REPerimeter.h>
#include <RESpot.h>

namespace vlr {

class REZone : public QObject, public REElement<rndf::Zone> {
  Q_OBJECT
public:
  REZone(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn, RESpot& re_spot, REPerimeter& re_perimeter);
  virtual ~REZone();

  rndf::Zone* create(double utm_x, double utm_y, const std::string& utm_zone);
  rndf::Zone* copy(rndf::Zone* source_segment, double delta_x, double delta_y);

  void move(rndf::Zone* l, double delta_x, double delta_y);
  void rotate(rndf::Zone* l, double center_x, double center_y, double theta);
  void updateGUI();

private:
  RESpot& re_spot_;
  REPerimeter& re_perimeter_;

private slots:
};

} // namespace vlr

#endif

