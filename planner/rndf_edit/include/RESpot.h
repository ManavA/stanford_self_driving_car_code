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


#ifndef RE_SPOT_H_
#define RE_SPOT_H_

#include <REElement.h>
#include <RESpotPoint.h>

namespace vlr {

class RESpot : public QObject, public REElement<rndf::Spot> {
  Q_OBJECT
public:
  //  RESpot(rndf::RoadNetwork& rn, rndf::Segment& s, double utm_x, double utm_y, double theta, double length);
//  RESpot(rndf::RoadNetwork& rn, rndf::Spot& lane);
  RESpot(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn, RESpotPoint& re_sp);
  virtual ~RESpot();

  rndf::Spot* create(rndf::Zone* s, double utm_x, double utm_y,  const std::string& utm_zone);
  rndf::Spot* copy(rndf::Spot* source_lane, rndf::Zone* dest_zone, double delta_x, double delta_y);
  void move(rndf::Spot* s, double delta_x, double delta_y);
  void rotate(rndf::Spot* s, double center_x, double center_y, double theta);
  void updateGUI();

private:
  RESpotPoint& re_sp_;

private slots:
  void on_spotWidth_valueChanged(double spot_width);
};

} // namespace vlr

#endif

