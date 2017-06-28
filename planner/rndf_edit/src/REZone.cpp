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


#include <string>
#include <rndf_edit_gui.h>
#include <REZone.h>

using namespace vlr;

namespace vlr {

REZone::REZone(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn, RESpot& re_spot, REPerimeter& re_perimeter) :
  REElement<rndf::Zone> (ui, rn), re_spot_(re_spot), re_perimeter_(re_perimeter) {

}

REZone::~REZone() {
}

rndf::Zone* REZone::create(double utm_x, double utm_y, const std::string& utm_zone) {
  elem_ = rn_->addZone();

  re_perimeter_.create(elem_, utm_x, utm_y, utm_zone);
  return elem_;
}

rndf::Zone* REZone::copy(rndf::Zone* source_zone, double delta_x, double delta_y) {
  if (!source_zone) {
    std::cout << "select a Zone before copying" << std::endl;
    return NULL;
  }

  elem_ = rn_->addZone(rn_->nextZoneStr());

  rndf::TPerimeterMap::const_iterator pit = source_zone->perimeters().begin(), pit_end = source_zone->perimeters().end();
  for (; pit != pit_end; ++pit) {
    re_perimeter_.copy((*pit).second, elem_, delta_x, delta_y);
  }

  rndf::TSpotMap::const_iterator sit = source_zone->spots().begin(), sit_end = source_zone->spots().end();
  for (; sit != sit_end; ++sit) {
    re_spot_.copy((*sit).second, elem_, delta_x, delta_y);
  }

  return elem_;
}

void REZone::move(rndf::Zone* z, double delta_x, double delta_y) {
  if (!z) {return;}

  rndf::TPerimeterMap::const_iterator pit = z->perimeters().begin(), pit_end = z->perimeters().end();
  for (; pit != pit_end; ++pit) {
    re_perimeter_.move((*pit).second, delta_x, delta_y);
  }

  rndf::TSpotMap::const_iterator sit = z->spots().begin(), sit_end = z->spots().end();
  for (; sit != sit_end; ++sit) {
    re_spot_.move((*sit).second, delta_x, delta_y);
  }
}

void REZone::rotate(rndf::Zone* z, double center_x, double center_y, double theta) {
  if (!z) {return;}

  rndf::TPerimeterMap::const_iterator pit = z->perimeters().begin(), pit_end = z->perimeters().end();
  for (; pit != pit_end; ++pit) {
    re_perimeter_.rotate((*pit).second, center_x, center_y, theta);
  }

  rndf::TSpotMap::const_iterator sit = z->spots().begin(), sit_end = z->spots().end();
  for (; sit != sit_end; ++sit) {
    re_spot_.rotate((*sit).second, center_x, center_y, theta);
  }
}

void REZone::updateGUI() {
  const QString baseTitle("Current Zone: ");

  if (!elem_) {
    ui_->zoneBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->zoneBox->setTitle(baseTitle + elem_->name().c_str());
}


} // namespace vlr
