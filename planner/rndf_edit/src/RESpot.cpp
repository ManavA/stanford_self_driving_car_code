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


#include <rndf_edit_gui.h>
#include <RESpot.h>

using namespace vlr;

namespace vlr {

RESpot::RESpot(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn, RESpotPoint& re_sp) :
  REElement<rndf::Spot> (ui, rn), re_sp_(re_sp) {
  connect(ui_->spotWidth, SIGNAL( valueChanged(double) ), this, SLOT(on_spotWidth_valueChanged(double)));
}

RESpot::~RESpot() {

}

rndf::Spot* RESpot::create(rndf::Zone* z, double utm_x, double utm_y, const std::string& utm_zone) {
  if (!z) {return NULL;}

  elem_ = rn_->addSpot(z);

  double half_spot_width = ui_->spotWidth->value() / 2;

     // add default spot points
  re_sp_.create(elem_, utm_x - half_spot_width, utm_y - 3, utm_zone);
  re_sp_.create(elem_, utm_x + half_spot_width, utm_y + 3, utm_zone);

  return elem_;
}

rndf::Spot* RESpot::copy(rndf::Spot* source_spot, rndf::Zone* dest_zone, double delta_x, double delta_y) {
  if (!dest_zone) {
    std::cout << "select a zone before copying" << std::endl;
    return NULL;
  }

  if (!source_spot) {
    std::cout << "select a Spot before copying" << std::endl;
    return NULL;
  }

  elem_ = rn_->addSpot(dest_zone, dest_zone->getNextSpotStr());

  rndf::TWayPointVec::const_iterator wpit = source_spot->wayPoints().begin(), wpit_end = source_spot->wayPoints().end();
  for (; wpit != wpit_end; ++wpit) {
    re_sp_.copy(*wpit, elem_, delta_x, delta_y);
  }

  return elem_;
}

void RESpot::move(rndf::Spot* s, double delta_x, double delta_y) {
  if (!s) {return;}

  rndf::TWayPointVec::const_iterator wpit = s->wayPoints().begin(), wpit_end = s->wayPoints().end();
  for (; wpit != wpit_end; ++wpit) {
    re_sp_.move(*wpit, delta_x, delta_y);
  }
}

void RESpot::rotate(rndf::Spot* s, double center_x, double center_y, double theta) {
  if (!s) {return;}

  dgc::dgc_transform_t t;
  dgc::dgc_transform_identity(t);
  dgc::dgc_transform_rotate_z(t, theta);

  rndf::TWayPointVec::const_iterator wpit = s->wayPoints().begin(), wpit_end = s->wayPoints().end();

  for (; wpit != wpit_end; ++wpit) {
    rndf::WayPoint* wp = *wpit;
    double x = wp->utmX() - center_x;
    double y = wp->utmY() - center_y;
    double z = 0;
    dgc::dgc_transform_point(&x, &y, &z, t);
    wp->setUtm(x + center_x, y + center_y, wp->utmZone());
  }
}

void RESpot::updateGUI() {
  const QString baseTitle("Current Spot: ");

  if (!elem_) {
    ui_->spotBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->spotBox->setTitle(baseTitle + elem_->name().c_str());
}

void RESpot::on_spotWidth_valueChanged(double spot_width) {
  if (!elem_) {return;}
  elem_->setSpotWidth(spot_width);
  ui_->glWindow->requestRedraw();
}

} // namespace vlr
