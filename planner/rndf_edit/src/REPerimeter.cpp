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
#include <REPerimeter.h>

using namespace vlr;

namespace vlr {

REPerimeter::REPerimeter(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn, REPerimeterPoint& re_pp) :
  REElement<rndf::Perimeter> (ui, rn), re_pp_(re_pp) {

//  QObject::connect(ui_->rightBoundary, SIGNAL(currentIndexChange()), this, SLOT(on_rightBoundary_currentIndexChanged()));
};

REPerimeter::~REPerimeter() {

};

rndf::Perimeter* REPerimeter::create(rndf::Zone* z, double utm_x, double utm_y, const std::string& utm_zone) {
  if (!z) {
    return NULL;
  }

  elem_ = rn_->addPerimeter(z);

    // add some default Perimeter points
  re_pp_.create(elem_, utm_x - 15, utm_y - 15, utm_zone);
  re_pp_.create(elem_, utm_x + 15, utm_y - 15, utm_zone);
  re_pp_.create(elem_, utm_x + 15, utm_y + 15, utm_zone);
  re_pp_.create(elem_, utm_x - 15, utm_y + 15, utm_zone);

  return elem_;
}

rndf::Perimeter* REPerimeter::copy(rndf::Perimeter* source_perimeter, rndf::Zone* dest_zone, double delta_x, double delta_y) {
  if (!dest_zone) {
    std::cout << "select a zone before copying" << std::endl;
    return NULL;
  }

  if (!source_perimeter) {
    std::cout << "select a perimeter before copying" << std::endl;
    return NULL;
  }

  elem_ = rn_->addPerimeter(dest_zone, dest_zone->getNextPerimeterStr());

  rndf::TPerimeterPointVec::const_iterator ppit = source_perimeter->perimeterPoints().begin(), ppit_end = source_perimeter->perimeterPoints().end();
  for (; ppit != ppit_end; ++ppit) {
    re_pp_.copy(*ppit, elem_, delta_x, delta_y);
  }

  return elem_;
}

void REPerimeter::move(rndf::Perimeter* p, double delta_x, double delta_y) {
  if (!p) {return;}

  rndf::TPerimeterPointVec::const_iterator ppit = p->perimeterPoints().begin(), ppit_end = p->perimeterPoints().end();

  for (; ppit != ppit_end; ++ppit) {
    re_pp_.move(*ppit, delta_x, delta_y);
  }
}

void REPerimeter::rotate(rndf::Perimeter* p, double center_x, double center_y, double theta) {
  if (!p) {return;}

  dgc::dgc_transform_t t;
  dgc::dgc_transform_identity(t);
  dgc::dgc_transform_rotate_z(t, theta);

  rndf::TPerimeterPointVec::const_iterator ppit = p->perimeterPoints().begin(), ppit_end = p->perimeterPoints().end();
  for (; ppit != ppit_end; ++ppit) {
    rndf::PerimeterPoint* pp = *ppit;
    double x = pp->utmX() - center_x;
    double y = pp->utmY() - center_y;
    double z = 0;
    dgc::dgc_transform_point(&x, &y, &z, t);
    pp->setUtm(x + center_x, y + center_y, pp->utmZone());
  }
}

void REPerimeter::updateGUI() {
  const QString baseTitle("Current Perimeter Point: ");

  if (!elem_) {
    ui_->perimeterBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->perimeterBox->setTitle(baseTitle + elem_->name().c_str());
}

} // namespace vlr
