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
#include <QtGui/QtGui>
#include <rndf_edit_gui.h>
#include <RESegment.h>

namespace vlr {

RESegment::RESegment(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn, RELane& re_lane) :
  REElement<rndf::Segment> (ui, rn), re_lane_(re_lane) {
};

RESegment::~RESegment() {

};

rndf::Segment* RESegment::create(double utm_x, double utm_y, const std::string& utm_zone) {

  elem_ = rn_->addSegment();

  double lane_width = ui_->laneWidth->value();
  double lane_length = 20;

  // add lanes
  rndf::Lane* lane = re_lane_.create(elem_, utm_x, utm_y - lane_width / 2, utm_zone, 0., lane_length, lane_width);
  lane->setLeftBoundaryType(rndf::Lane::BrokenWhite);
  lane->setRightBoundaryType(rndf::Lane::SolidWhite);

  lane = re_lane_.create(elem_, utm_x, utm_y + lane_width / 2, utm_zone, M_PI, lane_length, lane_width);
  lane->setLeftBoundaryType(rndf::Lane::BrokenWhite);
  lane->setRightBoundaryType(rndf::Lane::SolidWhite);

  //  for (int l = 0; l < 2; ++l) {
  //    re_lane_.crate(elem_, utm_x, utm_y + l*lane_width-lane_width/2, utm_zone);
  //  }

return elem_;
}

rndf::Segment* RESegment::copy(rndf::Segment* source_segment, double delta_x, double delta_y) {

  if (!source_segment) {
    std::cout << "select a Segment before copying" << std::endl;
    return NULL;
  }

  std::string strSegmentName = rn_->nextSegmentStr();
  rndf::Segment* dest_segment = rn_->addSegment(strSegmentName);

  rndf::TLaneSet::const_iterator lit, lit_end;
  for (lit = source_segment->getLanes().begin(), lit_end = source_segment->getLanes().end(); lit != lit_end; ++lit) {
    re_lane_.copy(*lit, dest_segment, delta_x, delta_y);
  }

  elem_ = dest_segment;
  return elem_;
}

void RESegment::move(rndf::Segment* s, double delta_x, double delta_y) {
  if (!s) {return;}

  rndf::TLaneSet::const_iterator lit, lit_end;

  for (lit = s->getLanes().begin(), lit_end = s->getLanes().end(); lit != lit_end; ++lit) {
    re_lane_.move(*lit, delta_x, delta_y);
  }
}

void RESegment::rotate(rndf::Segment* s, double center_x, double center_y, double theta) {
  if (!s) {return;}

  rndf::TLaneSet::const_iterator lit, lit_end;
  for (lit = s->getLanes().begin(), lit_end = s->getLanes().end(); lit != lit_end; ++lit) {
    re_lane_.rotate(*lit, center_x, center_y, theta);
  }
}

void RESegment::updateGUI() {
  const QString baseTitle("Current Segment: ");

  if (!elem_) {
    ui_->segmentBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->segmentBox->setTitle(baseTitle + elem_->name().c_str());
}

} // namespace vlr
