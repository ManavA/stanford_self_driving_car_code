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


#include <QtGui/QtGui>
#include <rndf_edit_gui.h>
#include <RETrafficLight.h>

namespace vlr {

RETrafficLight::RETrafficLight(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn) : REElement<rndf::TrafficLight>(ui, rn) {
  connect(ui_->tlLat, SIGNAL( editFinished() ), this, SLOT(on_tlLat_editFinished()));
  connect(ui_->tlLon, SIGNAL( editFinished() ), this, SLOT(on_tlLon_editFinished()));
  connect(ui_->tlUtmX, SIGNAL( editFinished() ), this, SLOT(on_tlUtmX_editFinished()));
  connect(ui_->tlUtmY, SIGNAL( editFinished() ), this, SLOT(on_tlUtmY_editFinished()));
  connect(ui_->tlHeight, SIGNAL( editFinished() ), this, SLOT(on_tlHeight_editFinished()));
  connect(ui_->tlOrientation, SIGNAL( editFinished() ), this, SLOT(on_tlOrientation_editFinished()));
  connect(ui_->tlLinkedWayPoints, SIGNAL( itemChanged(QListWidgetItem*) ), this, SLOT(on_tlLinkedWayPoints_itemChanged(QListWidgetItem*)));
}

RETrafficLight::~RETrafficLight() {

}

rndf::TrafficLight* RETrafficLight::create(double utm_x, double utm_y, const std::string& utm_zone, double yaw) {

  elem_ = rn_->addTrafficLight();

  elem_->setUtm(utm_x, utm_y, utm_zone);
  elem_->z(5); // set default height;
  elem_->orientation(yaw);

 return elem_;
}

rndf::TrafficLight* RETrafficLight::copy(rndf::TrafficLight* source_tl, double delta_x, double delta_y) {
  if (!source_tl) {
    std::cout << "Cannot copy - no traffic light selected" << std::endl;
    return NULL;
  }

  elem_ = rn_->addTrafficLight();
  elem_->setUtm(source_tl->utmX() + delta_x, source_tl->utmY() + delta_y, source_tl->utmZone());
  elem_->z(source_tl->z());
  elem_->orientation(source_tl->orientation());
  return elem_;
}

void RETrafficLight::move(rndf::TrafficLight* tl, double delta_x, double delta_y) {
  if (!tl) {return;}

  tl->setUtm(tl->utmX() + delta_x, tl->utmY() + delta_y, tl->utmZone());
}

void RETrafficLight::rotate(rndf::TrafficLight* tl, double yaw) {
  if (!tl) {return;}

  tl->orientation(yaw*M_PI/180.0);
}

void RETrafficLight::updateGUI() {
  const QString baseTitle("Current TrafficLight: ");

  if (!elem_) {
    ui_->trafficLightBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->trafficLightBox->setTitle(baseTitle + elem_->name().c_str());
  ui_->tlLat->setValue(elem_->lat());
  ui_->tlLon->setValue(elem_->lon());
  ui_->tlUtmX->setValue(elem_->utmX());
  ui_->tlUtmY->setValue(elem_->utmY());
  ui_->tlHeight->setValue(elem_->z());
  ui_->tlOrientation->setValue(elem_->orientation());
  ui_->tlLinkedWayPoints->clear();

  std::map<std::string, rndf::WayPoint*>::const_iterator wpit = elem_->linkedWayPoints().begin(), wpit_end = elem_->linkedWayPoints().end();
  for(; wpit != wpit_end; wpit++) {
    ui_->tlLinkedWayPoints->addItem((*wpit).second->name().c_str());
    int num_items = ui_->tlLinkedWayPoints->count();
    QListWidgetItem* item = ui_->tlLinkedWayPoints->item(num_items-1);
    item->setFlags(item->flags() | Qt::ItemIsEditable);
  }
    // add empty item to allow adding new way points
  addEmptyLine(new_way_point_txt,  *ui_->tlLinkedWayPoints);
}

void RETrafficLight::on_tlLat_editFinished() {

  if(!elem_) {return;}
  if(ui_->tlLat->value() == elem_->lat()) {return;}

  elem_->setLatLon(ui_->tlLat->value(), elem_->lon());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RETrafficLight::on_tlLon_editFinished() {

  if(!elem_) {return;}
  if(ui_->tlLon->value() == elem_->lon()) {return;}

  elem_->setLatLon(elem_->lat(), ui_->tlLon->value());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RETrafficLight::on_tlUtmX_editFinished() {

  if(!elem_) {return;}
  if(utmDiffZero(ui_->tlUtmX->value(), elem_->utmX())) {return;}

  elem_->setUtm(ui_->tlUtmX->value(), elem_->utmY(), elem_->utmZone());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RETrafficLight::on_tlUtmY_editFinished() {

  if(!elem_) {return;}
  if(utmDiffZero(ui_->tlUtmY->value(), elem_->utmY())) {return;}

  elem_->setUtm(elem_->utmX(), ui_->tlUtmY->value(), elem_->utmZone());

  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RETrafficLight::on_tlHeight_editFinished() {

  if(!elem_) {return;}

  double height = ui_->tlHeight->value();
  if(height == elem_->z()) {return;}
  elem_->z(height);

  ui_->glWindow->requestRedraw();
}

void RETrafficLight::on_tlOrientation_editFinished() {

  if(!elem_) {return;}

  double yaw = ui_->tlOrientation->value();
  if(yaw == elem_->orientation()) {return;}
  elem_->orientation(yaw*M_PI/180.0);

  ui_->glWindow->requestRedraw();
}

void RETrafficLight::on_tlLinkedWayPoints_itemChanged(QListWidgetItem* item) {

  if (!elem_) {return;}
  if(item->text().toStdString() == new_way_point_txt) {return;}

  rndf::TWayPointMap::const_iterator wpit = rn_->wayPoints().find(item->text().toStdString());
  if (wpit != rn_->wayPoints().end()) {
    item->setForeground(QBrush(QColor(0, 0, 0)));
    elem_->addWayPoint((*wpit).second);
  }
  else {
//    ui_->tlLinkedWayPoints->removeItemWidget(item);
    delete ui_->tlLinkedWayPoints->takeItem(ui_->tlLinkedWayPoints->row(item));
  }

  addEmptyLine(new_way_point_txt, *ui_->tlLinkedWayPoints);

  ui_->glWindow->requestRedraw();
}

const std::string RETrafficLight::new_way_point_txt="<add way point>";

} // namespace vlr
