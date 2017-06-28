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


#ifndef RE_ELEMENT_H_
#define RE_ELEMENT_H_

#include <QtGui/QtGui>
#include <aw_roadNetwork.h>

namespace Ui {
class RNDFEdit;
}

namespace vlr {

class REElementBase : public QObject {
  Q_OBJECT

public:
  REElementBase(QWidget* parent = NULL) {}
  virtual ~REElementBase() {}

};

template <class T> class REElement { // : public REElementBase {

public:
   REElement(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn) : ui_(ui), rn_(rn), elem_(NULL) {}

  virtual ~REElement() {}
//  template <class T> REElement(T& element);
//  virtual ~REElement();
//
//  template <class T, P> virtual void copy();
  void setRoadNetwork(rndf::RoadNetwork* rn) {rn_ = rn;}

  T* current() {return elem_;}
  void select(T* elem) {elem_ = elem;}
  void deselect() {elem_ = NULL;}

  static inline bool utmDiffZero(double c1, double c2) {
      return std::abs(c2 - c1) < 1e-5;
  }

//  static void addEmptyLine(const std::string& text, QListWidget& list);
  static void addEmptyLine(const std::string& text, QListWidget& list) {
    QListWidgetItem* item = list.item(list.count()-1);

    if(item) {
      if(item->text().toStdString() == text) {return;}
    }

    list.addItem(text.c_str());
    item = list.item(list.count()-1);
    item->setForeground(QBrush(QColor(150, 150, 150)));
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    }

protected:
  Ui::RNDFEdit* ui_;
  rndf::RoadNetwork* rn_;
  T* elem_;
};

} // namespace vlr

#endif

