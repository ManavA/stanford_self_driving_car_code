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


#ifndef PAW2_APP_H_
#define PAW2_APP_H_

#include <QtGui/QtGui>
#include <display.h>

namespace vlr {
class Paw2Gui;

class CreateDisplayEvent : public QEvent {
 public:
  CreateDisplayEvent(QEvent::Type type) : QEvent(type), display(NULL) {}
  vlr::Display** display;
};

class DisplayEvent : public QEvent {
 public:
  DisplayEvent(QEvent::Type type) : QEvent(type), display(NULL) {}
  vlr::Display* display;
};

class StatusUpdate : public QEvent {
 public:
  StatusUpdate(QEvent::Type type) : QEvent(type) {}
};

class Paw2App : public QApplication {
  Q_OBJECT

public:
  Paw2App(int& argc, char** argv) : QApplication(argc, argv), create_display_event_type_(-1),
      show_display_event_type_(-1), hide_display_event_type_(-1), status_update_event_type_(-1) {
  }

  virtual ~Paw2App() {}

  bool notify(QObject* receiver, QEvent* event) {
    try {
      return QApplication::notify(receiver, event);
    }
    catch (vlr::Ex<>& e) {
      std::cout << "Exception thrown:" << e.what() << std::endl;
      exit(-5);
    }
    return false;
  }

  vlr::Display* createDisplay() {
    vlr::Display* display=NULL;
    CreateDisplayEvent* cde = new CreateDisplayEvent((QEvent::Type)createDisplayEventType());
    cde->display = &display;
    QCoreApplication::postEvent(this, cde);
    while(!display) {usleep(10000);}
    return display;
 }

  void showDisplay(vlr::Display& display) {
    DisplayEvent* sde = new DisplayEvent((QEvent::Type)showDisplayEventType());
    sde->display = &display;
    QCoreApplication::postEvent(this, sde);
 }

  void updateStatus(Paw2Gui* gui) {
    StatusUpdate* sue = new StatusUpdate((QEvent::Type)statusUpdateEventType());
    QCoreApplication::postEvent((QObject*)gui, sue);
 }

  bool event(QEvent* ev) {
    if(ev->type() == createDisplayEventType()) {
      *static_cast<CreateDisplayEvent*>(ev)->display = new vlr::Display;
      return true;
    }
    else if(ev->type() == showDisplayEventType()) {
      static_cast<DisplayEvent*>(ev)->display->show();
      return true;
    }
    else if(ev->type() == hideDisplayEventType()) {
      static_cast<DisplayEvent*>(ev)->display->hide();
      return true;
    }
    return false;
  }

  int createDisplayEventType() {
    if(create_display_event_type_<0) {
      create_display_event_type_ = QEvent::registerEventType();
    }
  return create_display_event_type_;
  }

  int showDisplayEventType() {
    if(show_display_event_type_<0) {
      show_display_event_type_ = QEvent::registerEventType();
    }
  return show_display_event_type_;
  }

  int hideDisplayEventType() {
    if(hide_display_event_type_<0) {
      hide_display_event_type_ = QEvent::registerEventType();
    }
  return hide_display_event_type_;
  }

  int statusUpdateEventType() {
    if(status_update_event_type_<0) {
      status_update_event_type_ = QEvent::registerEventType();
    }
  return status_update_event_type_;
  }

private:
  int create_display_event_type_;
  int show_display_event_type_;
  int hide_display_event_type_;
  int status_update_event_type_;
};

} // namespace vlr
#endif // PAW2_APP_H_
