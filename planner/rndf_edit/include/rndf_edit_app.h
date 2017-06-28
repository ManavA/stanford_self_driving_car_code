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


#ifndef RNDF_EDIT_APP_H_
#define RNDF_EDIT_APP_H_

#include <iostream>
#include <global.h>
#include <QtGui/QtGui>

namespace vlr {

class RndfEditApp : public QApplication {
  Q_OBJECT

public:
  RndfEditApp(int& argc, char** argv) : QApplication(argc, argv) {}
  virtual ~RndfEditApp() {}


//  bool event(QEvent* ev) {
//    if(ev->type() == createDisplayEventType()) {
//      *static_cast<CreateDisplayEvent*>(ev)->display = new vlr::Display;
//      return true;
//    }
//    else if(ev->type() == showDisplayEventType()) {
//      static_cast<DisplayEvent*>(ev)->display->show();
//      return true;
//    }
//    else if(ev->type() == hideDisplayEventType()) {
//      static_cast<DisplayEvent*>(ev)->display->hide();
//      return true;
//    }
//    return false;
  //  }
  bool notify(QObject* receiver, QEvent* event) {
    try {
      return QApplication::notify(receiver, event);
    }
    catch(vlr::Ex<>& e) {
      std::cout << "Exception thrown:" << e.what() << std::endl;
      exit(-5);
    }
    return false;
  }
};

} // namespace vlr
#endif // RNDF_EDIT_APP_H_
