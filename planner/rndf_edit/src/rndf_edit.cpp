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


#include <aw_roadNetwork.h>
#include <aw_roadNetworkSearch.h>
#include <iostream>
#include <iomanip>
#include <imagery.h>
#include <string>

#include <conout.h>
#include <REConsole.h>
#include <rndf_edit_gui.h>
#include <rndf_edit_app.h>

#include <QtGui/QtGui>
#include <QtGui/QApplication>

using namespace vlr;

namespace vlr {

QApplication* qtapp=NULL;
RNDFEditGUI* gui=NULL;

//typedef void (* event_callback) (event ev, ios_base& ios, int index);

class BufferedStringBuf: public std::streambuf {
public:
    BufferedStringBuf(unsigned int bufSize = 1024) {
        if (bufSize) {
            char *ptr = new char[bufSize];
            setp(ptr, ptr + bufSize);
        }
        else {
            setp(0, 0);
        }
    }

    virtual ~BufferedStringBuf() {
        sync();
        delete[] pbase();
    }

    virtual void writeString(const std::string &str) {
      QApplication::postEvent(gui->ui.reConsole, new REConsoleEvent(REConsoleEvent::WriteRequest, str.c_str()));
//        conout(str.c_str());
    }

private:
    int overflow(int c) {
        sync();

        if (c != EOF) {
            if (pbase() == epptr()) {
                std::string temp;

                temp += char(c);
                writeString(temp);
            }
            else {
                sputc(c);
            }
        }

        return 0;
    }

    int sync() {
        if (pbase() != pptr()) {
            int len = int(pptr() - pbase());
            std::string temp(pbase(), len);
            writeString(temp);
            setp(pbase(), epptr());
        }
        return 0;
    }
};

} // namespace vlr

using namespace vlr;

int main(int argc, char **argv) {

  if (argc <= 1) {
    std::cout << "Error: Missing RNDF filename.\n";
    std::cout << argv[0] << " rndf_filename [imagery_path]\n";
    std::cout << "If no imagery path is given, imagery is read from $VLR_ROOT/data/imagery.\n";
    char* race_home = getenv("VLR_ROOT");
    if (race_home) {
      std::cout << "Currently $VLR_ROOT points to " << race_home << std::endl;
    }
    else {
      std::cout << "Currently $VLR_ROOT is not set :-(\n";
    }
    exit(0);
  }
  glutInit(&argc, argv);

  std::string imagery_folder;

  if (argc > 2) {
    imagery_folder = argv[2];
  }
  else {
    std::string race_dir;
    char* race_home = getenv("VLR_ROOT");
    if (race_home) {
      race_dir = race_home;
    }
    else {
      race_dir = "~";
    }

    imagery_folder = race_dir + "/data/imagery";
  }

  qtapp = new RndfEditApp(argc, argv);
  qtapp->connect(qtapp, SIGNAL(lastWindowClosed()), qtapp, SLOT(quit()));


  int imagery_zoom_level = 10; //kogmo_params_get_param_int(params, "imagery", "zoom_level");
  std::string rndf_name(argv[1]);

 try {
  gui = new RNDFEditGUI(rndf_name, imagery_folder, imagery_zoom_level, 3, 3);
  gui->show();
 }
 catch(vlr::Ex<>& e) {
   std::cout << "Terminating because of: " << e.what() << std::endl;
 }

// BufferedStringBuf sbuf;
// std::streambuf* old_buf = std::cout.rdbuf(&sbuf);

 // ...and loop (QT)
  qtapp->exec();

//  std::cout.rdbuf(old_buf);
  delete gui;
  delete qtapp;

  return 0;
}
