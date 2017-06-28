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


#ifndef MATPLOTLIB_INTERFACE_H
#define MATPLOTLIB_INTERFACE_H

#include <iostream>
#include <sstream>
#include <cstdarg>
#include <python2.6/Python.h>
#include <Eigen/Eigen>
#include <boost/algorithm/string.hpp>


/**********************************************
 * Things you will find useful.
 *********************************************/

//! Export an Eigen::VectorXd or Eigen::MatrixXd to a python numpy array with the same name as in the c++ program.
#define mpliExport(name) PyRun_SimpleString((std::string(#name) + std::string(" = ") + mpliToArray(name)).c_str())

//! true if mpliBegin() has ever been called.  mpli_end() does NOT clear this, as this variable prevents mpliBegin() from being called more than once due to a numpy bug.
bool g_mpli_begun = false;

//! Call before running anything else mpli-related.  Can only be called once because of a numpy bug.
void mpliBegin();
//! Call after done with everything mpli-related to clean up python.
void mpliEnd();
//! Export a double to a python numpy array with a new name.
void mpliNamedExport(const std::string& name, double dbl);
//! Export an Eigen::VectorXd to a python numpy array with a new name.
void mpliNamedExport(const std::string& name, const Eigen::VectorXd& vec);
//! Export an Eigen::MatrixXd to a python numpy array with a new name.
void mpliNamedExport(const std::string& name, const Eigen::MatrixXd& mat);
//! Executes the contents of a python file.
void mpliExecuteFile(const std::string& filename);
//! Executes str as a python command.
void mpli(const std::string& str);


/**********************************************
 * Backend stuff you don't need to care about.
 *********************************************/

//! Helper function for exportVector
std::string mpliToArray(const Eigen::VectorXd& vec);
//! Helper function for exportVector
std::string mpliToArray(const Eigen::MatrixXd& mat);
//! Helper function for exportVector
std::string mpliToArray(double val);

#endif // MATPLOTLIB_INTERFACE_H
