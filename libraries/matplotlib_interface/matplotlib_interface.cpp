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


#include <matplotlib_interface/matplotlib_interface.h>

using namespace std;
using namespace Eigen;

void mpliNamedExport(const std::string& name, double dbl) {
  PyRun_SimpleString((name + string(" = ") + mpliToArray(dbl)).c_str());
}

void mpliNamedExport(const std::string& name, const Eigen::VectorXd& vec) {
  PyRun_SimpleString((name + string(" = ") + mpliToArray(vec)).c_str());
}

void mpliNamedExport(const std::string& name, const Eigen::MatrixXd& mat) {
  PyRun_SimpleString((name + string(" = ") + mpliToArray(mat)).c_str());
}

void mpliExecuteFile(const string& filename) {
  FILE *fp = fopen(filename.c_str(), "r");
  PyRun_SimpleFile(fp, filename.c_str());
  fclose(fp);
}

void mpli(const string& str) {
  PyRun_SimpleString(str.c_str());
}

void mpliBegin() {

  if(g_mpli_begun) {
    cerr << "Error!  You cannot call mpliBegin() more than once.  This is because of a numpy bug that prevents importing more than once.  The global bool g_mpli_begun is true if mpliBegin() has ever been called." << endl;
    assert(0);
  }

  Py_Initialize();
  PyRun_SimpleString("import numpy");
  PyRun_SimpleString("import warnings");
  PyRun_SimpleString("warnings.simplefilter('ignore')"); // waitforbuttonpress causes a warning.  Is there a better function to use?  Warnings should be on..
  g_mpli_begun = true;
}

void mpliEnd() {
  Py_Finalize();
}

string mpliToArray(double val) {
  ostringstream oss;
  oss << "numpy.array( [" << val << "] )"; // Should this go to a double rather than a numpy array of size 1?
  return oss.str();
}

string mpliToArray(const VectorXd& vec) {
  ostringstream oss;
  oss << "numpy.array( [";
  for(int i = 0; i < vec.rows(); ++i) {
    oss << vec(i);
    if(i != vec.rows() - 1)
      oss << ", ";
  }
  oss << "] )";
  return oss.str();
}

string mpliToArray(const MatrixXd& mat) {
  ostringstream oss;
  oss << "numpy.array([";
  for(int i = 0; i < mat.rows(); ++i) {
    oss << "[";
    for(int j = 0; j < mat.cols(); ++j) { 
      oss << mat(i, j);
      if(j != mat.cols() - 1)
	oss << ", ";
    }
    oss << "]";
    if(i != mat.rows() - 1)
      oss << ",";
  }
  oss << "] )";
  return oss.str();
}
