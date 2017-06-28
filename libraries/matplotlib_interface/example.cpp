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

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
  
  // -- Initialize use of matplotlib-interface.
  mpliBegin();
  
  // -- Hello world with python.
  PyRun_SimpleString("print 'This is python speaking.'");
  cout << endl;

  // -- Export a double into python as a numpy array with size one.
  double dbl = 42.42;
  mpliExport(dbl); // A numpy array with the name 'dbl' now exists in python.
  cout << "dbl in C++: " << dbl << endl << endl;
  mpli("print 'dbl in python: '");
  mpli("print dbl");
  cout << endl;
  
  // -- Export an Eigen::VectorXd or Eigen::MatrixXd into python.
  VectorXd vec = VectorXd::Random(3);
  MatrixXd mat = MatrixXd::Random(3, 3);
  mpliExport(vec); // A numpy array with the name 'vec' now exists in python.
  mpliExport(mat); // A numpy array with the name 'mat' now exists in python.

  cout << "vec in C++: " << endl << vec << endl << endl;
  mpli("print 'vec in python: '");
  mpli("print vec");
  cout << endl;
  mpli("print 'vec.transpose() in python.  Note that there is no difference - it is a 1d array:'");
  mpli("print vec.transpose()");
  cout << endl;

  cout << "mat in C++: " << endl << mat << endl << endl;
  mpli("print 'mat in python: '");
  mpli("print mat");
  cout << endl;
  
  // -- Export a vector or matrix with a new name.
  VectorXd *rand = new VectorXd(2);
  *rand = VectorXd::Random(2);
  mpliNamedExport("new_name", *rand); // mpliExport(*rand) won't work because of the dereferencing *, so use mpliNamedExport instead.
  mpli("print 'new_name in python:'");
  mpli("print new_name");
  cout << endl;
  
  // -- Execute multiple lines in a string.
  ostringstream oss;
  oss << "print 'Multi-line string execution example.'" << endl;
  oss << "a = ['foo', 'bar']" << endl;
  oss << "print a[0]" << endl;
  oss << "print a[1]" << endl;
  cout << "Executing the following:" << endl << oss.str() << endl;
  mpli(oss.str());
  cout << endl;

  cout << "Press return to show some example plots." << endl;
  cin.ignore();
  mpli("from pylab import *");
  
  // -- Simple plot demo.
  VectorXd x(100);
  for(int i = 0; i < x.rows(); ++i) {
    x(i) = i*i;
  }
  mpliExport(x);
  mpli("plot(x)");
  mpli("draw()");
  mpli("waitforbuttonpress()");
  mpli("clf()");

  // -- Execute an entire file.  Export variables in c++ that describe a Gaussian, then display it with mpli.
  double var_x = 1;
  double var_y = 1;
  double var_xy = 0.5;
  double mu_x = 1;
  double mu_y = 2;
  mpliExport(var_x);
  mpliExport(var_y);
  mpliExport(var_xy);
  mpliExport(mu_x);
  mpliExport(mu_y);
  mpliExecuteFile("example_gaussian.py");

  // -- Histogram demo.
  VectorXd samples(10000);
  for(int i = 0; i < samples.rows(); ++i)
    samples(i) = 0.5 * VectorXd::Random(12).sum(); // Sample from normal distribution with mean 0, var 1.
  mpliExport(samples);
  mpli("hist(samples, 50, facecolor='green', alpha=0.75)");
  mpli("draw()");
  mpli("waitforbuttonpress()");
  mpli("clf()");
  
  mpliEnd();

  return 0;
}

void numpyMultipleImportTest() {
  // -- The Py_Initialize, import numpy, Py_Finalize loop segfaults the second time through due to a bug in numpy.
  //    If you remove the finalize call, it looks like it's only a constant and small memory leak.
  for(int i = 0; i < 20; ++i) {
    cout << i << endl;
    Py_Initialize();
    mpli("'print hi...'");
    mpli("import numpy");
    //Py_Finalize();
  }  
}
