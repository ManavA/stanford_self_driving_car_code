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

#include <horner.h>

namespace vlr {

//int horner(const std::vector<double>& a, double x, std::vector<double>& w) {
//    int n = a.size()-1;
//    if ( (int)w.size() != n+1) {return -1;}
//
//	double an = a[n];
//	w.assign(n+1, an); // w[0] wird Rueckgabewert
//	std::vector<double>::const_reverse_iterator it=a.rbegin(); it++;
//	for (int i=n-1; it!=a.rend(); it++, i--) {
//		std::vector<double>::iterator it2 = w.begin();
//		std::vector<double>::iterator it3 = it2; it3++;
//		*(it2) = *(it2)*x + (*it);
//		for (int j=1; j<=i; j++, it2++, it3++) {
//			*(it3) = *(it3)*x + (*it2);
//	    }
//	}
//
//	std::vector<double>::iterator it2 = w.begin(); it2++; it2++;
//	for (int i=2; i<=n; i++, it2++){
//		for (int j=2; j<=i; j++) (*it2)*=j; // Multiplikation mit i!
//		}
//	return 0;
//}



int horner(const std::vector<double>& a, double x, std::vector<double>& w) {
    int n = a.size()-1;
    if ( (int)w.size() != n+1) {return -1;}

  w.assign(n+1, a[n]); // w[0] is also return value

  for (int i=n-1; i>=0; i--) {
		w[0]=w[0]*x + a[i];
		for (int j=1; j<=i; j++) w[j] = w[j]*x + w[j-1];
	}

  for (int i=2; i<=n; i++){
		for (int j=2; j<=i; j++) w[i]=w[i]*j; // multiply with i!
		}
	return 0;
}

} // namespace vlr
