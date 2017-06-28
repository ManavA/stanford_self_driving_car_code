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


#ifndef HORNER_H_
#define HORNER_H_

#include <stdio.h>
#include <stdlib.h>
#include <vector>

namespace vlr {

//using namespace std;

// HORNERSCHEMA: Berechnnung aller Ableitungen eines Polynoms an der Stelle x
// Eingabe: Grad n; Zeiger auf Koeffizienten a, Argument x
// Ausgabe: Wert des Polynoms als Rueckgabewert
// Zeiger auf w, w[i] enth�lt den Wert der iten Ableitung
//ACHTUNG: Die Zeiger a und w m�ssen auf Felder der L�nge n+1 zeigen.

//double Horner(int n, double* a, double x, double* w) {
//	for (int i=0; i<n+1; i++) w[i]=a[n]; // w[0] wird Rueckgabewert
//    for (int i=n-1; i>=0; i--) {
//        w[0]=w[0]*x + a[i];
//        for (int j=1; j<=i; j++) w[j] = w[j]*x + w[j-1];
//        }
//        for (int i=2; i<=n; i++){
//            for (int j=2; j<=i; j++) w[i]=w[i]*j; // Multiplikation mit i!
//        }
//        return w[0];
//}


// New implementation:
int horner(const std::vector<double>& a, double x, std::vector<double>& w);

} // namespace vlr

#endif // HORNER_H_
