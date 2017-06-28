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


#ifndef VLRIMAGING_IPLCONVERT_H_
#define VLRIMAGING_IPLCONVERT_H_

#ifdef HAVE_OPENCV
#include <opencv/cv.h>
#include <iostream>

#include <vlrImage.h>

namespace vlr {

template <class T> IplImage* convert2Ipl(const Image<T>&)
	{
	std::cout << "data type not supported by Ipl\n";
	return 0;
	}

template <> IplImage* convert2Ipl(const Image<uint8_t>&);
template <> IplImage* convert2Ipl(const Image<int8_t>&);
template <> IplImage* convert2Ipl(const Image<uint16_t>&);
template <> IplImage* convert2Ipl(const Image<int16_t>&);
template <> IplImage* convert2Ipl(const Image<int32_t>&);
template <> IplImage* convert2Ipl(const Image<float>&);
template <> IplImage* convert2Ipl(const Image<double>&);


ImageBase* convertFromIpl(const IplImage& iplImg);

}	 // namespace vlr

#endif // HAVE_OPENCV
#endif // VLRIMAGING_IPLCONVERT_H_
