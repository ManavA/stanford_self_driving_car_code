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


#ifndef VLRIMAGING_MISC_H_
#define VLRIMAGING_MISC_H_

#include <cmath>
#include <vlrImage.h>

namespace vlr {

template <class T> Image<T> convert(ImageBase& img)
	{
	if(dynamic_cast<Image<unsigned char>*>(&img))
		{return Image<T>(*dynamic_cast<Image<unsigned char>*>(&img));}
	else if(dynamic_cast<Image<char>*>(&img))
		{return Image<T>(*dynamic_cast<Image<char>*>(&img));}
	else if(dynamic_cast<Image<unsigned short>*>(&img))
		{return Image<T>(*dynamic_cast<Image<unsigned short>*>(&img));}
	else if(dynamic_cast<Image<short>*>(&img))
		{return Image<T>(*dynamic_cast<Image<short>*>(&img));}
	else if(dynamic_cast<Image<unsigned int>*>(&img))
		{return Image<T>(*dynamic_cast<Image<unsigned int>*>(&img));}
	else if(dynamic_cast<Image<int>*>(&img))
		{return Image<T>(*dynamic_cast<Image<int>*>(&img));}
	else if(dynamic_cast<Image<float>*>(&img))
		{return Image<T>(*dynamic_cast<Image<float>*>(&img));}
	else if(dynamic_cast<Image<double>*>(&img))
		{return Image<T>(*dynamic_cast<Image<double>*>(&img));}
//	else if(dynamic_cast<Image<ImageBase::complex>*>(&img))
//		{return Image<T>(*dynamic_cast<Image<ImageBase::complex>*>(&img));}
//	else if(dynamic_cast<Image<ImageBase::dcomplex>*>(&img))
//		{return Image<T>(*dynamic_cast<Image<ImageBase::dcomplex>*>(&img));}

	throw("Unsupported data type.");
	}
}	// namespace vlr

#endif // VLRIMAGING_MISC_H_
