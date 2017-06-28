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


#ifndef VLRIMAGING_REORGANIZE_H_
#define VLRIMAGING_REORGANIZE_H_

namespace vlr {

typedef enum {COLORORG_GRAY, COLORORG_RGB, COLORORG_RGBA, COLORORG_BGR, COLORORG_BGRA} ColorOrganization_t;

template <class T, ColorOrganization_t colorOrg>
class cpReorganize {
public:
void chunky2Planar(const T*, uint32_t, vlr::Image<T>&)
	{
	std::cout << "Unknown color format.\n";
	}

void planar2Chunky(const vlr::Image<T>&, T*, uint32_t)
	{
	std::cout << "Unknown color format.\n";
	}
};

template <class T> class cpReorganize<T, COLORORG_RGB>
{
public:
	void chunky2Planar(const T* chunky, uint32_t cPaddedWidth, Image<T>& img)
		{
		uint32_t cLineOffset=cPaddedWidth-img.width();
		uint32_t pLineOffset=img.paddedWidth()-img.width();

		T* rdata = img.data();
		T* gdata = rdata + img.paddedWidth()*img.height();
		T* bdata = gdata + img.paddedWidth()*img.height();

		for(uint32_t y=0; y<img.height(); y++)
			{
			for(uint32_t x=0; x<img.width(); x++)
				{
				*rdata++ = *chunky++;
				*gdata++ = *chunky++;
				*bdata++ = *chunky++;
				}

			rdata += pLineOffset;
			gdata += pLineOffset;
			bdata += pLineOffset;
			chunky += cLineOffset;
			}
		}

	void planar2Chunky(const Image<T>& img, T* chunky, uint32_t cPaddedWidth)
		{
		uint32_t cLineOffset=cPaddedWidth-img.width();
		uint32_t pLineOffset=img.paddedWidth()-img.width();

		const T* rdata = img.data();
		const T* gdata = rdata + img.paddedWidth()*img.height();
		const T* bdata = gdata + img.paddedWidth()*img.height();

		for(uint32_t y=0; y<img.height(); y++)
			{
			for(uint32_t x=0; x<img.width(); x++)
				{
				*chunky++ = *rdata++;
				*chunky++ = *gdata++;
				*chunky++ = *bdata++;
				}

			rdata += pLineOffset;
			gdata += pLineOffset;
			bdata += pLineOffset;
			chunky += cLineOffset;
			}
		}
};

}	// namespace vlr

#endif // VLRIMAGING_REORGANIZE_H_
