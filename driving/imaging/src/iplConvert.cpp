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


#include <iostream>

#ifdef HAVE_OPENCV

#include <cv.h>
#include <base.h>

#include <iplConvert.h>
#include <reorganize.h>

namespace vlr {

template <> IplImage* convert2Ipl(const Image<uint8_t>& img)
	{
	if(img.channels() != 1 && img.channels() != 3) {return 0;}

	IplImage* iplImg = cvCreateImage( cvSize(img.width(), img.height()), IPL_DEPTH_8U, img.channels() );

	if(img.channels() == 1)
		{
		const uint8_t* data=img.const_data();
		uint8_t* resdata=(uint8_t*)iplImg->imageData;
		uint32_t dstPaddedWidth=iplImg->widthStep/sizeof(uint8_t);

		for(uint32_t y=0; y<img.height(); y++)
			{
			memcpy(&resdata[y*dstPaddedWidth], &data[y*img.paddedWidth()],img.width()*sizeof(uint8_t));
			}
		return iplImg;
		}

	cpReorganize<uint8_t, COLORORG_RGB> reorg;

	reorg.planar2Chunky(img, (uint8_t*)iplImg->imageData, iplImg->widthStep/(3*sizeof(uint8_t)));

	return iplImg;
	}

template <> IplImage* convert2Ipl(const Image<int8_t>& img)
	{
	if(img.channels() != 1 && img.channels() != 3) {return 0;}

	IplImage* iplImg = cvCreateImage( cvSize(img.width(), img.height()), IPL_DEPTH_8S, img.channels() );

	if(img.channels() == 1)
		{
		const int8_t* data=img.const_data();
		int8_t* resdata=(int8_t*)iplImg->imageData;
		uint32_t dstPaddedWidth=iplImg->widthStep/sizeof(int8_t);

		for(uint32_t y=0; y<img.height(); y++)
			{
			memcpy(&resdata[y*dstPaddedWidth], &data[y*img.paddedWidth()],img.width()*sizeof(int8_t));
			}

		return iplImg;
		}

	cpReorganize<int8_t, COLORORG_RGB> reorg;

	reorg.planar2Chunky(img, (int8_t*)iplImg->imageData, iplImg->widthStep/(3*sizeof(int8_t)));

	return iplImg;
	}

template <> IplImage* convert2Ipl(const Image<uint16_t>& img)
	{
	if(img.channels() != 1 && img.channels() != 3) {return 0;}

	IplImage* iplImg = cvCreateImage( cvSize(img.width(), img.height()), IPL_DEPTH_16U, img.channels() );

	if(img.channels() == 1)
		{
		const uint16_t* data=img.const_data();
		uint16_t* resdata=(uint16_t*)iplImg->imageData;
		uint32_t dstPaddedWidth=iplImg->widthStep/sizeof(uint16_t);

		for(uint32_t y=0; y<img.height(); y++)
			{
			memcpy(&resdata[y*dstPaddedWidth], &data[y*img.paddedWidth()],img.width()*sizeof(uint16_t));
			}

		return iplImg;
		}

	cpReorganize<uint16_t, COLORORG_RGB> reorg;

	reorg.planar2Chunky(img, (uint16_t*)iplImg->imageData, iplImg->widthStep/(3*sizeof(uint16_t)));

	return iplImg;
	}

template <> IplImage* convert2Ipl(const Image<int16_t>& img)
	{
	if(img.channels() != 1 && img.channels() != 3) {return 0;}

	IplImage* iplImg = cvCreateImage( cvSize(img.width(), img.height()), IPL_DEPTH_16S, img.channels() );

	if(img.channels() == 1)
		{
		const int16_t* data=img.const_data();
		int16_t* resdata=(int16_t*)iplImg->imageData;
		uint32_t dstPaddedWidth=iplImg->widthStep/sizeof(int16_t);

		for(uint32_t y=0; y<img.height(); y++)
			{
			memcpy(&resdata[y*dstPaddedWidth], &data[y*img.paddedWidth()],img.width()*sizeof(int16_t));
			}

		return iplImg;
		}

	cpReorganize<int16_t, COLORORG_RGB> reorg;

	reorg.planar2Chunky(img, (int16_t*)iplImg->imageData, iplImg->widthStep/(3*sizeof(int16_t)));

	return iplImg;
	}

template <> IplImage* convert2Ipl(const Image<int32_t>& img)
	{
	if(img.channels() != 1 && img.channels() != 3) {return 0;}

	IplImage* iplImg = cvCreateImage( cvSize(img.width(), img.height()), IPL_DEPTH_32S, img.channels() );

	if(img.channels() == 1)
		{
		const int32_t* data=img.const_data();
		int32_t* resdata=(int32_t*)iplImg->imageData;
		uint32_t dstPaddedWidth=iplImg->widthStep/sizeof(int32_t);

		for(uint32_t y=0; y<img.height(); y++)
			{
			memcpy(&resdata[y*dstPaddedWidth], &data[y*img.paddedWidth()],img.width()*sizeof(int32_t));
			}

		return iplImg;
		}

	cpReorganize<int32_t, COLORORG_RGB> reorg;

	reorg.planar2Chunky(img, (int32_t*)iplImg->imageData, iplImg->widthStep/(3*sizeof(int32_t)));

	return iplImg;
	}

template <> IplImage* convert2Ipl(const Image<float>& img)
	{
	if(img.channels() != 1 && img.channels() != 3) {return 0;}

	IplImage* iplImg = cvCreateImage( cvSize(img.width(), img.height()), IPL_DEPTH_32F, img.channels() );

	if(img.channels() == 1)
		{
		const float* data=img.const_data();
		float* resdata=(float*)iplImg->imageData;
		uint32_t dstPaddedWidth=iplImg->widthStep/sizeof(float);

		for(uint32_t y=0; y<img.height(); y++)
			{
			memcpy(&resdata[y*dstPaddedWidth], &data[y*img.paddedWidth()],img.width()*sizeof(float));
			}

		return iplImg;
		}

	cpReorganize<float, COLORORG_RGB> reorg;

	reorg.planar2Chunky(img, (float*)iplImg->imageData, iplImg->widthStep/(3*sizeof(float)));

	return iplImg;
	}

template <> IplImage* convert2Ipl(const Image<double>& img)
	{
	if(img.channels() != 1 && img.channels() != 3) {return 0;}

	IplImage* iplImg = cvCreateImage( cvSize(img.width(), img.height()), IPL_DEPTH_64F, img.channels() );

	if(img.channels() == 1)
		{
		const double* data=img.const_data();
		double* resdata=(double*)iplImg->imageData;
		uint32_t dstPaddedWidth=iplImg->widthStep/sizeof(double);

		for(uint32_t y=0; y<img.height(); y++)
			{
			memcpy(&resdata[y*dstPaddedWidth], &data[y*img.paddedWidth()],img.width()*sizeof(double));
			}

		return iplImg;
		}

	cpReorganize<double, COLORORG_RGB> reorg;

	reorg.planar2Chunky(img, (double*)iplImg->imageData, iplImg->widthStep/(3*sizeof(double)));

	return iplImg;
	}

template <class T> inline void convertFromIplGray_internal(const IplImage& iplImg, Image<T>& img)
	{
	uint32_t srcPaddedWidth=iplImg.widthStep/sizeof(T);
	uint32_t srcPlaneSize=srcPaddedWidth*iplImg.height;
	uint32_t dstPlaneSize=img.paddedWidth()*img.height();
	const T* data=(const T*)iplImg.imageData;
	T* resdata=img.data();

	for(uint32_t c=0; c<img.channels(); c++)
		{
		for(uint32_t y=0; y<img.height(); y++)
			{
				memcpy(&resdata[c*dstPlaneSize+y*img.paddedWidth()],
						&data[c*srcPlaneSize+y*srcPaddedWidth],img.width()*sizeof(T));
			}
		}
	}

template <class T> inline void convertFromIplRGB_internal(const IplImage& iplImg, Image<T>& img)
	{
	uint32_t srcPaddedWidth=iplImg.widthStep/(3*sizeof(T));
	uint32_t dstPlaneSize=img.paddedWidth()*img.height();
	const T* data=(const T*)iplImg.imageData;
	T* rdata=img.data();
	T* gdata=rdata+dstPlaneSize;
	T* bdata=gdata+dstPlaneSize;

	for(uint32_t y=0; y<img.height(); y++)
		{
		uint32_t offset=3*y*srcPaddedWidth;
		uint32_t offset2=y*img.paddedWidth();
		for(uint32_t x=0; x<img.width(); x++)
			{
			rdata[offset2]=data[offset++];
			gdata[offset2]=data[offset++];
			bdata[offset2++]=data[offset++];
			}
		}
	}

ImageBase* convertFromIpl(const IplImage& iplImg)
	{
	ImageBase::colorSpace_t colorSpace = (iplImg.nChannels == 3 ? ImageBase::CS_RGB : ImageBase::CS_GRAY);

	switch(iplImg.depth)
		{
		case IPL_DEPTH_8U:
			{
			Image<uint8_t>* img = new Image<uint8_t>(iplImg.width, iplImg.height, iplImg.nChannels, iplImg.width, true, colorSpace);
			if(colorSpace == ImageBase::CS_RGB)	{convertFromIplRGB_internal(iplImg, *img);}
			else									{convertFromIplGray_internal(iplImg, *img);}
			return img;
			}

		case IPL_DEPTH_8S:
			{
			Image<int8_t>* img = new Image<int8_t>(iplImg.width, iplImg.height, iplImg.nChannels, iplImg.width, true, colorSpace);
			if(colorSpace == ImageBase::CS_RGB)	{convertFromIplRGB_internal(iplImg, *img);}
			else									{convertFromIplGray_internal(iplImg, *img);}
			return img;
			}

		case IPL_DEPTH_16U:
			{
			Image<uint16_t>* img = new Image<uint16_t>(iplImg.width, iplImg.height, iplImg.nChannels, iplImg.width, true, colorSpace);
			if(colorSpace == ImageBase::CS_RGB)	{convertFromIplRGB_internal(iplImg, *img);}
			else									{convertFromIplGray_internal(iplImg, *img);}
			return img;
			}

		case IPL_DEPTH_16S:
			{
			Image<int16_t>* img = new Image<int16_t>(iplImg.width, iplImg.height, iplImg.nChannels, iplImg.width, true, colorSpace);
			if(colorSpace == ImageBase::CS_RGB)	{convertFromIplRGB_internal(iplImg, *img);}
			else									{convertFromIplGray_internal(iplImg, *img);}
			return img;
			}

		case IPL_DEPTH_32S:
			{
			Image<int32_t>* img = new Image<int32_t>(iplImg.width, iplImg.height, iplImg.nChannels, iplImg.width, true, colorSpace);
			if(colorSpace == ImageBase::CS_RGB)	{convertFromIplRGB_internal(iplImg, *img);}
			else									{convertFromIplGray_internal(iplImg, *img);}
			return img;
			}

		case IPL_DEPTH_32F:
			{
			Image<float>* img = new Image<float>(iplImg.width, iplImg.height, iplImg.nChannels, iplImg.width, true, colorSpace);
			if(colorSpace == ImageBase::CS_RGB)	{convertFromIplRGB_internal(iplImg, *img);}
			else									{convertFromIplGray_internal(iplImg, *img);}
			return img;
			}

		case IPL_DEPTH_64F:
			{
			Image<double>* img = new Image<double>(iplImg.width, iplImg.height, iplImg.nChannels, iplImg.width, true, colorSpace);
			if(colorSpace == ImageBase::CS_RGB)	{convertFromIplRGB_internal(iplImg, *img);}
			else									{convertFromIplGray_internal(iplImg, *img);}
			return img;
			}
		}

	return 0;
	}

}	// namespace vlr

#endif // HAVE_OPENCV
