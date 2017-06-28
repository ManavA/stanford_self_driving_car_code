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


#include <stdio.h>
#include <string.h>
#include <typeinfo>
#include <algorithm>
#include <map>

#include <vlrImage.h>

namespace vlr {

template <class T>
Image<T>::Image(uint32_t width, uint32_t height,
			 uint32_t channels, uint32_t paddedWidth, bool manageBuffer, colorSpace_t colorSpace) :
			 ImageBase(width, height, channels, paddedWidth, manageBuffer, colorSpace) {
	create();
	}

template <class T>
Image<T>::Image(uint32_t width, uint32_t height, uint32_t channels, colorSpace_t colorSpace) :
       ImageBase(width, height, channels, width, true, colorSpace) {
  create();
  }

template <class T>
Image<T>::Image(uint32_t width, uint32_t height, uint32_t channels) :
       ImageBase(width, height, channels, width, true, CS_GRAY) {
  create();
  }

template <class T>
Image<T>::Image(uint32_t width, uint32_t height) :
       ImageBase(width, height, 1, width, true, CS_GRAY) {
  create();
  }

template <class T>
void Image<T>::create() {
  elementSize_ = sizeof(T);
  if(manageBuffer_ && numElements_>0) {data_ = new T[numElements_];}
  type_name_=typeid(*this).name();
}

template <class T>
Image<T>::Image(const Image& img, bool ignoreManageBuffer, bool copyData, bool copyTags) :
			ImageBase(img.width_, img.height_, img.channels_, img.paddedWidth_, img.manageBuffer_, img.colorSpace_) {

  if(ignoreManageBuffer) {manageBuffer_=true;}  // create a buffer for the data regardless of source
	create();
	if(manageBuffer_ && numElements_>0) {
		if(copyData) {memcpy(data_, img.data_, numElements_*elementSize_);}
	}
	if(copyTags) {tags=img.tags;}
}

template <class T>
Image<T>& Image<T>::operator=(const Image<T>& img)
	{
	if(this == &img) {return *this;}

		// TODO: assigment operator always creates deep copy
	if(!sameDims(img) || !manageBuffer_)
//		{throw("assignment failed: images have different dimensions");}
		{
		T* tmp=0;

		if(numElements_>0)  {tmp = new T[numElements_];}
		if(data_ && manageBuffer_) {delete[] data_;}

		data_ = tmp;
		width_=img.width();
		height_=img.height();
		channels_=img.channels();
		paddedWidth_=img.paddedWidth();
		manageBuffer_=true;
		}

	if(numElements_>0)  {memcpy(data_, img.data_, numElements_*elementSize_);}

	tags=img.tags;
	colorSpace_=img.colorSpace();

	return *this;
	}

template <class T>
Image<T>::~Image()
	{
	if(data_ && manageBuffer_) {delete[] data_;}
	}


template <class T>
void Image<T>::bounds(T& lowerBound, T& upperBound)
	{
	T* dataPtr=data_;

	uint32_t alignGap = paddedWidth_ - width_;

	lowerBound = upperBound = *dataPtr;

	for(uint32_t c=0; c<channels_; c++)
		{
		for (uint32_t y=0; y<height_; y++)
			{
			for (uint32_t x=0; x<width_; x++)
				{
				if(*dataPtr < lowerBound) {lowerBound=*dataPtr;}
				else if(*dataPtr > upperBound) {upperBound=*dataPtr;}
				dataPtr++;
				}
			dataPtr+=alignGap;
			}
		}
	}

template <class T>
void Image<T>::normalize(T newMin, T newMax)
	{
	T imgMinVal, imgMaxVal, dFactor;

	if(!data_) {
	  throw("zero data pointer");
	}

  T* dataPtr=data_;

  uint32_t alignGap = paddedWidth_ - width_;

	bounds(imgMinVal, imgMaxVal);

	if (imgMaxVal == imgMinVal) {return;}

	dFactor = (newMax - newMin) / (imgMaxVal - imgMinVal);

	for(uint32_t c=0; c<channels_; c++)
		{
		for (uint32_t y=0; y<height_; y++)
			{
			for (uint32_t x=0; x<width_; x++)
				{
				*dataPtr=(*dataPtr-imgMinVal)*dFactor + newMin;
				dataPtr++;
				}

			dataPtr+=alignGap;
			}
		}
	}


template <class T>
Image<T> Image<T>::operator + (const Image<T>& img) const
	{
	if(!sameDims(img)) {throw("Different image dimensions.");}

	Image<T> res = Image<T>(*this);	// TODO: Implement tag class filtering...
	const T* data1 =data_;
	const T* data2 = const_cast<Image<T>*>(&img)->data();
	T* resdata = res.data();
	for(uint32_t i=0; i<numElements_; i++)
		{
		*resdata++= *data1++ + *data2++;
		}

	return res;
	}

template <class T>
bool Image<T>::reformat(uint32_t newWidth, uint32_t newHeight, uint32_t newChannels,
									uint32_t newPaddedWidth, colorSpace_t newColorSpace)

	{
	if(newColorSpace != CS_GRAY && newChannels<3) {return false;}

	colorSpace_ = newColorSpace;

	if(newWidth==width_ && newHeight==height_ && newChannels==channels_ && newPaddedWidth==paddedWidth_) {return true;}

	if(data_ && manageBuffer_) {delete[] data_; data_=NULL;}

	width_=newWidth;
	height_=newHeight;
	channels_=newChannels;
	paddedWidth_=newPaddedWidth;
	manageBuffer_=true;

	numElements_ = paddedWidth_*height_*channels_;

	if(numElements_ > 0) {data_ = new T[numElements_];}
	tags.clear();
	return true;
	}

template <class T>
Image<T> Image<T>::operator () (uint32_t x, uint32_t y, uint32_t roiWidth, uint32_t roiHeight) const
	{

	if(roiWidth == 0 || roiHeight == 0 || x+roiWidth > width_ || y+roiHeight > height_)
		{throw VLRException("illegal roi dimensions.");}

	Image<T> res(roiWidth, roiHeight, channels_, paddedWidth_, false, colorSpace_);
	res.data_=&data_[y*paddedWidth_+x];

	return res;
	}

// instantiations for supported data types
template class Image<uint8_t>;
template class Image<int8_t>;
template class Image<char>;  // should be int8_t ?!?
template class Image<uint16_t>;
template class Image<int16_t>;
template class Image<uint32_t>;
template class Image<int32_t>;
template class Image<float>;
template class Image<double>;
}	// namespace vlr
