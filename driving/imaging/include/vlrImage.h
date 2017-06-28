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


#ifndef VLRIMAGING_IMAGE_H_
#define VLRIMAGING_IMAGE_H_

#include <inttypes.h>
#include <algorithm>
#include <map>

#include <global.h>
#include <vlrImageTags.h>

namespace vlr {

class ImageBase
{
public:
		// Struct for (float) complex data compatible to Ipp32fc
	typedef struct {float re, im;} complex;

		// Struct for (double) complex data compatible to Ipp64fc
	typedef struct {double re, im;} dcomplex;

	typedef enum {BOMO_IGNORE=0, BOMO_CONSTANT, BOMO_REPLICATE, BOMO_REFLECT, BOMO_WRAP} borderMode_t;
	typedef enum {PAD_LEFT, PAD_SYM, PAD_RIGHT, PAD_TOP, PAD_BOTTOM, PAD_CUSTOM} padMode_t;

	typedef enum
		{
		CS_GRAY,    // Gray level
		CS_RGB,     // RGB
		CS_CMY,     // CMY
		CS_CMYK,    // CMYK
		CS_HSV,     // HSV
		CS_HLS,     // HLS
		CS_XYZ,     // XYZ
		CS_LUV,     // LUV
		CS_LAB,     // LAB
		CS_YCR,     // YCR
		CS_YCC,     // YCC
    CS_YUV,     // YUV
    CS_RGB_C    // RGB CHUNKY
		} colorSpace_t;

	typedef std::multimap<uint32_t, TagBase*>::const_iterator tagCIter;

protected:
	ImageBase(uint32_t width, uint32_t height,
			 uint32_t channels, uint32_t paddedWidth,
			 bool manageBuffer, colorSpace_t colorSpace) :
			 width_(width),
			 height_(height),
			 channels_(channels),
			 paddedWidth_(paddedWidth),
			 manageBuffer_(manageBuffer),
			 colorSpace_(colorSpace),
			 timeStamp_(driving_common::Time::current())
	{
	numElements_ = paddedWidth_*height_*channels_;
	}

public:
	virtual ~ImageBase() {}

public:
	inline uint32_t           width() const {return width_;}
	inline uint32_t           height() const {return height_;}
	inline uint32_t           channels() const {return channels_;}
	inline uint32_t           paddedWidth() const {return paddedWidth_;}
	inline bool			          manageBuffer() const {return manageBuffer_;}
	inline uint32_t           numElements() const {return numElements_;}
	inline colorSpace_t	      colorSpace() const {return colorSpace_;}
	inline double	 	          timeStamp() const {return timeStamp_;}
  inline void               setTimeStamp(double time_stamp) {timeStamp_=time_stamp;}
  inline const std::string& name() const {return name_;}
  inline void               setName(const std::string& name)  {name_ = name;}
  inline const std::string& typeName() const {return type_name_;}

  inline TagPoint* addPoint(const float x, const float y, const std::string& label,
                                const float r, const float g, const float b, const float a) {
    TagPoint* pt = new TagPoint(x, y, label, r, g, b, a);
    tags.insert(std::make_pair((uint32_t)tidPoint, pt));
    return pt;
  }

  inline TagPoint* addPoint(const float x, const float y, const std::string& label) {
    TagPoint* pt = new TagPoint(x, y, label);
    tags.insert(std::make_pair((uint32_t)tidPoint, pt));
    return pt;
  }

  inline TagPoint* addPoint(const float x, const float y) {
    TagPoint* pt = new TagPoint(x, y);
    tags.insert(std::make_pair((uint32_t)tidPoint, pt));
    return pt;
  }

	inline void points(std::pair<tagCIter, tagCIter>& range) {
		range=tags.equal_range(tidPoint);
  }

  inline TagLine* addLine(const float x0, const float y0, const float x1, const float y1,
                              const std::string& label, const float r, const float g,
                              const float b, const float a) {
    TagLine* ln = new TagLine(x0, y0, x1, y1, label, r, g, b, a);
    tags.insert(std::make_pair((uint32_t)tidLine, ln));
    return ln;
  }

  inline TagLine* addLine(const float x0, const float y0, const float x1, const float y1,
                              const std::string& label) {
    TagLine* ln = new TagLine(x0, y0, x1, y1, label);
    tags.insert(std::make_pair((uint32_t)tidLine, ln));
    return ln;
  }

  inline TagLine* addLine(const float x0, const float y0, const float x1, const float y1) {
    TagLine* ln = new TagLine(x0, y0, x1, y1);
    tags.insert(std::make_pair((uint32_t)tidLine, ln));
    return ln;
  }

  inline void lines(std::pair<tagCIter, tagCIter>& range) {
    range=tags.equal_range(tidLine);
  }

protected:
	uint32_t width_;
	uint32_t height_;
	uint32_t channels_;
	uint32_t paddedWidth_;
	uint32_t numElements_;
	bool manageBuffer_;
	colorSpace_t colorSpace_;
	double timeStamp_;

	std::string name_;
	std::string description_;

	std::multimap<uint32_t, TagBase*> tags;
  std::string type_name_;    // type name is used to make e.g. displays work with derived classes
};

template <class T> class Image : public ImageBase
{
public:
	Image(uint32_t width, uint32_t height, uint32_t channels, uint32_t paddedWidth,
           bool manageBuffer, colorSpace_t colorSpace);
  Image(uint32_t width, uint32_t height, uint32_t channels, colorSpace_t colorSpace);
  Image(uint32_t width, uint32_t height, uint32_t channels);
  Image(uint32_t width, uint32_t height);
  Image(const Image& img, bool manageBuffer=true, bool copyData=true, bool copyTags=true);

	Image& operator=(const Image& img);

	virtual ~Image();

		// inline functions
	inline uint32_t  elementSize() const {return elementSize_;}
  inline T*        data()        const {return data_;}
  inline const T*  const_data()  const {return (const T*)data_;}
	inline T*			   setData(T* newData, bool manageNewBuffer=false) {
		T* tmp=data_;
		manageBuffer_= manageNewBuffer;
		data_= newData;
		if(manageBuffer_ && tmp) {delete[] tmp; tmp=0;}

		return tmp;
		}

  inline bool sameDims(const Image& img) const {
    return (img.width() == width_ && img.height() == height_ &&
        img.channels() == channels_ && img.paddedWidth() == paddedWidth_);
    }

  template <class nT> operator Image<nT>() {
		Image<nT> res(width_, height_, channels_, paddedWidth_, true, colorSpace_);

		T* dataPtr=data_;
		nT* resdata = res.data();

		uint32_t alignGap = paddedWidth_ - width_;
printf("%s\n", __FUNCTION__);
		for(uint32_t c=0; c<channels_; c++)
			{
			for (uint32_t y=0; y<height_; y++)
				{
				for (uint32_t x=0; x<width_; x++)
					{
					*resdata=nT(*dataPtr);
					resdata++;
					dataPtr++;
					}

				resdata+=alignGap;
				dataPtr+=alignGap;
				}
			}

		return res;
		}

	void bounds(T& lowerBound, T& upperBound);
	void normalize(T newMin, T newMax);
	bool reformat(uint32_t newWidth, uint32_t newHeight, uint32_t newChannels,
										uint32_t newPaddedWidth, colorSpace_t newColorSpace);

	Image<T> operator + (const Image<T>& img) const;

	inline T& operator [] (const uint32_t pos) const {return data_[pos];}
	inline T& operator () (const uint32_t x, const uint32_t y) const {return data_[y*paddedWidth_+x];}
	inline T& operator () (const uint32_t x, const uint32_t y, const uint32_t c) const
							 {return data_[c*paddedWidth_*height_+y*paddedWidth_+x];}
	Image<T> operator () (uint32_t x, uint32_t y, uint32_t roiWidth, uint32_t roiHeight) const;

private:
	void create();

private:
	uint32_t elementSize_;
	T* data_;
	};

}	// namespace vlr

#endif // VLRIMAGING_IMAGE_H_
