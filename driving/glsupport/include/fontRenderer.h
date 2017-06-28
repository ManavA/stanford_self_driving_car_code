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


#ifndef FONTRENDERER_H_
#define FONTRENDERER_H_

#include <map>
#include <string>
#include <FTGL/ftgl.h>
#include <FTGL/FTGLPolygonFont.h>

namespace vlr {

class FontRenderer {
public:
	FontRenderer();
	~FontRenderer();

	bool addFont(const std::string& fileName);
	bool addFont(const std::string& fontName, const unsigned char* mem, size_t memSize);
	void removeFont(const std::string& fontName);

	int numFonts() {return font_map_.size();}
    void setFontSize(uint32_t size);

	void drawString2D(const std::string& text, float x, float y, const std::string& fontName);
	void drawString2D(const std::string& text, float x, float y);
	void drawString2D(const std::string& text, float x, float y, float font_size);
	void drawString3D(const std::string&, float x, float y, float z, const std::string& fontName);
	void drawString3D(const std::string&, float x, float y, float z);

private:
	std::map<std::string, FTFont*> font_map_;
	FTFont* default_font_;
	uint32_t size_;
};

} // namespace vlr

#endif // FONTRENDERER_H_
