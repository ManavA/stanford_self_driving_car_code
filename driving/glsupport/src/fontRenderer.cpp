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


#include <GL/gl.h>
#include <vlrException.h>

#include <helvetica.h>
#include <fontRenderer.h>

namespace vlr {

FontRenderer::FontRenderer() : default_font_(0), size_(10) {
  if (!addFont("helvetica", helvetica_resource_data, helvetica_resource_size)) {
    throw VLRException("Cannot add default font resource.");
  }
  default_font_ = (*font_map_.begin()).second;
//  default_font_->UseDisplayList(false);
}

FontRenderer::~FontRenderer() {
  std::map<std::string, FTFont*>::const_iterator fit, fit_end;

  for (fit = font_map_.begin(), fit_end = font_map_.end(); fit != fit_end; ++fit) {
    delete (*fit).second;
  }
}

bool FontRenderer::addFont(const std::string& fileName) {
  FTFont* font = NULL;

  try {
//	font = new FTGLPixmapFont(fileName.c_str());
    font = new FTGLPolygonFont(fileName.c_str());
  }
  catch (...) {
    return false;
  }

  font->FaceSize(size_);

  //TODO: Extract fontName from file / fileName
  std::pair<std::map<std::string, FTFont*>::iterator, bool> pair = font_map_.insert(
      std::make_pair(fileName.c_str(), font));

  if (pair.second) {
    return true;
  }

  delete font;

  return false;
}

bool FontRenderer::addFont(const std::string& fontName, const unsigned char* mem, size_t memSize) {
  FTFont* font = NULL;

  if (!mem) {
    return false;
  }

  try {
//	font = new FTGLPixmapFont(mem, memSize);
    font = new FTGLPolygonFont(mem, memSize);
  }
  catch (...) {
    return false;
  }

  font->FaceSize(size_);

  std::pair<std::map<std::string, FTFont*>::iterator, bool> pair = font_map_.insert(std::make_pair(fontName, font));

  if (pair.second) {
    return true;
  }

  delete font;

  return false;
}

void FontRenderer::removeFont(const std::string& fontName) {
  std::map<std::string, FTFont*>::iterator fit = font_map_.find(fontName);

  if (fit == font_map_.end()) {
    return;
  }

  FTFont* font = (*fit).second;

  font_map_.erase(fit);

  delete font;
}

void FontRenderer::drawString2D(const std::string& text, float x, float y, const std::string& fontName) {
  std::map<std::string, FTFont*>::iterator fit = font_map_.find(fontName);

  if (fit == font_map_.end()) {
    return;
  }

  FTFont* font = (*fit).second;

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(x, y, 0.);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  font->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString2D(const std::string& text, float x, float y) {
  return; // TODO: Fix performance issue
  glPushMatrix();
  glTranslatef(x, y, 0.);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  default_font_->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString2D(const std::string& text, float x, float y, float font_size) {
//  return; // TODO: Fix performance issue
  glPushMatrix();
  glTranslatef(x, y, 0.);

  glScalef(0.1*font_size, 0.1*font_size, 1); // compatibility scale...

  default_font_->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString3D(const std::string& text, float x, float y, float z, const std::string& fontName) {
  std::map<std::string, FTFont*>::iterator fit = font_map_.find(fontName);

  if (fit == font_map_.end()) {
    return;
  }

  FTFont* font = (*fit).second;

  glPushMatrix();
  glTranslatef(x, y, z);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  font->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString3D(const std::string& text, float x, float y, float z) {
  glPushMatrix();
  glTranslatef(x, y, z);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  default_font_->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::setFontSize(uint32_t size) {
  default_font_->FaceSize(size_);
  size_=size;
}

} // namespace vlr
