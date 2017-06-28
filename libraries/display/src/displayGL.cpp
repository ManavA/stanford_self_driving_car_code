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

#include <displayGL.h>

#include <typeinfo>

namespace vlr {

DisplayGL::DisplayGL() :
  GLWidget(QGLFormat(QGL::DoubleBuffer), NULL) {
  create(MODE_2D, DEFAULT_FPS);
}

DisplayGL::DisplayGL(QWidget* parent, displayMode_t mode, double frame_rate, QGLFormat gl_format) :
  GLWidget(gl_format, parent) {
  create(mode, frame_rate);
}

void DisplayGL::create(displayMode_t mode, double frame_rate) {

  texType = GL_TEXTURE_RECTANGLE_ARB;
  imgBuf = NULL;
  texBuf = NULL;
  mode_ = mode;
  refreshTimeMS = 1000 * std::max(0.00001, frame_rate);
  showGridXY = true;
  showGridXZ = true;
  showGridYZ = true;
  bufColorFormat = GL_LUMINANCE;
  useColorMap = false;
  useTexture = true;
  normalize_data_ = true;
  gamma = 1.0;
  userKeyPressFunc = NULL; // TODO: remove those and make originals virtual..or needed for compatibility?!?
  userMousePressFunc = NULL;
  userMouseReleaseFunc = NULL;
  userMouseMoveFunc = NULL;
  heightScale_ = 1.0f;
  scale_ = 1.0f;
  current_slice_ = 0;
  slice_offset_ = 0;
  create_snapshot_ = false;
  snapshot_buf_ = NULL;
  color_map_red_ = gamma_map_red_;
  color_map_green_ = gamma_map_green_;
  color_map_blue_ = gamma_map_blue_;
  color_map_alpha_ = gamma_map_alpha_;

  setInitialCameraPos(-90.0, 89.99, 500.0, 0, 0, 0); // TODO: make inititial distance dependend on image
  setCameraParams(0.01, 0.3, 0.001, 0.009, 60, 0.4, 200000);
  createGammaMap<float> (256, gamma, gamma_map_red_);
  memcpy(gamma_map_green_, gamma_map_red_, 256 * sizeof(float));
  memcpy(gamma_map_blue_, gamma_map_red_, 256 * sizeof(float));
  for (unsigned short i = 0; i < 256; i++) {
    gamma_map_alpha_[i] = 1.0f;
  }

  //    float phistep = 360.0/256.0;
  //    float hlsMap[3*256];
  //    float phi = 0;
  //    for(uint32_t i=0; i<3*256; i++)
  //      {
  //      hlsMap[i] = (float)i/255.0; i++;///3;i++;//rInt(phi);
  //      hlsMap[i++] = 0.5;
  //      hlsMap[i] = 1;
  //  //    phi+=phistep;
  //      }
  //    for(uint32_t i=0; i<256; i++)
  //      {
  //      hlsMap[3*i] = (float)i/255.0;
  //      hlsMap[3*i+1] = 0.5;
  //      hlsMap[3*i+2] = 1;
  //  //    phi+=phistep;
  //      }
  //
  //    IppiSize roi;
  //    roi.width=256;roi.height=1;
  //
  //    float tmap [3*256];
  //    if (ippiHLSToRGB_32f_C3R(hlsMap, 3 * 256, (float*) tmap, 3 * 256, roi) != ippStsNoErr) {
  //    std::cout << "Fehler bei der Farbraumkonvertierung.\n";
  //  }
  //
  //  printf(" = {\n");
  //  for (uint32_t j = 0; j < 8; j++) {
  //    for (uint32_t i = 0; i < 32; i++) {
  //      printf("%f, ", tmap[3*(j*32+i)]);
  //    }
  //    printf("\n");
  //  }
  //  printf("}\n");
  //
  //  printf(" = {\n");
  //  for (uint32_t j = 0; j < 8; j++) {
  //    for (uint32_t i = 0; i < 32; i++) {
  //      printf("%f, ", tmap[3*(j*32+i)+1]);
  //    }
  //    printf("\n");
  //  }
  //  printf("}\n");
  //
  //  printf(" = {\n");
  //  for (uint32_t j = 0; j < 8; j++) {
  //    for (uint32_t i = 0; i < 32; i++) {
  //      printf("%f, ", tmap[3*(j*32+i)+2]);
  //    }
  //    printf("\n");
  //  }
  //  printf("}\n");
  //  exit(0);
  setFocusPolicy(Qt::StrongFocus);
}

DisplayGL::~DisplayGL() {
}

void DisplayGL::resizeEvent(QResizeEvent* event) {
  // correct gl viewport and redraw
  resizeGL(event->size().width(), event->size().height());
}

void DisplayGL::mousePressEvent(QMouseEvent* event) {
  switch (event->modifiers()) {
  case Qt::ControlModifier: // selected object is affected
    if (userMousePressFunc) {
      uint8_t buttons = 0;
      if (event->buttons() & Qt::LeftButton) {
        buttons |= LEFT_BUTTON;
      }
      if (event->buttons() & Qt::MidButton) {
        buttons |= MIDDLE_BUTTON;
      }
      if (event->buttons() & Qt::RightButton) {
        buttons |= RIGHT_BUTTON;
      }
      userMousePressFunc(event->x(), event->y(), buttons);
      requestRedraw();
    }
    else {
      // use ctrl + left as replacement for right; useful for Mac Trackpad
      QMouseEvent mouse_event(event->type(), QPoint(event->x(), event->y()), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
      GLWidget::mousePressEvent(&mouse_event);
    }
    break;

  default: // let base class handle other cases
    GLWidget::mousePressEvent(event);
    break;
  }
}

void DisplayGL::mouseReleaseEvent(QMouseEvent* event) {
  switch (event->modifiers()) {
  case Qt::ControlModifier: // selected object is affected
    if (userMouseReleaseFunc) {
      uint8_t buttons = 0;
      if (event->buttons() & Qt::LeftButton) {
        buttons |= LEFT_BUTTON;
      }
      if (event->buttons() & Qt::MidButton) {
        buttons |= MIDDLE_BUTTON;
      }
      if (event->buttons() & Qt::RightButton) {
        buttons |= RIGHT_BUTTON;
      }
      userMouseReleaseFunc(event->x(), event->y(), buttons);
      requestRedraw();
    }
    else {
      // use ctrl + left as replacement for right; useful for Mac Trackpad
      QMouseEvent mouse_event(event->type(), QPoint(event->x(), event->y()), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
      GLWidget::mouseReleaseEvent(&mouse_event);
    }
    break;

  default: // let base class handle other cases
    GLWidget::mouseReleaseEvent(event);
    break;
  }
}

void DisplayGL::mouseMoveEvent(QMouseEvent *event) {
  //double x2, y2, utm_x, utm_y;
  //pickPoint(event->x(), event->y(), &x2, &y2);
  //utm_x = x2 + gui->rndf_center.x;
  //utm_y = y2 + gui->rndf_center.y;
  //gui->last_utm_x = utm_x;
  //gui->last_utm_y = utm_y;

  switch (event->modifiers()) {
  case Qt::ControlModifier: // selected object is affected
    if (userMouseMoveFunc) {
      uint8_t buttons = 0;
      if (event->buttons() & Qt::LeftButton) {
        buttons |= LEFT_BUTTON;
      }
      if (event->buttons() & Qt::MidButton) {
        buttons |= MIDDLE_BUTTON;
      }
      if (event->buttons() & Qt::RightButton) {
        buttons |= RIGHT_BUTTON;
      }
      userMouseMoveFunc(event->x(), event->y(), buttons);
      requestRedraw();
    }
    else {
      // use ctrl + left as replacement for right; useful for Mac Trackpad
      QMouseEvent mouse_event(event->type(), QPoint(event->x(), event->y()), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
      GLWidget::mouseMoveEvent(&mouse_event);
    }
    break;

  case Qt::NoModifier:
    //		gui->last_mouse_x = event->x();
    //		gui->last_mouse_y = event->y();

  default: // let base class handle other cases
    GLWidget::mouseMoveEvent(event);
    break;
  }

  //gui->last_move_utm_x = utm_x;
  //gui->last_move_utm_y = utm_y;
}

// Parses keyboard commands
void DisplayGL::keyPressEvent(QKeyEvent* event) {
  //double x2, y2, utm_x, utm_y;

  uint32_t key = (uint32_t) (*(event->text().toAscii().constData()));

  //	std::cout << "keyboard: "<< key << " (#"<< (int) key <<") ";

  switch (key) {
  case '2':
    setDisplayMode(MODE_2D);
    std::cout << "2d mode" << std::endl;
    break;

  case '3':
    setDisplayMode(MODE_3D);
    std::cout << "3d mode" << std::endl;
    break;

  case 'g':
    gamma -= .1;
    gamma = std::max(0.01, gamma);
    createGammaMap<float> (256, gamma, gamma_map_red_);
    memcpy(gamma_map_green_, gamma_map_red_, 256 * sizeof(float));
    memcpy(gamma_map_blue_, gamma_map_red_, 256 * sizeof(float));
    color_map_red_ = gamma_map_red_;
    color_map_green_ = gamma_map_green_;
    color_map_blue_ = gamma_map_blue_;
    color_map_alpha_ = gamma_map_alpha_;
    glPixelMapfv(GL_PIXEL_MAP_R_TO_R, 256, color_map_red_);
    glPixelMapfv(GL_PIXEL_MAP_G_TO_G, 256, color_map_green_);
    glPixelMapfv(GL_PIXEL_MAP_B_TO_B, 256, color_map_blue_);
    glPixelMapfv(GL_PIXEL_MAP_A_TO_A, 256, color_map_alpha_);
    useColorMap = false;
    std::cout << "decrease gamma => " << gamma << std::endl;
    break;

  case 'G':
    gamma += .1;
    createGammaMap<float> (256, gamma, gamma_map_red_);
    memcpy(gamma_map_green_, gamma_map_red_, 256 * sizeof(float));
    memcpy(gamma_map_blue_, gamma_map_red_, 256 * sizeof(float));
    color_map_red_ = gamma_map_red_;
    color_map_green_ = gamma_map_green_;
    color_map_blue_ = gamma_map_blue_;
    color_map_alpha_ = gamma_map_alpha_;
    glPixelMapfv(GL_PIXEL_MAP_R_TO_R, 256, color_map_red_);
    glPixelMapfv(GL_PIXEL_MAP_G_TO_G, 256, color_map_green_);
    glPixelMapfv(GL_PIXEL_MAP_B_TO_B, 256, color_map_blue_);
    glPixelMapfv(GL_PIXEL_MAP_A_TO_A, 256, color_map_alpha_);
    useColorMap = false;
    std::cout << "increase gamma => " << gamma << std::endl;
    break;

  case 'c':
    if (!useColorMap) {
      color_map_red_ = cmap_rb1_red_;
      color_map_green_ = cmap_rb1_green_;
      color_map_blue_ = cmap_rb1_blue_;
      color_map_alpha_ = gamma_map_alpha_;
      glPixelMapfv(GL_PIXEL_MAP_R_TO_R, 256, color_map_red_);
      glPixelMapfv(GL_PIXEL_MAP_G_TO_G, 256, color_map_green_);
      glPixelMapfv(GL_PIXEL_MAP_B_TO_B, 256, color_map_blue_);
      glPixelMapfv(GL_PIXEL_MAP_A_TO_A, 256, color_map_alpha_);
      useColorMap = true;
      std::cout << "enabled color map." << std::endl;
    }
    else {
      color_map_red_ = gamma_map_red_;
      color_map_green_ = gamma_map_green_;
      color_map_blue_ = gamma_map_blue_;
      color_map_alpha_ = gamma_map_alpha_;
      glPixelMapfv(GL_PIXEL_MAP_R_TO_R, 256, color_map_red_);
      glPixelMapfv(GL_PIXEL_MAP_G_TO_G, 256, color_map_green_);
      glPixelMapfv(GL_PIXEL_MAP_B_TO_B, 256, color_map_blue_);
      glPixelMapfv(GL_PIXEL_MAP_A_TO_A, 256, color_map_alpha_);
      useColorMap = false;
      std::cout << "disabled color map." << std::endl;
    }
    break;

  case '+': {
    scale_ *= 1.5;
    std::cout << "scale factor => " << scale_ * 100 << "\%" << std::endl;
    QResizeEvent* re = new QResizeEvent(QSize(width() * scale_, height() * scale_), size());
    resizeEvent(re);
  }
    break;

  case 'l': {
    initLights();
    std::cout << "lighting enabled" << std::endl;
  }
    break;

  case 't': {
    if(useTexture) {
      useTexture=false;
      std::cout << "texture disabled" << std::endl;
    }
    else {
      useTexture=true;
      std::cout << "texture enabled" << std::endl;
    }
  }
    break;

  case '-': {
    scale_ /= 1.5;
    scale_ = std::max(scale_, .001f);
    std::cout << "scale factor => " << scale_ * 100 << "\%" << std::endl;
    QResizeEvent* re = new QResizeEvent(QSize(width() * scale_, height() * scale_), size());
    resizeEvent(re);
  }
    break;

  case '>': {
    uint32_t channel_stride = (imgBuf->colorSpace() == ImageBase::CS_RGB ? 3 : 1);
    current_slice_ = std::min(current_slice_ + 1, imgBuf->channels() / channel_stride - 1);
    slice_offset_ = width() * height() * channel_stride * current_slice_;

    std::cout << "displaying slice " << current_slice_ << std::endl;
  }
    break;

  case '<': {
    if (current_slice_ == 0) {
      break;
    }
    uint32_t channel_stride = (imgBuf->colorSpace() == ImageBase::CS_RGB ? 3 : 1);
    current_slice_--;
    slice_offset_ = width() * height() * channel_stride * current_slice_;

    std::cout << "displaying slice " << current_slice_ << std::endl;
  }
    break;

  default:
    //			std::cout << "(no command)"<< std::endl;
    break;
  }

  if (userKeyPressFunc) {
    userKeyPressFunc(key);
  }

  requestRedraw();
}

void DisplayGL::initLights() {
  GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat light_diffuse[] = { .2, .2, .2, 1 };
  GLfloat light_specular[] = { .8, .8, .8, 1 };
  GLfloat light_position[] = { 0.0, 100.0, 30.0, 1.0 };

  GLfloat mat_diffuse[] = { 0.2, 0.2, 0.2, 1.0 };
  GLfloat mat_specular[] = { 0.8, 0.8, 0.8, 1.0 };
  GLfloat mat_emission[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat mat_shininess[] = { 50.0 };

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable( GL_LIGHT0);
  glEnable( GL_LIGHTING);

  glEnable( GL_COLOR_MATERIAL);
  glShadeModel( GL_SMOOTH);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
  glEnable( GL_NORMALIZE);
  glEnable( GL_AUTO_NORMAL);
}

void DisplayGL::initializeGL(void) {
  glEnable( GL_DEPTH_TEST);
  //	initLights();

  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Really Nice Perspective Calculations

  glGenTextures(1, &imgTexture);
  glBindTexture(texType, imgTexture);
  glTexParameteri(texType, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  //	glMatrixMode(GL_PROJECTION);
  //	glLoadIdentity();
  //	glMatrixMode(GL_MODELVIEW);
  //	glLoadIdentity();
  //	glOrtho(0., width(), 0., height(), -1, 1);
  //	glViewport(0, 0, width(), height());

  //	glClearColor(0.0,0.0,0.0,0.0);

  glPixelMapfv(GL_PIXEL_MAP_R_TO_R, 256, color_map_red_);
  glPixelMapfv(GL_PIXEL_MAP_G_TO_G, 256, color_map_green_);
  glPixelMapfv(GL_PIXEL_MAP_B_TO_B, 256, color_map_blue_);
  glPixelMapfv(GL_PIXEL_MAP_A_TO_A, 256, color_map_alpha_);
  glPixelTransferi(GL_MAP_COLOR, GL_TRUE);

  glClearColor(0.13, 0.17, 0.32, 1.0);

  connect(&timer, SIGNAL(timeout()), this, SLOT(redraw(void)));

  timer.start(DISPLAY_REFRESH_DELAY_MS);
  gl_initialized_ = true;
}

void DisplayGL::paintGL() {
  boost::unique_lock<boost::mutex> lock(mutex_);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (imgBuf) {
    if (imgBuf->width() == 1 || imgBuf->height() == 1) {
      internalPaint1d();
    }
    else if (mode_ == MODE_2D) {
      internalPaint2d();
    }
    else if (mode_ == MODE_3D) {
      internalPaint3d();
    }
  }
  else {
    //    printf("No image buffer :-( (0x%x)\n",(uint32_t)imgBuf);
  }
  if (create_snapshot_) {
    glReadPixels(0, 0, width(), height(), GL_RGB, GL_UNSIGNED_BYTE, snapshot_buf_);
    create_snapshot_ = false;
  }

  refresh_required = false;
}

bool DisplayGL::updateImage(ImageBase& img) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  return updateBuffer(img, &imgBuf, bufColorFormat);
}

bool DisplayGL::updateTexture(ImageBase& img) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  return updateBuffer(img, &texBuf, texColorFormat);
}

void DisplayGL::removeTexture() {
  boost::unique_lock<boost::mutex> lock(mutex_);
  if (texBuf) {
    delete texBuf;
    texBuf = NULL;
    requestRedraw();
  }
}

bool DisplayGL::updateBuffer(ImageBase& img, ImageBase** dest, int& destColorFormat) {

  //  printf("id for img of type char (0x%x): %s\n", (uint32_t)&typeid(Image<char>), typeid(Image<char>).name());
  //  printf("id for img of type uint8_t (0x%x): %s\n", (uint32_t)&typeid(Image<uint8_t>), typeid(Image<uint8_t>).name());
  //  printf("id for img of type short (0x%x): %s\n", (uint32_t)&typeid(Image<short>), typeid(Image<short>).name());
  //  printf("id for img of type unsigned short (0x%x): %s\n", (uint32_t)&typeid(Image<unsigned short>), typeid(Image<unsigned short>).name());
  //  printf("id for img of type int (0x%x): %s\n", (uint32_t)&typeid(Image<int>), typeid(Image<int>).name());
  //  printf("id for img of type uint32_t (0x%x): %s\n", (uint32_t)&typeid(Image<uint32_t>), typeid(Image<uint32_t>).name());
  //  printf("id for img of type float (0x%x): %s\n", (uint32_t)&typeid(Image<float>), typeid(Image<float>).name());
  //  printf("id for img of type double (0x%x): %s\n", (uint32_t)&typeid(Image<double>), typeid(Image<double>).name());
  //  printf("id of lovely image (0x%x): %s\n", (uint32_t)&typeid(img), img.typeName().c_str());

  try {
    if (img.typeName() == typeid(Image<uint8_t> ).name()) {
      makeImageBuffer(*static_cast<Image<uint8_t>*> (&img), (Image<uint8_t>**) (dest), destColorFormat);
      data_type_ = TYPE_UCHAR;
    }
    else if (img.typeName() == typeid(Image<char> ).name()) {
      makeImageBuffer(*static_cast<Image<char>*> (&img), (Image<char>**) (dest), destColorFormat);
      data_type_ = TYPE_CHAR;
    }
    else if (img.typeName() == typeid(Image<unsigned short> ).name()) {
      makeImageBuffer(*static_cast<Image<unsigned short>*> (&img), (Image<unsigned short>**) (dest), destColorFormat);
      data_type_ = TYPE_USHORT;
    }
    else if (img.typeName() == typeid(Image<short> ).name()) {
      makeImageBuffer(*static_cast<Image<short>*> (&img), (Image<short>**) (dest), destColorFormat);
      data_type_ = TYPE_SHORT;
    }
    else if (img.typeName() == typeid(Image<uint32_t> ).name()) {
      makeImageBuffer(*static_cast<Image<uint32_t>*> (&img), (Image<uint32_t>**) (dest), destColorFormat);
      data_type_ = TYPE_UINT;
    }
    else if (img.typeName() == typeid(Image<int> ).name()) {
      makeImageBuffer(*static_cast<Image<int>*> (&img), (Image<int>**) (dest), destColorFormat);
      data_type_ = TYPE_INT;
    }
    else if (img.typeName() == typeid(Image<float> ).name()) {
      makeImageBuffer(*static_cast<Image<float>*> (&img), (Image<float>**) (dest), destColorFormat);
      static_cast<Image<float>*> (*dest)->normalize(0, 1);
      data_type_ = TYPE_FLOAT;
    }
    else if (img.typeName() == typeid(Image<double> ).name()) {
      Image<float> timg(*static_cast<Image<double>*> (&img));
      makeImageBuffer(timg, (Image<float>**) (dest), destColorFormat);
      static_cast<Image<float>*> (*dest)->normalize(0, 1);
      //      makeImageBuffer(*dynamic_cast<Image<double>*> (&img), (Image<double>**) (dest), destColorFormat);
      //      dynamic_cast<Image<double>*>(imgBuf)->normalize(0,1);
      data_type_ = TYPE_DOUBLE;
    }
    else {
      *dest = NULL;
      return false;
    }
  }
  catch (...) {
    *dest = NULL;
    return false;
  }

  requestRedraw();
  return true;
}

template<class T>
bool DisplayGL::makeImageBuffer(Image<T>& img, Image<T>** dest, int& destColorFormat) {
  Image<T>* res;

  try {
    res = new Image<T> (img, true, false, true);
  }
  catch (...) {
    return false;
  }
  switch (img.colorSpace()) {
  case ImageBase::CS_RGB: {
    cpReorganize<T, COLORORG_RGB> reorg;
    reorg.planar2Chunky(img, res->data(), res->paddedWidth());
    destColorFormat = GL_RGB;
  }
    break;

  case ImageBase::CS_RGB_C: {
    memcpy(res->data(), img.data(), res->numElements() * sizeof(T));
    destColorFormat = GL_RGB;
  }
    break;

  case ImageBase::CS_GRAY:
    memcpy(res->data(), img.data(), res->numElements() * sizeof(T));
    destColorFormat = GL_LUMINANCE;
    break;

  default:
    std::cout << "Display lib currently only supports RGB and gray images.\n";
    if (*dest) {
      delete *dest;
      *dest = NULL;
    }
    return false;
  }

  if (*dest) {
    delete *dest;
    *dest = NULL;
  }
  *dest = res;
  // calculate min and max value if data should be normalized or is one dimensional
  if ((normalize_data_ || (*dest)->width() == 1 || (*dest)->height() == 1) && *dest == imgBuf) {
    T minval, maxval;
    img.bounds(minval, maxval);
    minval_ = (double) minval;
    maxval_ = (double) maxval;
    if ((*dest)->width() == 1 || (*dest)->height() == 1) {
      if (maxval_ - minval_ != 0) {
        heightScale_ = std::max(img.width(), img.height()) / (maxval_ - minval_);
      }
    }
  }

  return true;
}

bool DisplayGL::snapshot(Image<uint8_t>& res) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  try {
    snapshot_buf_ = new uint8_t[width() * height() * 3 * sizeof(uint8_t)];
  }
  catch (...) {
    return false;
  }

  if (!res.reformat(width(), height(), 3, width(), ImageBase::CS_RGB)) {
    return false;
  }
  create_snapshot_ = true;
  requestRedraw();
  lock.release()->unlock();
  while (create_snapshot_) {
    usleep(10000);
  }

  cpReorganize<uint8_t, COLORORG_RGB> reorg;
  reorg.chunky2Planar(snapshot_buf_, width(), res);

  delete[] snapshot_buf_;
  snapshot_buf_ = NULL;

  return true;
}

template<class T>
bool DisplayGL::createGammaMap(uint32_t mapSize, double gamma, T* map) {
  if (!map || gamma == 0.0) {
    return false;
  }

  double base = 1.0 / (mapSize - 1.0);

  for (uint32_t i = 0; i < mapSize; i++) {
    map[i] = (T) (pow(i * base, 1.0 / gamma));
  }

  return true;
}

const float DisplayGL::cmap_rb1_red_[256] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 0.988235, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823530, 0.800000, 0.776470, 0.752941, 0.729412, 0.705882,
    0.682353, 0.658824, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647, 0.494118, 0.470588, 0.447059, 0.423529, 0.400000, 0.376470, 0.352941,
    0.329412, 0.305882, 0.282353, 0.258823, 0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094118, 0.070588, 0.047059, 0.023529, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.023530, 0.047059, 0.070588, 0.094118, 0.117647,
    0.141176, 0.164706, 0.188235, 0.211765, 0.235295, 0.258824, 0.282353, 0.305882, 0.329412, 0.352941, 0.376470, 0.400000, 0.423530, 0.447059, 0.470588,
    0.494118, 0.517647, 0.541176, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412, 0.752941, 0.776470, 0.800000, 0.823530,
    0.847059, 0.870588, 0.894118, 0.917647, 0.941176, 0.964706, 0.988236, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, };

const float DisplayGL::cmap_rb1_green_[256] = { 0.000000, 0.023529, 0.047059, 0.070588, 0.094118, 0.117647, 0.141176, 0.164706, 0.188235, 0.211765, 0.235294,
    0.258824, 0.282353, 0.305882, 0.329412, 0.352941, 0.376471, 0.400000, 0.423529, 0.447059, 0.470588, 0.494118, 0.517647, 0.541176, 0.564706, 0.588235,
    0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412, 0.752941, 0.776471, 0.800000, 0.823529, 0.847059, 0.870588, 0.894118, 0.917647, 0.941177,
    0.964706, 0.988235, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.988235, 0.964706, 0.941176,
    0.917647, 0.894118, 0.870588, 0.847059, 0.823529, 0.800000, 0.776470, 0.752941, 0.729412, 0.705882, 0.682353, 0.658823, 0.635294, 0.611765, 0.588235,
    0.564706, 0.541176, 0.517647, 0.494117, 0.470588, 0.447059, 0.423529, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258823, 0.235294,
    0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094117, 0.070588, 0.047059, 0.023529, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, };

const float DisplayGL::cmap_rb1_blue_[256] = { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.023529, 0.047059, 0.070588, 0.094118, 0.117647, 0.141177, 0.164706, 0.188235, 0.211765, 0.235294, 0.258824, 0.282353, 0.305883, 0.329412, 0.352941,
    0.376471, 0.400000, 0.423530, 0.447059, 0.470588, 0.494118, 0.517647, 0.541177, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882,
    0.729412, 0.752941, 0.776471, 0.800000, 0.823529, 0.847059, 0.870588, 0.894118, 0.917647, 0.941176, 0.964706, 0.988235, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.988236, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823530,
    0.800000, 0.776470, 0.752941, 0.729412, 0.705882, 0.682353, 0.658824, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647, 0.494118, 0.470588,
    0.447059, 0.423530, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258824, 0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647,
    0.094118, 0.070588, 0.047059, 0.023530, 0.000000, };

//const float DisplayGL::cmap_bone_red_[64] = {
//0.0000, 0.0139, 0.0278, 0.0417, 0.0556, 0.0694, 0.0833, 0.0972, 0.1111, 0.1250, 0.1389, 0.1528, 0.1667, 0.1806, 0.1944, 0.2083,
//0.2222, 0.2361, 0.2500, 0.2639, 0.2778, 0.2917, 0.3056, 0.3194, 0.3333, 0.3472, 0.3611, 0.3750, 0.3889, 0.4028, 0.4167, 0.4306,
//0.4444, 0.4583, 0.4722, 0.4861, 0.5000, 0.5139, 0.5278, 0.5417, 0.5556, 0.5694, 0.5833, 0.5972, 0.6111, 0.6250, 0.6389, 0.6528,
//0.6745, 0.6962, 0.7179, 0.7396, 0.7613, 0.7830, 0.8047, 0.8264, 0.8481, 0.8698, 0.8915, 0.9132, 0.9349, 0.9566, 0.9783, 1.0000
//};
//
//
//const float DisplayGL::cmap_bone_green_[64] = {
//0.0000, 0.0139, 0.0278, 0.0417, 0.0556, 0.0694, 0.0833, 0.0972, 0.1111, 0.1250, 0.1389, 0.1528, 0.1667, 0.1806, 0.1944, 0.2083,
//0.2222, 0.2361, 0.2500, 0.2639, 0.2778, 0.2917, 0.3056, 0.3194, 0.3385, 0.3576, 0.3767, 0.3958, 0.4149, 0.4340, 0.4531, 0.4722,
//0.4913, 0.5104, 0.5295, 0.5486, 0.5677, 0.5868, 0.6059, 0.6250, 0.6441, 0.6632, 0.6823, 0.7014, 0.7205, 0.7396, 0.7587, 0.7778,
//0.7917, 0.8056, 0.8194, 0.8333, 0.8472, 0.8611, 0.8750, 0.8889, 0.9028, 0.9167, 0.9306, 0.9444, 0.9583, 0.9722, 0.9861, 1.0000
//};
//
//
//const float DisplayGL::cmap_bone_blue_[64] = {
//0.0052, 0.0243, 0.0434, 0.0625, 0.0816, 0.1007, 0.1198, 0.1389, 0.1580, 0.1771, 0.1962, 0.2153, 0.2344, 0.2535, 0.2726, 0.2917,
//0.3108, 0.3299, 0.3490, 0.3681, 0.3872, 0.4062, 0.4253, 0.4444, 0.4583, 0.4722, 0.4861, 0.5000, 0.5139, 0.5278, 0.5417, 0.5556,
//0.5694, 0.5833, 0.5972, 0.6111, 0.6250, 0.6389, 0.6528, 0.6667, 0.6806, 0.6944, 0.7083, 0.7222, 0.7361, 0.7500, 0.7639, 0.7778,
//0.7917, 0.8056, 0.8194, 0.8333, 0.8472, 0.8611, 0.8750, 0.8889, 0.9028, 0.9167, 0.9306, 0.9444, 0.9583, 0.9722, 0.9861, 1.0000
//};

} // namespace vlr
