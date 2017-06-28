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


#include <vehiclemodels.h>

namespace vlr {

//TO DO: add passat model as vehicle 0

////draw a vehicle model
//void dgc_draw_model(VehicleModelGL& model) {
//  model.draw();
//}

//vehicle model constructor
VehicleModelsGL::VehicleModelsGL() {
  models_.clear();
  models_.push_back(new VehicleStickeredPassatModel);
  models_.push_back(new VehicleEliseModel);
  models_.push_back(new VehicleHummerModel);
  models_.push_back(new VehiclePorscheModel);
  models_.push_back(new VehicleLamborghiniModel);
}

//vehicle model destructor
VehicleModelsGL::~VehicleModelsGL() {
  for (int i = 0; i < (int) models_.size(); i++)
    delete models_[i];
}

//initialize all models
void VehicleModelsGL::init() {
  for (int i = 0; i < (int) models_.size(); i++)
    models_[i]->Init();
}

void VehicleModelsGL::draw(VehicleModelId_t model) {
  //printf("drawing model %i\n", model);
  if ((int)model - 1 >= (int) models_.size() || (int)model < 1) {model = PASSAT_MODEL_ID;}
  models_[model - 1]->draw();
}

const std::string VehicleModelsGL::model_names[NUM_VEHICLE_MODELS] = {"Passat", "StickeredPassat", "Elise", "Hummer", "Porsche", "Lamborghini"};

} // namespace vlr

