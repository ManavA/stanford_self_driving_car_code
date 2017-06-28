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



#ifndef VEHICLEMODELS_H
#define VEHICLEMODELS_H

#include <string>
#include <vector>
#include <GL/gl.h>
#include <GL/glu.h>
#include <global.h>
#include <textures.h>

namespace vlr {

enum VehicleModelId_t {PASSAT_MODEL_ID=0, STICKERED_PASSAT_MODEL_ID, ELISE_MODEL_ID, HUMMER_MODEL_ID, PORSCHE_MODEL_ID, LAMBORGHINI_MODEL_ID, NUM_VEHICLE_MODELS};

//macro to define a class derived from VehicleModelGL named via the argument
//first defines function to generate openGL display list for model
//Init calls the generator function
#define modelClass(name)\
					GLint generate##name();\
					class Vehicle##name##Model : public VehicleModelGL {\
					public:\
					~Vehicle##name##Model() { if(display_list_) glDeleteLists(display_list_, 1); };\
					void Init()   { display_list_ = generate##name(); };\
				}\


//model class
class VehicleModelGL {
protected:	
	GLint display_list_;
public :
	VehicleModelGL() {display_list_ = 0;}
	virtual ~VehicleModelGL()	{}
	virtual void Init() {}
  void draw() {glCallList(display_list_);}
};


//Defined models.  To add a new one, implement a function generate_"name"_model and add a class here
//                 then add it to the constructor of VehicleModelsGL and to the dgc_model_name_to_id() function
modelClass(StickeredPassat);
modelClass(Elise);
modelClass(Hummer);
modelClass(Porsche);
modelClass(Lamborghini);

//class containing all vehicle models
class VehicleModelsGL {

protected:
	std::vector<VehicleModelGL*> models_;

public:
	VehicleModelsGL();
	~VehicleModelsGL();
	void init();
	void draw(VehicleModelId_t model);																  //draw a particular model given an ID
	void draw(std::string model_name) {draw(modelNameToId(model_name)); };		//draw a model given a name
	int numVehicles() {return (int)models_.size();};

	//returns an integer identifier given a model string
	static inline VehicleModelId_t modelNameToId(std::string model_name) {
	  for(int i = 0; i < NUM_VEHICLE_MODELS; i++) {
	    if(model_names[i] == model_name) {return (VehicleModelId_t)i;}
	  }
	  return (VehicleModelId_t)-1;
  }

	  public:
	    static const std::string model_names[NUM_VEHICLE_MODELS];// {"Passat", "StickeredPassat", "Elise", "Hummer", "Porsche", "Lamborghini"};
};


} // namespace vlr
#endif
