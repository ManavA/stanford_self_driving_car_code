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


#ifndef AW_SITUATIONINTERPRETATION_HPP_
#define AW_SITUATIONINTERPRETATION_HPP_

#include <interpretation.hpp>
#include "aw_ChsmPlanner.hpp"


namespace vlr {

class SituationInterpretation {
public:
	typedef sit::Dbn_XCBTSZ											Dbn;
	typedef std::set<int>											ObjIdSet;
	typedef std::map< int, boost::shared_ptr<dbnl::ParticlePdf> >	EstimationMap;


	SituationInterpretation(vlr::ChsmPlanner* planner, int threads = 0, int particles_per_obj = 500, int batch_length = 1);


	/*! \brief estimates the unobservable states of the global environment.
	 * \remark updates the states directly in the planner (e.g. vehicle_manager_)
	 * @param planner
	 * @param new_time
	 */
	void estimate(double new_time);


	int particles_per_obj_;				//!< number of particles that are used for inference per object
	double clutter_threshold_;			//!< likelihood threshold used for data association. Every measurement below this threshold will be classified as clutter

private:
	vlr::ChsmPlanner* planner_;			//!< reference to the planner

	dbnl::IdGenerator<> id_gen_;		//!< id generator used to generate object ids
	ObjIdSet obj_ids_;					//!< set of ids of currently existing objects
	double time_;						//!< time of state estimation

	Dbn dbn_;							//!< the dynamic bayesian network used to estimate the current state of the environment
	dbnl::ParticleInference inference_;	//!< particle inference used for the dbn
//	boost::shared_ptr< Answer > a_;

	EstimationMap estimation_;			//!< results of the current state estimations for each vehicle (vehicle id -> X,B,T)
};


} // namespace vlr

#endif /* AW_SITUATIONINTERPRETATION_HPP_ */
