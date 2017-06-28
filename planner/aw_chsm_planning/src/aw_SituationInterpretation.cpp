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


#include <aw_SituationInterpretation.hpp>

using namespace sit;

namespace vlr {

///*!
// *
// * @param pdf: comprises particles where the first dimension is a factor (e.g. symbolic behavior)
// * @param factor_size is the number of values the factor can take form of
// * @return
// */
//template< class PdfKernelEst = NormalPdfEst >
//typename PdfKernelEst::BaseMixturePdf* estimate_factorized_mixture_pdf(const ParticlePdf& pdf, int factor_size) {
//	assert( pdf.particles.size() );
//	MixturePdf<PdfKernelEst> mixture_est( pdf[0].size()-1, factor_size );
//	for (int i = 0; i < pdf.particles.size(); ++i) {
//		mixture_est.kernels.add_sample( pdf.particles[i].tail(1, pdf.particles[i].size()-1), pdf.weights[i] );
//		mixture_est.weights.add_sample( pdf.particles[i][0], pdf.weights[i] );
//	}
//	typename PdfKernelEst::BaseMixturePdf* res_mixture = mixture_est.get_pdf();
//	res_mixture.weights /= weights.sum();
//	return res_mixture;
//}


SituationInterpretation::SituationInterpretation(vlr::ChsmPlanner* planner, int threads, int particles_per_obj, int batch_length) :
	particles_per_obj_(particles_per_obj),
	clutter_threshold_(0.),
	planner_(planner),
	dbn_(batch_length, 0),		// (t-1)->(t)
	inference_(0, threads)
{
	assert( batch_length > 0 && "Error: The dbn needs to have at least one time slize");
	assert( batch_length == 1 && "Error: currently the batch_length can only be one, since the dbn uses the same time step length for each step");


}


void SituationInterpretation::estimate(double new_time)
{
	// make a time step to prepare the dbn for the next estimation
	dbn_.time_step();
	dbn_.vehicle_cdf->func.dt = new_time - time_;		// adapted time step length
	for (ObjIdSet::const_iterator it = obj_ids_.begin(); it != obj_ids_.end(); ++it) {
		dbn_.add_node(dbn_.X, dbn_.t, make_id(*it));
	}
	dbn_.match_prototypes();

	//-------------------------------------------------------------------------
	// make one prediction step for data association starting from the estimation of the last time step
	//-------------------------------------------------------------------------

	// init evidences for prediction by adding the estimation result from last time step
	EvidenceSet evidences(dbn_);
	for (ObjIdSet::const_iterator it = obj_ids_.begin(); it != obj_ids_.end(); ++it) {
		int obj_id = *it;
		evidences.add_evidence( dbn_.get_node( dbn_.X, dbn_.t_min, make_id( obj_id ) ), estimation_[obj_id] );
	}

	// add a question on the measurement node for each currently known obj
	QuestionSet q_pred(dbn_);
	bimap<int, NodeId> obj_q_map;
	for (ObjIdSet::const_iterator it = obj_ids_.begin(); it != obj_ids_.end(); ++it) {
		int obj_id = *it;
		NodeId q_id = q_pred.add_question( dbn_.get_node( dbn_.Z, dbn_.t, make_id( obj_id ) ) );
		obj_q_map.left.insert( make_pair(obj_id, q_id) );
	}

	// infer prediction
	std::auto_ptr< ParticleInference::Answer > meas_prediction( inference_.infer(q_pred, evidences) );

	// retrieve results for data assoc (VehicleId -> Z)
	boost::unordered_map< int, boost::shared_ptr< ParticlePdf > > predicted_measurements;
	for (bimap<int, NodeId>::right_map::const_iterator q_it = obj_q_map.right.begin(); q_it != obj_q_map.right.end(); ++q_it) {
		NodeId q_id = q_it->first;
		int obj_id = q_it->second;
		predicted_measurements.insert( make_pair(obj_id, meas_prediction->answer_pdfs[q_id]) );
	}

	// retrieve the measured vehicle states from the planners vehicle manager
	assert( planner_->vehicle_manager );
	boost::unordered_map<int, StateVec> measurements;
	std::map<int, Vehicle>& vehicles = planner_->vehicle_manager->vehicle_map;
	for (std::map<int, Vehicle>::const_iterator v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
		assert( measurements.find( v_it->first ) == measurements.end() );
		const Vehicle& vehicle = v_it->second;

		// convert the measurement data
		StateVec evidence( Dbn::Z_SIZE );
		evidence[Dbn::Z_x1] = vehicle.xMatchedFrom();
		evidence[Dbn::Z_x2] = vehicle.yMatchedFrom();
		evidence[Dbn::Z_psi] = vehicle.yawMatchedFrom();
		evidence[Dbn::Z_v] = vehicle.speed();
		measurements.insert( make_pair(v_it->first, evidence) );
	}

	// data assoc (obj, measurement)
	std::auto_ptr< boost::bimap<int, int> > assoc( GreedyDataAssoc<>()(predicted_measurements, measurements, clutter_threshold_) );

	// add nodes for unresolved measurements and associate them
	for (boost::unordered_map<int, StateVec>::const_iterator m_it = measurements.begin(); m_it != measurements.end(); ++m_it) {
		int m_id = m_it->first;
		if ( assoc->right.find(m_id) == assoc->right.end() ) {
			int obj_id = id_gen_();
			obj_ids_.insert(obj_id);
			dbn_.add_node( dbn_.X, dbn_.t, make_id(obj_id) );
			assoc->left.insert( make_pair( obj_id, m_id ) );
			// add prototype for init prior
			assert(false);
		}
	}

	// remove objects that left the area of interest
	// if (dist(ego_pdf.mean, obj_pdf.mean) > max_dist_threshold || det(obj_pdf.covar) > max_uncertainty_threshold) {
	//		int obj_id = ;
	//		assert( assoc.left.find(obj_id) == assoc.left.end() );
	//		id_gen.free_id(obj_id);
	//		dbn_.remove_node( dbn_.X, dbn_.t, make_id(obj_id) );
	//		remove prior prototype
	//	}

	// build dbn
	dbn_.match_prototypes();

	// add evidences to the network
	for (boost::unordered_map<int, StateVec>::const_iterator it = measurements.begin(); it != measurements.end(); ++it) {
		int m_id = it->first;
		bimap<int, int>::right_map::const_iterator assoc_it = assoc->right.find( m_id );
		// add evidence for associated measurements
		if ( assoc_it != assoc->right.end() ) {
			int obj_id = assoc_it->second;
			evidences.add_evidence( dbn_.get_node( dbn_.Z, 0, make_id( obj_id ) ), it->second );
		}
	}



	//-------------------------------------------------------------------------
	// make estimation
	//-------------------------------------------------------------------------

	// set questions
	// x, l, c, b, p, t

	// set particles according to number of objects
	inference_.particle_count = obj_ids_.size() * particles_per_obj_;

	// infer
	// save estimation results
	// map< int, shared_ptr<ParticlePdf> >

	// calculate most likely plans

	// calculate most likeli behavior according to ml plan

	// calculate expectation of trajectory
	// calculate expectation of

	// update future vehicle poses
	// update vehicle poses
	// -> lock vehicle manager

	// update time
	time_ = new_time;
}


} // namespace vlr
