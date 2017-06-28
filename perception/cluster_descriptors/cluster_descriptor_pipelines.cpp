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


#include <cluster_descriptors/cluster_descriptors.h>

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;

DescriptorPipeline::DescriptorPipeline(int num_components, int num_threads) :
  Pipeline(num_threads),
  num_components_(num_components),
  num_threads_(num_threads),
  output_nodes_(vector< vector< shared_ptr<DescriptorNode> > >(num_components))
{
  vector< shared_ptr<DescriptorNode> > all_descriptor_nodes;
  generateClusterDescriptorPipeline(vector< shared_ptr<MatrixXf> >(num_components_, shared_ptr<MatrixXf>(new MatrixXf())),
				    vector< shared_ptr<VectorXf> >(num_components_, shared_ptr<VectorXf>(new VectorXf())),
				    &nodes_, &all_descriptor_nodes);

  assert(!nodes_.empty());
  assert(!all_descriptor_nodes.empty());
  
  // -- Fill output_nodes_.
  int num_outputs_per_component = (int)all_descriptor_nodes.size() / num_components_;
  assert(num_outputs_per_component * num_components_ == (int)all_descriptor_nodes.size());
  size_t idx = 0;
  for(int i = 0; i < num_components_; ++i) {
    output_nodes_[i] = vector< shared_ptr<DescriptorNode> >(num_outputs_per_component);
    for(int j = 0; j < num_outputs_per_component; ++j, ++idx) {
      output_nodes_[i][j] = all_descriptor_nodes[idx];
    }
  }
  
  // -- Fill cloud_orienters_.  Assumes they will be in order.
  cloud_orienters_.reserve(num_components_);
  for(size_t i = 0; i < nodes_.size(); ++i) { 
    shared_ptr<CloudOrienter> foo = boost::dynamic_pointer_cast<CloudOrienter>(nodes_[i]);
    if(foo)
      cloud_orienters_.push_back(foo);
  }
  assert(cloud_orienters_.size() == (size_t)num_components_);

  // -- Turn on debug flags.
  if(getenv("DEBUG")) {
    for(size_t i = 0; i < nodes_.size(); ++i) {
      nodes_[i]->debug_ = true;
    }
  }

  assertCompleteness();
    
}

void DescriptorPipeline::setInputs(const vector< shared_ptr<MatrixXf> >& clouds,
				   const vector< shared_ptr<VectorXf> >& intensities) {
  assert(clouds.size() == intensities.size());
  assert(clouds.size() <= (size_t)num_components_);
  for(size_t i = 0; i < cloud_orienters_.size(); ++i) {
    if(i < clouds.size()) {
      cloud_orienters_[i]->disabled_ = false;
      cloud_orienters_[i]->setInputCloud(clouds[i]);
      cloud_orienters_[i]->setInputIntensity(intensities[i]);
    }
    else
      cloud_orienters_[i]->disabled_ = true;
  }
}

void DescriptorPipeline::setInput(shared_ptr<MatrixXf> cloud,
				  shared_ptr<VectorXf> intensity) { 
  
  cloud_orienters_[0]->disabled_ = false;
  cloud_orienters_[0]->setInputCloud(cloud);
  cloud_orienters_[0]->setInputIntensity(intensity);
  
  for(size_t i = 1; i < cloud_orienters_.size(); ++i)
      cloud_orienters_[i]->disabled_ = true;
}

int DescriptorPipeline::getNumBytes() const {
  int num_bytes = 0;
  for(size_t i = 0; i < output_nodes_[0].size(); ++i)
    num_bytes += output_nodes_[0][i]->getDescriptorLength();
  return num_bytes;
}

vector<string> DescriptorPipeline::getDescriptorNames() const {
  vector<string> names(output_nodes_[0].size());
  for(size_t i = 0; i < output_nodes_[0].size(); ++i) {
    names[i] = output_nodes_[0][i]->getFullName();
  }
  return names;
}

vector< shared_ptr<DescriptorNode> > DescriptorPipeline::getOutputDescriptorNodes(size_t id) {
  assert(id <= (size_t)num_components_);
  for(size_t i = 0; i < output_nodes_[id].size(); ++i) {
    assert(!output_nodes_[id][i]->disabled_);
  }
  return output_nodes_[id];
}

std::vector< std::vector< boost::shared_ptr<pipeline::DescriptorNode> > > DescriptorPipeline::getOutputDescriptorNodes() {
  return output_nodes_;
}

// vector< vector< shared_ptr<VectorXf> > > computeDescriptors(shared_ptr<MatrixXf> clouds, shared_ptr<VectorXf> intensities) {
//   size_t num_chunks = ceil((float)clouds.size() / (float)num_components_);
//   for(size_t i = 0; i < num_chunks; ++i) {

  
//   vector< vector < shared_ptr<VectorXf> > > descriptors(num_components_, vector< shared_ptr<VectorXf> >(output_nodes_[0].size()));
//   for(size_t i = 0; i < output_nodes_.size(); ++i) {
//     assert(descriptors[i].size() == output_nodes_[i].size());
//     for(size_t j = 0; j < output_nodes_[i].size(); ++j) {
//       descriptors[i][j] = output_nodes_[i][j]->getDescriptor();
//     }
//   }
    
//   return descriptors;  
// }

void addCluster(shared_ptr<MatrixXf> cloud, shared_ptr<VectorXf> intensities,
		const vector<float>& u_offset_pcts,
		const vector<float>& v_offset_pcts,
		vector< shared_ptr<ComputeNode> >* nodes,
		vector< shared_ptr<DescriptorNode> >* descriptor_nodes) {
  
    shared_ptr<CloudOrienter> co(new CloudOrienter());
    co->input_cloud_ = cloud;
    co->input_intensities_ = intensities;
    nodes->push_back(co);

//SpinImageCustom_axis0,0,1_rowRes0.3_colRes0.3_numRows10_numCols10_radius-1 \
//SpinImageCustom_axis0,0,1_rowRes0.15_colRes0.15_numRows10_numCols5_radius-1 \
//SpinImageCustom_axis0,0,1_rowRes0.1_colRes0.1_numRows10_numCols5_radius-1 \
//SpinImageCustom_axis0,0,1_rowRes0.2_colRes0.2_numRows10_numCols15_radius-1 \
//SpinImageCustom_axis0,0,1_rowRes0.2_colRes0.2_numRows10_numCols5_radius-1

    shared_ptr<CloudSpinner> cs(new CloudSpinner(co));
    nodes->push_back(cs);
    vector< shared_ptr<SpinImage> > sis;
    sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 3, 10, 10)));
    sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 7, 14, 7)));
    sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 10, 20, 10)));
    sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 5, 10, 5)));
    for(size_t i = 0; i < sis.size(); ++i) {
      nodes->push_back(sis[i]);
      shared_ptr<Whitener> wh(new Whitener(sis[i]));
      nodes->push_back(wh);
      descriptor_nodes->push_back(wh);
    }
     
    shared_ptr<OrientedBoundingBoxSize> obbs(new OrientedBoundingBoxSize(co));
    nodes->push_back(obbs);
    descriptor_nodes->push_back(obbs);

    int ppm = 15;
    shared_ptr<CloudProjector> cp0(new CloudProjector(0, ppm, co, 3, 2*ppm, 2*ppm)); //Show at least 2m in each direction.
    shared_ptr<CloudProjector> cp1(new CloudProjector(1, ppm, co, 3, 2*ppm, 2*ppm));
    shared_ptr<CloudProjector> cp2(new CloudProjector(2, ppm, co, 3, 2*ppm, 2*ppm));
    nodes->push_back(cp0);
    nodes->push_back(cp1);
    nodes->push_back(cp2);
    

    // -- For each HogArray, add the HogWindows and RandomProjectors.
    vector< shared_ptr<HogArray> > hog_arrays;
    hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
    hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
    hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
    hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
    hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(ppm, ppm), cv::Size(ppm, ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
    hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
    for(size_t i = 0; i < hog_arrays.size(); ++i) { 
      nodes->push_back(hog_arrays[i]);

      for(size_t j = 0; j < u_offset_pcts.size(); ++j) {
	shared_ptr<HogWindow> hw(new HogWindow(j, hog_arrays[i]));
	shared_ptr<RandomProjector> rp = shared_ptr<RandomProjector>(new RandomProjector(20, (j+1) * 42000, hw));
	
	nodes->push_back(hw);
	nodes->push_back(rp);
	descriptor_nodes->push_back(rp);
      }
    }

}


// FEATURE_NAMES :=  	BoundingBoxRaw_radius0 BoundingBoxSpectral_radius0_spectralRadius0 ShapeSpectral_radius0 \
// 			CloudProjectionHog_kernel5x5_CloudProjection_intensity1_axis1_rows30_cols30_pixels_per_meter15_offset0x1_Hog_winSize30x30_blockSize30x30_blockStride30x30_cellSize10x10_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel5x5_CloudProjection_intensity1_axis1_rows30_cols30_pixels_per_meter15_offset1x1_Hog_winSize30x30_blockSize30x30_blockStride30x30_cellSize10x10_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel3x3_CloudProjection_intensity1_axis0_rows20_cols20_pixels_per_meter10_offset0.5x0.5_Hog_winSize20x20_blockSize20x20_blockStride20x20_cellSize5x5_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel5x5_CloudProjection_intensity1_axis1_rows30_cols30_pixels_per_meter10_offset0.5x0.5_Hog_winSize30x30_blockSize30x30_blockStride30x30_cellSize10x10_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel5x5_CloudProjection_intensity1_axis2_rows30_cols60_pixels_per_meter10_offset0.5x0.5_Hog_winSize60x30_blockSize60x30_blockStride60x30_cellSize10x10_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel3x3_CloudProjection_intensity1_axis1_rows20_cols20_pixels_per_meter20_offset1x1_Hog_winSize20x20_blockSize20x20_blockStride20x20_cellSize5x5_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel3x3_CloudProjection_intensity1_axis1_rows20_cols20_pixels_per_meter20_offset0x1_Hog_winSize20x20_blockSize20x20_blockStride20x20_cellSize5x5_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel3x3_CloudProjection_intensity1_axis1_rows20_cols20_pixels_per_meter20_offset0.5x0_Hog_winSize20x20_blockSize20x20_blockStride20x20_cellSize5x5_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjectionHog_kernel3x3_CloudProjection_intensity1_axis1_rows30_cols60_pixels_per_meter5_offset0.5x0.5_Hog_winSize60x30_blockSize60x30_blockStride60x30_cellSize10x10_nBins6_derivAperture1_winSigma-1_histNormType0_L2HysThreshold0.2_gammaCorrection0 \
// 			CloudProjection_intensity1_axis0_rows16_cols8_pixels_per_meter8_offset0.5x0.5 \
// 			CloudProjection_intensity1_axis1_rows16_cols8_pixels_per_meter8_offset0.5x0.5 \
// 			CloudProjection_intensity1_axis0_rows10_cols10_pixels_per_meter5_offset0.5x0.5 \
// 			CloudProjection_intensity1_axis1_rows10_cols10_pixels_per_meter5_offset0.5x0.5 \
// 			CloudProjection_intensity1_axis2_rows8_cols13_pixels_per_meter2.5_offset0.5x0.5 \
// 			CloudProjection_intensity1_axis1_rows10_cols20_pixels_per_meter5_offset0.5x0.5 \
// 			CloudProjection_intensity1_axis1_rows8_cols16_pixels_per_meter2_offset0.5x0.5 \


void generateClusterDescriptorPipeline(vector< shared_ptr<MatrixXf> > clouds,
				       vector< shared_ptr<VectorXf> > intensities,
				       vector< shared_ptr<ComputeNode> >* nodes,
				       vector< shared_ptr<DescriptorNode> >* descriptor_nodes) {

  assert(descriptor_nodes->empty());
  assert(nodes->empty());
  assert(clouds.size() == intensities.size());

  // -- Set up Hog Array offsets.
  //0.5, 0.5
  //0.5, 0
  //0, 1
  //1, 1

  vector<float> u_offset_pcts, v_offset_pcts;
  //  u_offset_pcts.push_back(0); v_offset_pcts.push_back(0);
  //  u_offset_pcts.push_back(0); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(0); v_offset_pcts.push_back(1);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0.5);
  //  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(1);
  // u_offset_pcts.push_back(1); v_offset_pcts.push_back(0);
  //  u_offset_pcts.push_back(1); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(1); v_offset_pcts.push_back(1);
//   u_offset_pcts.push_back(0.25); v_offset_pcts.push_back(0.75);
//   u_offset_pcts.push_back(0.75); v_offset_pcts.push_back(0.25);
//   u_offset_pcts.push_back(0.25); v_offset_pcts.push_back(0.25);
//   u_offset_pcts.push_back(0.75); v_offset_pcts.push_back(0.75);

  // -- Allocate spaces for the nodes.  This is ugly.
  nodes->reserve((clouds.size() + 1) * 300); //Rough upper bound on num nodes per pipeline segment.
  descriptor_nodes->reserve((clouds.size() + 1) * 300);
  
  
  for(size_t i = 0; i < clouds.size(); ++i) { 
    timeval start, end;
    gettimeofday(&start, NULL);
    addCluster(clouds[i], intensities[i], u_offset_pcts, v_offset_pcts, nodes, descriptor_nodes);
    gettimeofday(&end, NULL);
    //cout << (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000. << " ms to create pipeline for one object. " << endl;
  }
}
