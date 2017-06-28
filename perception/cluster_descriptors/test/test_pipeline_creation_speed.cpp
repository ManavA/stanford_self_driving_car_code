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
#include <pipeline/pipeline.h>
#include <fstream>

#define NUM_CLUSTERS 1000
#define NUM_POINTS 1000

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;
using namespace pipeline;

void getRandomCloud(int num_pts, MatrixXf* cloud, VectorXf* intensities) {
  *cloud = MatrixXf::Random(num_pts, 3);
  *intensities = VectorXf::Random(num_pts);
}

int main(int argc, char** argv) {
  shared_ptr<MatrixXf> cloud(new MatrixXf());
  shared_ptr<VectorXf> intensity(new VectorXf());
  getRandomCloud(NUM_POINTS, cloud.get(), intensity.get());
  vector< shared_ptr<MatrixXf> > clouds(NUM_CLUSTERS, cloud);
  vector< shared_ptr<VectorXf> > intensities(NUM_CLUSTERS, intensity);

  vector< shared_ptr<ComputeNode> > nodes;
  vector< shared_ptr<DescriptorNode> > descriptor_nodes;
  timeval start, end;
  gettimeofday(&start, NULL);
  generateClusterDescriptorPipeline(clouds, intensities, &nodes, &descriptor_nodes);
  gettimeofday(&end, NULL);
  cout << "Average time to generate pipeline for one cluster: "
       << ((end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.) / (double)NUM_CLUSTERS << " ms. " << endl;
    
  
}
