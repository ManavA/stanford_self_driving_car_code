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
#include <gtest/gtest.h>
#include <fstream>

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;

void deserializeMatrix(istream& is, MatrixXf* target) {
  int rows;
  int cols;
  string str;
  is >> rows;
  is >> cols;
//  cout << "rows :" << rows << endl;
//  cout << "cols :" << cols << endl;
  getline(is, str);
  float* buf = (float*)malloc(sizeof(float)*rows*cols);
  is.read((char*)buf, sizeof(float)*rows*cols);
  MatrixXf tmp = Eigen::Map<MatrixXf>(buf, rows, cols); //TODO: This copy is probably not necessary.
  *target = tmp;
  free(buf);
  getline(is, str);
}

void getRandomCloud(int num_pts, MatrixXf* cloud, VectorXf* intensities) {
  *cloud = MatrixXf::Random(num_pts, 3);
  *intensities = VectorXf::Random(num_pts);
}

void getTestCloud(MatrixXf* points, VectorXf* intensities) {
  MatrixXf cloud;
  ifstream file;
  file.open("test/pointcloud0000");
  if(!file.is_open()) {
    cerr << "Could not open test file." << endl;
    exit(1);
  }
    
  deserializeMatrix(file, &cloud);
  file.close();

  int idx = rand() % cloud.rows();
  vector<int> indices;
  indices.reserve(cloud.rows() / 10);
  for(int i = 0; i < cloud.rows(); ++i) {
    if((cloud.block(i, 0, 1, 3) - cloud.block(idx, 0, 1, 3)).norm() < 300)
      indices.push_back(i);
  }
  *points = MatrixXf(indices.size(), 3);
  *intensities = VectorXf(indices.size());
  for(size_t i = 0; i < indices.size(); ++i) {
    points->row(i) = cloud.block(indices[i], 0, 1, 3);
    (*intensities)(i) = cloud(indices[i], 3);
  }

  *points /= 100.; //Put in meters.
}

void getDescriptorPipeline(vector< shared_ptr<ComputeNode> >* nodes,
			   vector< shared_ptr<DescriptorNode> >* descriptor_nodes) {
  shared_ptr<CloudOrienter> co(new CloudOrienter());
  MatrixXf* points = new MatrixXf;
  VectorXf* intensities = new VectorXf;
  getTestCloud(points, intensities);
  co->input_cloud_ = shared_ptr<MatrixXf>(points);
  co->input_intensities_ = shared_ptr<VectorXf>(intensities);

  shared_ptr<CloudProjector> cp0(new CloudProjector(0, 50, co));
  shared_ptr<CloudProjector> cp1(new CloudProjector(1, 50, co));
  shared_ptr<CloudProjector> cp2(new CloudProjector(2, 50, co));
  shared_ptr<OrientedBoundingBoxSize> obbs(new OrientedBoundingBoxSize(co));
    
  vector<float> u_offset_pcts, v_offset_pcts;
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(0); v_offset_pcts.push_back(0);
  u_offset_pcts.push_back(1); v_offset_pcts.push_back(1);
  shared_ptr<HogArray> ha(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(75, 75), cv::Size(75, 75), cv::Size(75, 75), cv::Size(25, 25), 9));
  for(size_t j = 0; j < u_offset_pcts.size(); ++j) {
    shared_ptr<HogWindow> hw(new HogWindow(j, ha));
    nodes->push_back(hw);
    nodes->push_back(shared_ptr<RandomProjector>(new RandomProjector(10, 42000*(j+1), hw)));
  }
  nodes->push_back(co);
  nodes->push_back(cp0);
  nodes->push_back(cp1);
  nodes->push_back(cp2);
  nodes->push_back(obbs);
  nodes->push_back(ha);
    
  // -- Fill descriptor_nodes.
  for(size_t i = 0; i < nodes->size(); ++i) {
    shared_ptr<DescriptorNode> tmp = boost::dynamic_pointer_cast<DescriptorNode, ComputeNode>(nodes->at(i));
    if(tmp)
      descriptor_nodes->push_back(tmp);
  }

  if(getenv("DEBUG")) {
    for(size_t i = 0; i < nodes->size(); ++i) {
      (*nodes)[i]->debug_ = true;
    }
  }
    
}

TEST(AllDescriptors, single_vs_multithreaded) {
  int num_separate_pipelines = 1000;
  time_t start, end;
  
  // -- Compute single-threaded descriptors.
  vector< shared_ptr<DescriptorNode> > descriptor_nodes;
  vector< shared_ptr<ComputeNode> > nodes;
  srand(0);
  for(int i = 0; i < num_separate_pipelines; ++i) { 
    getDescriptorPipeline(&nodes, &descriptor_nodes);
  }
  Pipeline pl(nodes, 1);
  cout << "Starting computation." << endl;
  time(&start);
  pl.compute();
  time(&end);
  cout << "Single threaded computation took " << difftime(end, start) << " seconds." << endl;
  vector< shared_ptr<VectorXf> > descriptors(descriptor_nodes.size());
  for(size_t i = 0; i < descriptor_nodes.size(); ++i)
    descriptors[i] = descriptor_nodes[i]->getDescriptor();

  cout << pl.reportTiming() << endl;
  
  pl.flush();
  // -- Do the same with multiple threads.
  descriptor_nodes.clear();
  nodes.clear();
  srand(0);

  for(int i = 0; i < num_separate_pipelines; ++i) {
    getDescriptorPipeline(&nodes, &descriptor_nodes);
  }

  cout << "Starting multithreaded computation." << endl;
  Pipeline pl2(nodes, 8);
  time(&start);
  pl2.compute();
  time(&end);
  cout << "Multithreaded computation took " << difftime(end, start) << " seconds." << endl;
  // -- Make sure they are the same.
  for(size_t i = 0; i < descriptor_nodes.size(); ++i) {
    //cout << descriptor_nodes[i]->getShortName() << endl;
    if(!descriptor_nodes[i]->getDescriptor()) { 
      EXPECT_FALSE(descriptors[i]);
      continue;
    }
    
    float norm = (*descriptor_nodes[i]->getDescriptor() - *descriptors[i]).norm();
    //cout << "Sum of squared differences for " << descriptor_nodes[i]->getShortName() << ": " << norm << endl;
    EXPECT_FLOAT_EQ(norm, 0);
  }

  cout << pl2.reportTiming() << endl;
  
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


