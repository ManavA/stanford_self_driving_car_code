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


#include <pipeline/pipeline.h>
#include <gtest/gtest.h>

#define NUM_THREADS 2

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;

class ExampleNode : public ComputeNode {
 public:
  ExampleNode(int id);
  
 protected:
  int id_; // A parameter of the computation.

  void _flush() {};
  void _compute();
  std::string _getName() const;  
};
  
void ExampleNode::_compute() {
  cout << getFullName() << " is computing." << endl;
  usleep(5e5);
}

ExampleNode::ExampleNode(int id) :
  ComputeNode(),
  id_(id)
{
}
  
string ExampleNode::_getName() const {
  ostringstream oss;
  oss << "ExampleNode" << id_;
  return oss.str();
}

TEST(Pipeline, terminates) {
  vector< shared_ptr<ComputeNode> > nodes;
  for(int i=0; i<10; ++i)
    nodes.push_back(shared_ptr<ExampleNode>(new ExampleNode(i)));    

  cout << "Using " << sysconf(_SC_NPROCESSORS_ONLN) << " threads." << endl;
  Pipeline pl(nodes, sysconf(_SC_NPROCESSORS_ONLN));
  pl.compute();
  EXPECT_TRUE(true);
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
