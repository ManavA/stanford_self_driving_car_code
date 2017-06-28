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


#ifndef CACHED_COMPUTATION_PIPELINE_H_
#define CACHED_COMPUTATION_PIPELINE_H_

#include <Eigen/Eigen>
#include <pthread.h>
#include <boost/shared_ptr.hpp>
#include <errno.h>
#include <locale>
#include <list>
#include <sys/time.h>
#include <set>
#include <queue>

namespace pipeline { 

//! Abstract base class that represents a node in the computation graph.  All nodes inherit from this class, at least indirectly.
class ComputeNode {  
 public:
  //! Whether to pause after computation and display debugging output by running display()
  bool debug_;
  //! Whether to compute or not.
  bool disabled_;
  
  ComputeNode();
  //! Returns a string that is unique to this node for the given set of ancestor nodes.
  std::string getFullName() const;
  //! Returns a string that is unique to this node for the given set of ancestor nodes, yet is still short and informative.
  std::string getShortName() const;
  //! Returns computation time in ms.
  double getComputationTime() const;
  
 protected:
  std::vector<ComputeNode*> outputs_;

  void registerInput(boost::shared_ptr<ComputeNode> input);
  
 private:
  //! Time to compute, in milliseconds.
  double time_msec_;
  std::vector<ComputeNode*> inputs_;
  bool done_computation_;
  bool started_computation_;
  pthread_mutex_t mutex_;

  //! Performs computation, given data from nodes in inputs_.
  virtual void _compute() = 0;
  //! Returns a name that is unique for any parameter settings of this node.
  virtual std::string _getName() const = 0;
  //! Display function, called on compute() when debug_ == true.
  virtual void _display() const;
  //! Clears all cached data.
  virtual void _flush() = 0;

  void flush();
  //! Virtual for overloading by Descriptor and other abstract ComputeNode subclasses.
  virtual void display() const;
  void compute();
  std::string getGenealogy() const;
  bool ready();

  bool trylock();
  void lock();
  void unlock();
  
  friend class Pipeline;
  friend class DescriptorNode;
  //  friend void* propagateComputation(void *pipeline);
  friend void* propagateComputation2(void *pipeline);
};

//! Defines a common descriptor interface based on the Eigen::VectorXf.
class DescriptorNode : public ComputeNode {
 public:
  DescriptorNode();
  virtual int getDescriptorLength() const = 0;
  //! NULL if no descriptor could be computed.
  boost::shared_ptr<Eigen::VectorXf> getDescriptor() const;
  std::string printDescriptor() const;
  
 private:
  void display() const;
  virtual boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const = 0;
};
  
//! Class that represents the entire computation graph and manages its execution.
class Pipeline {
 public:
  std::vector< boost::shared_ptr<ComputeNode> > nodes_;

  Pipeline(const Pipeline& other);
  Pipeline(int num_threads = 1);
  Pipeline(const std::vector< boost::shared_ptr<ComputeNode> >& nodes, int num_threads);
  //! Clears all cached data in the pipeline, i.e. calls flush on all nodes.
  void flush();
  //! Runs all computation until completion.
  void compute();
  //! Returns a string with timing results for each node name.  Nodes that have identical getShortName()s will be averaged together.
  std::string reportTiming();
  

 protected:
  //! Makes sure that all nodes in the pipeline are actually in nodes_.
  //! This prevents the possibility of a very nasty bug.
  void assertCompleteness();
  
 private:
  int num_threads_;
  //std::queue<ComputeNode*> ready_queue_;
  std::set<ComputeNode*> ready_set_;
  pthread_mutex_t mutex_;
  bool done_computation_;
  size_t num_nodes_computing_;

  ComputeNode* getReadyNode(bool* done);
  //! Adds newly ready nodes to the ready_queue_.
  void registerCompleted(ComputeNode* node);

  bool trylock();
  void lock();
  void unlock();
  
  //friend void* propagateComputation(void *pipeline);
  friend void* propagateComputation2(void *pipeline);
};

//void* propagateComputation(void *pipeline);
void* propagateComputation2(void *pipeline);
 
}

#endif
