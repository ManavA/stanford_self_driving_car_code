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
#include <map>
#include <pipeline/pipeline.h>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

namespace pipeline {

ComputeNode::ComputeNode() :
  debug_(false),
  disabled_(false),
  done_computation_(false),
  started_computation_(false),
  mutex_(pthread_mutex_t())
{
}

bool ComputeNode::trylock() {
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}
  
void ComputeNode::lock() {
  pthread_mutex_lock(&mutex_);
}
  
void ComputeNode::unlock() {
  pthread_mutex_unlock(&mutex_);
}
  
double ComputeNode::getComputationTime() const {
  assert(done_computation_);
  return time_msec_;
}
  
void ComputeNode::display() const {
  _display();
}
  
void ComputeNode::_display() const {
  cout << "  " << getFullName() << " has no display functionality." << endl;
}
  
void ComputeNode::flush() {
  _flush();
  done_computation_ = false;
  started_computation_ = false;
  time_msec_ = -1;
}
  
void ComputeNode::compute() {
  assert(!trylock());
  assert(!started_computation_);
    if(done_computation_)
    cerr << getFullName() << " is done computation, but its compute() function is being called!" << endl;
  assert(!done_computation_);
  
  started_computation_ = true;
  
  timeval start, end;
  gettimeofday(&start, NULL);
  _compute();
  gettimeofday(&end, NULL);
  done_computation_ = true;
  time_msec_ = (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.;

  if(debug_) {
    cout << "Displaying " << getFullName() << endl;
    cout << "  Computation took " << time_msec_ << " ms." << endl;
    display();
  }
  unlock();
}

  //! TODO: getShortName is 32-bit unsafe!
string ComputeNode::getShortName() const {
  locale loc;
  const collate<char>& coll = use_facet<collate<char> >(loc);
  string anc = getGenealogy();
  long hash = coll.hash(anc.data(), anc.data() + anc.length());

  ostringstream oss;
  oss << _getName() << "+genealogy:" << hash;
  if(oss.str().length() >= getFullName().length())
    return getFullName();

  return oss.str();
}
  

string ComputeNode::getGenealogy() const {
  if(inputs_.size() == 0)
    return string("");
    
  ostringstream oss;
  oss << "(";
  for(size_t i = 0; i < inputs_.size(); ++i) { 
    oss << inputs_[i]->getFullName();
    if(i < inputs_.size() - 1)
      oss << "&&";
  }
  oss << ")=>";
  return oss.str();
}
  
    
string ComputeNode::getFullName() const {
  ostringstream oss;
  oss << getGenealogy() << _getName();
  return oss.str();
}

void ComputeNode::registerInput(shared_ptr<ComputeNode> input) {
  inputs_.push_back(input.get());
  input->outputs_.push_back(this);
}

bool ComputeNode::ready() {
  // If it's locked, either another thread is adding this to the ready list already,
  // or another thread is already computing on it.
  if(!trylock())
    return false;
  
  if(started_computation_ || disabled_) {
    unlock();
    return false;
  }
  
  for(size_t i = 0; i < inputs_.size(); ++i) {
    if(!inputs_[i]->done_computation_ || inputs_[i]->disabled_) {
      unlock();
      return false;
    }
  }

  unlock();
  return true;
}

DescriptorNode::DescriptorNode() : ComputeNode()
{
}

void DescriptorNode::display() const {
  cout << "  " << printDescriptor() << endl;
  _display();
}
  
shared_ptr<VectorXf> DescriptorNode::getDescriptor() const {
  assert(done_computation_);
  return _getDescriptor();
}

string DescriptorNode::printDescriptor() const {
  assert(done_computation_);
  ostringstream oss;
  oss << getFullName() << " descriptor (length " << getDescriptorLength() << "): ";
  if(!getDescriptor() || getDescriptor()->rows() == 0)
    oss << "Empty descriptor.";
  else
    oss << getDescriptor()->transpose();
  return oss.str();
}
  
Pipeline::Pipeline(const Pipeline& other) :
  nodes_(other.nodes_),
  num_threads_(other.num_threads_),
  ready_set_(other.ready_set_),
  mutex_(other.mutex_),
  done_computation_(other.done_computation_),
  num_nodes_computing_(other.num_nodes_computing_)
{
  assertCompleteness();
}

Pipeline::Pipeline(const vector< shared_ptr<ComputeNode> >& nodes, int num_threads) :
  nodes_(nodes),
  num_threads_(num_threads),
  mutex_(pthread_mutex_t()),
  done_computation_(false),
  num_nodes_computing_(0)
{
  assertCompleteness();
}

Pipeline::Pipeline(int num_threads) :
  num_threads_(num_threads),
  mutex_(pthread_mutex_t()),
  done_computation_(false),
  num_nodes_computing_(0)
{
}

bool Pipeline::trylock() {
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}
  
void Pipeline::lock() {
  pthread_mutex_lock(&mutex_);
}
  
void Pipeline::unlock() {
  pthread_mutex_unlock(&mutex_);
}

  
void Pipeline::assertCompleteness() {
  queue<ComputeNode*> to_check;
  for(size_t i = 0; i < nodes_.size(); ++i) {
    if(nodes_[i]->inputs_.empty())
      to_check.push(nodes_[i].get());
  }

  set<ComputeNode*> found;
  while(!to_check.empty()) {
    ComputeNode* active = to_check.front();
    to_check.pop();
    found.insert(active); //Won't insert duplicates.
    for(size_t i = 0; i < active->outputs_.size(); ++i) {
      to_check.push(active->outputs_[i]);
    }
  }

  assert(found.size() == nodes_.size());  
}
  
void Pipeline::flush() {
  num_nodes_computing_ = 0;
  for(size_t i = 0; i < nodes_.size(); ++i) { 
    nodes_[i]->flush();
    assert(!nodes_[i]->done_computation_);
  }
  done_computation_ = false;
}

string Pipeline::reportTiming() {
  assert(done_computation_);
  
  ostringstream oss;
  map<string, double> times;
  map<string, double> num_computations;
  for(size_t i = 0; i < nodes_.size(); ++i) {
    times[nodes_[i]->getShortName()] += nodes_[i]->getComputationTime();
    ++num_computations[nodes_[i]->getShortName()];
  }

  assert(times.size() == num_computations.size());
  map<string, double>::iterator cit;
  for(cit = num_computations.begin(); cit != num_computations.end(); ++cit) {
    oss << cit->first << " computed " << cit->second << " times, average time was " << times[cit->first] / cit->second << " ms." << endl;
  }

  return oss.str();
}
  
void Pipeline::compute() {
  assert(!done_computation_);
  assert(ready_set_.empty());

  // -- Find all ready nodes and put them into the queue.
  for(size_t i = 0; i < nodes_.size(); ++i) {
    if(nodes_[i]->ready())
      ready_set_.insert(nodes_[i].get());
  }
  assert(!ready_set_.empty());

  // -- Spawn off num_threads - 1 worker threads.
  pthread_t threads[num_threads_-1];
  for(int i=0; i<num_threads_-1; ++i) {
    timeval start, end;
    gettimeofday(&start, NULL);
    pthread_create(&(threads[i]), NULL, propagateComputation2, (void*)this);
    gettimeofday(&end, NULL);
    //cout << "Creating 1 thread took " << end.tv_usec - start.tv_usec << " usec." << endl;
  }

  // -- Start computation in this thread.
  propagateComputation2(this); // Returns when all computation is finished.

  // -- Wait until all worker threads are done.
  for(int i=0; i<num_threads_-1; ++i)
    pthread_join(threads[i], NULL);

  done_computation_ = true;
}

  
ComputeNode* Pipeline::getReadyNode(bool* done) {
  lock();
  
  // -- If we're done, say so and return.
  if(num_nodes_computing_ == 0 && ready_set_.empty()) { 
    *done = true;
    unlock();
    return NULL;
  }
  
  // -- If there are no nodes ready, say so and return.
  if(ready_set_.empty()) {
    *done = false;
    unlock();
    return NULL;
  }
  
  // -- Otherwise, hand out the next node in the queue.
  ComputeNode* node = *ready_set_.begin();
  ready_set_.erase(ready_set_.begin()); //amortized constant

  if(!ready_set_.empty())
    assert(*ready_set_.begin() != node);
  assert(node);

  ++num_nodes_computing_;
  node->lock();
  unlock();
  return node;
}

void Pipeline::registerCompleted(ComputeNode* node) { 
  lock();
  --num_nodes_computing_;
  
  // -- Add all newly ready nodes.
  for(size_t i = 0; i < node->outputs_.size(); ++i) {
    if(node->outputs_[i]->ready()) { 
      ready_set_.insert(node->outputs_[i]); //
    }
  }

  unlock();
  node->unlock();
}


void* propagateComputation2(void *pipeline) {
  Pipeline& pl = *((Pipeline*) pipeline);

  while(true) {
    bool done = false;
    ComputeNode* node = pl.getReadyNode(&done);
    if(done)
      break;
    
    else if(!node)
      usleep(10);
    
    else { 
      node->compute();
      pl.registerCompleted(node);
    }
  }

  return NULL;
}

} //namespace
