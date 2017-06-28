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


#include <multibooster/multibooster.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  MultiBoosterDataset mbd(argv[1]);
  cout << mbd.status() << endl;

  size_t num_descr = mbd.objs_[0]->descriptors_.size();
  set<Object*> objects_perm;
  for(size_t i = 0; i < mbd.objs_.size(); ++i) {
    assert(mbd.objs_[i]->descriptors_.size() == num_descr);
    objects_perm.insert(mbd.objs_[i]);
  }


  for(size_t i = 0; i < num_descr; ++i) {
    int num_unique = 0;
    set<Object*>::iterator it;
    set<Object*> objects = objects_perm;
    while(!objects.empty()) {
      ++num_unique;
      Object* active = *objects.begin();
      objects.erase(active);
      vector<Object*> to_delete;
      for(it = objects.begin(); it != objects.end(); ++it) {
	if(fastEucSquared(*(*it)->descriptors_[i].vector,
			  *active->descriptors_[i].vector,
			  (*it)->descriptors_[i].length_squared,
			  active->descriptors_[i].length_squared) < 1e-4) {
	  to_delete.push_back(*it);
	}
      }
      for(size_t i = 0; i < to_delete.size(); ++i) {
	objects.erase(to_delete[i]);
      }
    }
    cout << "Descriptor " << mbd.feature_map_.toName(i) << " has " << num_unique << " unique descriptors, out of " << mbd.objs_.size() << " total." << endl;
  }
    
  return 0;
}
