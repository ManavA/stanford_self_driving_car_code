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


#include<multibooster/multibooster.h>

using namespace std;
using namespace qpOASES;
using namespace Eigen;

#define USE_QP false



WeakClassifierTree::WeakClassifierTree(const vector<WeakClassifier*>& pwcs, size_t recursion_limit) :
  pwcs_(pwcs),
  a_(VectorXf::Zero(pwcs_.size())),
  b_(0),
  X_(MatrixXf::Zero(pwcs_[0]->center_.rows(), pwcs_.size())),
  lchild_(NULL),
  rchild_(NULL),
  recursion_limit_(recursion_limit),
  depth_(0),
  max_wcs_(10),
  thresh_(1e-4),
  is_leaf_(false)
{
  growTree();
}

WeakClassifierTree::WeakClassifierTree(const vector<WeakClassifier*>& pwcs, const vector<WeakClassifier*>& consider, size_t recursion_limit,
	       size_t depth, const vector<VectorXf>& region_a, const vector<float>& region_b) :
  pwcs_(pwcs),
  consider_(consider),
  a_(VectorXf::Zero(pwcs_.size())),
  b_(0),
  region_a_(region_a),
  region_b_(region_b),
  X_(MatrixXf::Zero(pwcs_[0]->center_.rows(), pwcs_.size())),
  lchild_(NULL),
  rchild_(NULL),
  recursion_limit_(recursion_limit),
  depth_(depth),
  max_wcs_(10),
  thresh_(1e-4),
  is_leaf_(false)
{
  growTree();
}


void WeakClassifierTree::growTree() {
  // -- If we don't need any more splits, we're done.
  if(pwcs_.size() < max_wcs_ || depth_ == recursion_limit_) {
    makeLeaf();
    return;
  }

  // -- Otherwise, start building the next split.
  // -- Fill X_ with the center points.
  for(size_t i=0; i<pwcs_.size(); ++i) {
    X_.col(i) = pwcs_[i]->center_;
  }
  
  // -- Subtract off the mean.
  VectorXf mean = X_.rowwise().sum() / X_.cols();
  for(size_t i=0; i<pwcs_.size(); ++i) {
    X_.col(i) -= mean;
  }
  
  // -- If all of the weak classifiers had the same center, this is also a leaf.
  if(X_.sum() == 0) {
    makeLeaf();
    return;
  }
  
  // -- Power method to find the eigenvector of XX', i.e. 1st principal component.
  MatrixXf Xt = X_.transpose();
  bool done = false;
  while(!done) { 

    // Choose a random vector for a_.
    a_ = VectorXf::Zero(X_.rows());
    for(size_t i=0; i<(size_t)a_.rows(); ++i) {
      a_(i) = (double)rand() / (double)RAND_MAX;
    }
    a_.normalize();
    
    VectorXf prev = a_;
    while(true) { 
      prev = a_;
      a_ = X_ * (Xt * a_);

      // In the unlikely event that our random vector is in the nullspace
      // of AA', try again.
      if(a_.sum() == 0) {
	break;
      }
      a_.normalize();
      if((a_ - prev).norm() < thresh_) {
	done = true;
	break;
      }
    }
  }

  // -- Compute b_ to be the mean value of a_'xt for xt in pwcs_.
  VectorXf bs = VectorXf::Zero(pwcs_.size());
  for(size_t i=0; i<pwcs_.size(); ++i) {
    bs(i) = a_.dot(pwcs_[i]->center_);
  }
  b_ = bs.sum() / (float)bs.rows();

  // -- Add the newly computed a_ and b_ into the full list of constraints for the left and right children.
  // The right child region is all x for a_'x >= b_, or -a_'x <= -b_
  // The left child region is all x for a_'x <= b_
  vector<VectorXf> region_a_left = region_a_;
  vector<VectorXf> region_a_right = region_a_;
  vector<float> region_b_left = region_b_;
  vector<float> region_b_right = region_b_;
    
  region_a_left.push_back(a_);
  region_b_left.push_back(b_);
  region_a_right.push_back(-a_);
  region_b_right.push_back(-b_);

  // -- Compute which weak classifiers in the region go on which side of the split.
  vector<WeakClassifier*> left, right, left_consider, right_consider;
  left.reserve(pwcs_.size() + consider_.size());
  right.reserve(pwcs_.size() + consider_.size());
  left_consider.reserve(pwcs_.size() + consider_.size());
  right_consider.reserve(pwcs_.size() + consider_.size());
    
  for(size_t i=0; i<pwcs_.size(); ++i) {
    double dist = bs(i) - b_; //computeDistanceToSplit(pwcs_[i]->center_);
    double dist2 = dist*dist;

    if(dist == 0) {
      right.push_back(pwcs_[i]);
      left.push_back(pwcs_[i]);
    }
    else if(dist > 0) {
      right.push_back(pwcs_[i]);
      if(dist2 <= pwcs_[i]->theta_) {
	left_consider.push_back(pwcs_[i]);
      }
    }
    else {
      left.push_back(pwcs_[i]);
      if(dist2 <= pwcs_[i]->theta_) {
	right_consider.push_back(pwcs_[i]);
      }
    }
  }

  // -- If all the weak classifiers are very close to each other, they can end up not being split by the
  //    boundary.  If this happens, then call this a leaf and be done with it.
  if(left.empty() || right.empty()) { 
    makeLeaf();
    return;
  }
  
  // -- See which weak classifiers that leak into this region might also leak into the child regions.
  for(size_t i=0; i<consider_.size(); ++i) {
    if(USE_QP) { 
      int lc2=0, rc2=0;
      // -- Use QP solver to find which wcs belong in the consider list.
      double dist_left = computePointToPolytopeDistanceSquared(region_a_left, region_b_left, consider_[i]->center_);
      double dist_right = computePointToPolytopeDistanceSquared(region_a_right, region_b_right, consider_[i]->center_);

      // -- Add to consider lists.
      if(dist_left <= consider_[i]->theta_) {
	left_consider.push_back(consider_[i]);
	lc2++;
      }
      if(dist_right <= consider_[i]->theta_) {
	right_consider.push_back(consider_[i]);
	rc2++;
      }
    }

    else { 
      // -- Fast heuristic version that is basically the same.
      int rc=0, lc=0;
      double dist = a_.dot(consider_[i]->center_) - b_;
      double dist2 = dist*dist;
      if(dist == 0) {
	rc++;
	lc++;
	right_consider.push_back(pwcs_[i]);
	left_consider.push_back(pwcs_[i]);
      }
      else if(dist > 0) {
	rc++;
	right_consider.push_back(consider_[i]);
	if(dist2 <= consider_[i]->theta_) {
	  lc++;
	  left_consider.push_back(consider_[i]);
	}
      }
      else {
	lc++;
	left_consider.push_back(consider_[i]);
	if(dist2 <= consider_[i]->theta_) {
	  rc++;
	  right_consider.push_back(consider_[i]);
	}
      }
    }
  }
      
  // -- Create the split.
//   double left_removed = pwcs_.size() + consider_.size() - left.size() - left_consider.size();
//   double right_removed = pwcs_.size() + consider_.size() - right.size() - right_consider.size();
  lchild_ = new WeakClassifierTree(left, left_consider, recursion_limit_, depth_+1, region_a_left, region_b_left);
  rchild_ = new WeakClassifierTree(right, right_consider, recursion_limit_, depth_+1, region_a_right, region_b_right);
} 


void WeakClassifierTree::makeLeaf() {
  is_leaf_ = true;
  all_ = pwcs_;
  all_.insert(all_.end(), consider_.begin(), consider_.end());
}

void WeakClassifierTree::query(const Eigen::VectorXf& sample, float len_sq,
		   std::vector<WeakClassifier*>* activated, int* num_euc, int* num_dot) {
  assert(activated->empty());
  assert(sample.rows() == pwcs_[0]->center_.rows());
  if(depth_ == 0)
    activated->reserve(pwcs_.size());

  // -- Descend the tree.
  if(!is_leaf_) {
    float atx = a_.dot(sample);
    ++(*num_dot);

//     if(atx == b_) {
//       cerr << "whoa" << endl;
//       exit(0);
//     }
    if(atx > b_)
      rchild_->query(sample, len_sq, activated, num_euc, num_dot);
    else //Non-perfect handling of edge cases, but this isn't going to matter.
      lchild_->query(sample, len_sq, activated, num_euc, num_dot);
    
//     else {
//       vector<WeakClassifier*> l_activated, r_activated;
//       lchild_->query(sample, len_sq, activated, num_euc, num_dot);
//       rchild_->query(sample, len_sq, r_activated, num_euc, num_dot);
//       activated->insert(activated->end(), r_activated.begin(), r_activated.end());
//     }
  }

  // -- Once in the leaf, find all weak classifiers that are activated.
  else {
    *num_euc = all_.size();
    for(size_t i=0; i<all_.size(); ++i)
      if(fastEucSquared(sample, all_[i]->center_, len_sq, all_[i]->length_squared_) <= all_[i]->theta_)
	activated->push_back(all_[i]);
  }
}

WeakClassifierTree::~WeakClassifierTree() {
  if(lchild_)
    delete lchild_;
  if(rchild_)
    delete rchild_;
}


//! Use a QP solver to determine the distance from a point to a polytope.
double computePointToPolytopeDistanceSquared(const vector<VectorXf>& As, const vector<float>& Bs, const VectorXf& point) {
  assert(Bs.size() == As.size());
  
  // -- Put all variables into a form that qpOASES can use.
  int num_rows = As.size();
  int num_cols = As[0].rows(); //I know, it's weird.
  double A[num_rows * num_cols];
  for(int i=0; i<num_rows; ++i) {
    assert(As[i].rows() == num_cols);
    for(int j=0; j<num_cols; ++j) {
      A[i*num_cols + j] = As[i](j);
    }
  }

  double ubA[Bs.size()];
  for(size_t i=0; i<Bs.size(); ++i) {
    ubA[i] = Bs[i];
  }

  double g[point.rows()];
  for(int i=0; i<point.rows(); ++i) {
    g[i] = -point(i);
  }

  // -- Run the solver.
  int nWSR = 100;
  QProblem solver(point.rows(), As.size(), HST_IDENTITY); //Our Hessian = I.
  solver.setPrintLevel(PL_NONE);
  returnValue rv = solver.init(NULL, g, A, NULL, NULL, NULL, ubA, nWSR, NULL);
  if(rv != SUCCESSFUL_RETURN) {
    cout << "Bad solve!" << endl;
    assert(0);
  }

  double xopt[point.rows()];
  solver.getPrimalSolution(xopt);

  // -- Put xopt into eigen.
  VectorXf closest_in_poly = VectorXf::Zero(point.rows());
  for(int i=0; i<closest_in_poly.rows(); ++i) {
    closest_in_poly(i) = xopt[i];
  }
  
  for(size_t i=0; i<As.size(); ++i) {
    assert(As[i].dot(closest_in_poly) - Bs[i] <= 1e-4);
  }

  
  // -- Return the distance.
  double dist=0;
  for(int i=0; i<point.rows(); ++i) {
    double tmp = (point(i) - xopt[i]);
    dist += tmp * tmp;
  }

  return dist;
}
