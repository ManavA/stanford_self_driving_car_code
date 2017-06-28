/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Alex Teichman
*********************************************************************/
#include <multibooster/multibooster.h>
#include <signal.h>
#include <Eigen/LU>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;
#define EIGEN_NO_DEBUG

bool g_int = false;

MultiBooster::MultiBooster(MultiBoosterDataset* mbd) :
  version_string_(CLASSIFIER_VERSION),
  class_map_(vector<string>()),
  feature_map_(vector<string>()),
  battery_(vector< vector<WeakClassifier*> >(0)),
  pwcs_(vector<WeakClassifier*>(0)),
  mbd_(0),
  verbose_(false),
  total_objs_seen_(vector<size_t>(mbd->class_map_.size())),
  num_bg_seen_(0),
  prior_(VectorXf::Zero(mbd->class_map_.size())),
  trees_(vector<WeakClassifierTree*>(0)),
  wc_limiter_(0)
{
  useDataset(mbd);
  battery_ = vector< vector<WeakClassifier*> >(feature_map_.size());
}

MultiBooster::MultiBooster(string filename) :
  version_string_(CLASSIFIER_VERSION),
  class_map_(vector<string>()),
  feature_map_(vector<string>()),
  mbd_(0),
  verbose_(false),
  wc_limiter_(0)
{
  ifstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    throw 1;
  }

  bool success = deserialize(f);
  if(!success)
    throw 1;
  
  f.close();
}

MultiBooster::MultiBooster(const MultiBooster& mb) :
  version_string_(mb.version_string_),
  class_map_(mb.class_map_),
  feature_map_(mb.feature_map_),
  mbd_(mb.mbd_),
  verbose_(mb.verbose_),
  total_objs_seen_(mb.total_objs_seen_),
  num_bg_seen_(mb.num_bg_seen_),
  prior_(mb.prior_),
  wc_limiter_(0),
  log_weights_(mb.log_weights_)
{
  // -- Copy weak classifiers.
  pwcs_ = vector<WeakClassifier*>(mb.pwcs_.size(), 0);
  for(size_t t=0; t<mb.pwcs_.size(); ++t) {
    pwcs_[t] = new WeakClassifier(*mb.pwcs_[t]);
  }

  // -- Rebuild battery_.
  battery_ = vector< vector<WeakClassifier*> >(feature_map_.size());
  for(size_t t=0; t<pwcs_.size(); ++t)
    battery_[pwcs_[t]->fsid_].push_back(pwcs_[t]);
}

MultiBooster::~MultiBooster() {
  for(size_t i=0; i<pwcs_.size(); ++i) {
    delete pwcs_[i];
  }
  for(size_t i=0; i<trees_.size(); ++i) {
    delete trees_[i];
  }
}


void MultiBooster::applyNewMappings(const NameMapping& new_class_map, const NameMapping& new_feature_map, bool strict) {

  // -- Make sure these mappings don't delete things.
  for(size_t i=0; i<class_map_.size(); ++i) {
    new_class_map.toId(class_map_.toName(i)); //This will error if all current names don't exist in the new map.
  }
  for(size_t i=0; i<feature_map_.size(); ++i) {
    new_feature_map.toId(feature_map_.toName(i)); //This will error if all current names don't exist in the new map.
  }

  // -- Also make sure there is nothing new in the new mappings.
  if(strict) {
    if(!new_class_map.isPermutation(class_map_))
      cerr << "Class maps do not have the same names!" << endl;
    if(!new_feature_map.isPermutation(feature_map_))
      cerr << "Feature maps do not have the same names!" << endl;
    assert(new_class_map.isPermutation(class_map_));
    assert(new_feature_map.isPermutation(feature_map_));
  }

  // -- Translate weak classifiers.
  NameTranslator class_translator(class_map_, new_class_map);
  NameTranslator feature_translator(feature_map_, new_feature_map);
  for(size_t t=0; t<pwcs_.size(); ++t) {
    pwcs_[t]->translate(class_translator, feature_translator);
  }

  // -- Translate battery and centers, and expand if we are getting new feature spaces.
  vector< vector<WeakClassifier*> > new_battery(feature_translator.size());
  vector<MatrixXf> new_centers(feature_translator.size());
  for(size_t d=0; d<battery_.size(); ++d) {
    size_t remap = feature_translator.toMap2(d);
    new_battery[remap] = battery_[d];
  }
  battery_ = new_battery;

  // -- Translate trees.
  vector<WeakClassifierTree*> trees(feature_translator.size(), NULL);
  for(size_t d=0; d<trees_.size(); ++d) {
    size_t remap = feature_translator.toMap2(d);
    trees[remap] = trees_[d]; //TODO: Causes memory leak?
  }
  trees_ = trees;
  
  // -- Translate log_weights_, unless we don't have one yet. 
  if(log_weights_.cols() != 0) {
    MatrixXd new_lw = MatrixXd::Zero(class_translator.size(), log_weights_.cols());
    for(size_t c=0; c<(size_t)log_weights_.rows(); ++c) {
      size_t remap = class_translator.toMap2(c);
      for(size_t m=0; m<(size_t)log_weights_.cols(); ++m)
	new_lw(remap, m) = log_weights_(c,m);
    }
    log_weights_ = new_lw;
  }

  // -- prior_ and total_objs_seen_.
//   cout << class_map_.serialize() << endl;
//   cout << new_class_map.serialize() << endl;
//   cout << class_translator.size() << endl;
  VectorXf prior = VectorXf::Zero(class_translator.size());
  vector<size_t> total_objs_seen(class_translator.size());
  for(int c=0; c<prior_.rows(); ++c) {
    size_t remap = class_translator.toMap2(c);
    prior(remap) = prior_(c);
    total_objs_seen[remap] = total_objs_seen_[c];
  }
  total_objs_seen_ = total_objs_seen;
  prior_ = prior;


  class_map_ = new_class_map;
  feature_map_ = new_feature_map;
}


void WeakClassifier::translate(const NameTranslator& class_translator, const NameTranslator& feature_translator) {
  // -- Tranlate feature spaces.
  fsid_ = feature_translator.toMap2(fsid_);
  
  // Translate vals_.  If there are new classes, the corresponding vals will be initialized to zero.
  VectorXf new_vals = VectorXf::Zero(class_translator.size());
  assert(new_vals.rows() >= vals_.rows());
  for(size_t c=0; c<(size_t)vals_.rows(); ++c) {
    size_t idx = class_translator.toMap2(c);
    new_vals(idx) = vals_(c);
  }
  vals_ = new_vals;
}

void updateAccumulators(MatrixXd* H, VectorXd* g, const vector<int>& idx) {
  assert(H->rows() == H->cols());
  assert(g->rows() == H->rows());
  
  for(int i=0; i<(int)idx.size(); ++i) {
    (*g)(idx[i])++;
    for(int j=0; j<(int)idx.size(); ++j) {
      (*H)(idx[i], idx[j])++;
    }
  }
}
    
void MultiBooster::updateGradientAndHessian(MultiBoosterDataset* mbd) {
  if(pwcs_.empty())
    return;
  
  augmentAndTranslate(mbd);

  // -- Initialize Hessian and gradient accumulators if we haven't yet.
  if(H_neg_.empty()) {
    cout << "Initializing H and g." << endl;
    assert(H_pos_.empty() && g_pos_.empty() && g_neg_.empty());

    H_neg_.reserve(class_map_.size());
    H_pos_.reserve(class_map_.size());
    g_neg_.reserve(class_map_.size());
    g_pos_.reserve(class_map_.size());
    for(size_t c=0; c<class_map_.size(); ++c) {
      H_neg_.push_back(MatrixXd::Zero(pwcs_.size(), pwcs_.size()));
      H_pos_.push_back(MatrixXd::Zero(pwcs_.size(), pwcs_.size()));
      g_neg_.push_back(VectorXd::Zero(pwcs_.size()));
      g_pos_.push_back(VectorXd::Zero(pwcs_.size()));
    }
  }

  
  clock_t start = clock();
  cout << "Updating gradient and Hessian";  cout.flush();
  for(size_t m=0; m<mbd->objs_.size(); ++m) {
    //cout << m << "/" << mbd->objs_.size() << endl;

    if((int)m%(int)(mbd->objs_.size() / 10) == 0)
      cout << "."; cout.flush();

    // -- Build the activation vector from all feature spaces.
    vector<int> idx;
    idx.reserve(pwcs_.size());

    for(size_t d=0; d<battery_.size(); ++d) {
      vector<WeakClassifier*> act;
      findActivatedWCs(*mbd->objs_[m], d, &act);
      if(act.empty())
	continue;

      for(size_t t=0; t<act.size(); ++t) 
	idx.push_back(act[t]->id_);
    }

    //cout << "idx: " << (double)idx.size() / (double)pwcs_.size() << endl;
    
    // -- For each class, update the gradient and Hessian accumulators.
    for(size_t c=0; c<class_map_.size(); ++c) {       
      if(mbd->ymc_(c,m) == 1) {
	updateAccumulators(&H_pos_[c], &g_pos_[c], idx);
      }
      else if(mbd->ymc_(c,m) == -1) {
	updateAccumulators(&H_neg_[c], &g_neg_[c], idx);
      }
      else
	assert(0);
    }
  }
  cout << " done.  Took " << (double)(clock() - start) / (double) CLOCKS_PER_SEC << " seconds." << endl;
}


void MultiBooster::augmentAndTranslate(MultiBoosterDataset* mbd) {
  // -- Augment our name maps with the dataset's and apply them to all data structures in the classifier.
  NameMapping new_class_map = class_map_;
  new_class_map.augment(mbd->class_map_);
  NameMapping new_feature_map = feature_map_;
  new_feature_map.augment(mbd->feature_map_);
  applyNewMappings(new_class_map, new_feature_map, false);

  // -- Put the dataset into our naming convention.
  mbd->applyNewMappings(class_map_, feature_map_);
} 

//! Loads a dataset for training.
void MultiBooster::useDataset(MultiBoosterDataset* mbd) {
  mbd_ = mbd;
  augmentAndTranslate(mbd);

  // -- Make sure there are no unlabeled objects. (-2s)
  for(size_t m=0; m<mbd_->objs_.size(); ++m) {
    if(mbd_->objs_[m]->label_ <= -2) {
      cerr << "Supervised learning on a dataset with unlabeled data makes no sense." << endl;
      throw 1;
    }
  }
  
  // -- Update statistics.
  for(size_t c=0; c<mbd->num_objs_of_class_.size(); ++c) {
    total_objs_seen_[c] += mbd->num_objs_of_class_[c];
  }
  num_bg_seen_ += mbd->num_bg_;

  // -- Update gradient and Hessian.
  //updateGradientAndHessian();// Making the user do this manually for now.

  // -- Learn prior and initialize the log_weights_ with it.
  learnPrior();
  log_weights_ = MatrixXd::Zero(class_map_.size(), mbd_->objs_.size());
  for(size_t c=0; c<class_map_.size(); ++c) {
    for(size_t m=0; m<mbd_->objs_.size(); ++m) {
      log_weights_(c,m) = -mbd->ymc_(c,m) * prior_[c];
    }
  }
//   cout << prior_ << endl;
//   cout << "lw" << endl;
//   cout << log_weights_.col(log_weights_.cols()-1) << endl;
}

void MultiBooster::balanceResponses() {
  for(size_t c=0; c<class_map_.size(); ++c) {
    assert(prior_(c) == 0); //TODO: Remove this.
    MatrixXd Hessian = exp(-prior_(c)) * H_pos_[c] + exp(prior_(c)) * H_neg_[c];
    VectorXd gradient = -exp(-prior_(c)) * g_pos_[c] + exp(prior_(c)) * g_neg_[c];
    
    // -- Find the step direction.
    cout << "Solving... ";  cout.flush();
    VectorXd new_weights;
    clock_t start = clock();
    //    Hessian.ldlt().solve(-gradient, &new_weights);
new_weights =  Hessian.lu().solve(-gradient);
//    Eigen::PartialPivLU<MatrixXd> lu(Hessian);
//    lu.solve(-gradient, &new_weights);
    cout << " done.  Took " << (double)(clock() - start) / (double) CLOCKS_PER_SEC << " seconds." << endl;
    cout << new_weights.transpose() << endl;
    for(size_t t=0; t<pwcs_.size(); ++t) {
      assert(!isnan(new_weights(t)));
      assert(!isinf(new_weights(t)));
      pwcs_[t]->vals_(c) = new_weights(t);
    }
  }
}

void MultiBooster::balanceResponsesLineSearch(MultiBoosterDataset* mbd) {
  augmentAndTranslate(mbd);

  // -- Compute the responses (without the priors).
  MatrixXf responses = MatrixXf::Zero(class_map_.size(), mbd->objs_.size());
  for(size_t m=0; m<mbd->objs_.size(); ++m) {
    VectorXf tmp = classify(*mbd->objs_[m]);
    responses.col(m) = tmp - prior_;
  }
  
  // -- Do exact line search to find the step length.
  //    This is an unconstrained, univariate optimization problem that cannot be solved analytically,
  //    so Newton's method with backtracking line search is being used.  x is the step size.
  double x = 1; // Step size of large problem we are solving for.
  double alpha = .3;
  double beta = .8;
  double threshold = 1e-6;
  double delta = -1;

  // -- Compute exp(-ymc(prior + x * response)), the weights for this step size.  Also get objective.
  MatrixXf weights = MatrixXf::Zero(class_map_.size(), mbd->objs_.size());
  for(size_t c=0; c<class_map_.size(); ++c)
    for(size_t m=0; m<mbd->objs_.size(); ++m)
      weights(c,m) = exp(-mbd->ymc_(c,m) * (prior_(c) + x*responses(c,m)));
  double obj = weights.sum();

  while(abs(delta) > threshold) {
    cout << "x: " << x << ", obj: " << obj << endl;

    // -- Compute gradient.
    double gradient = 0;
    for(size_t c=0; c<class_map_.size(); ++c)
      for(size_t m=0; m<mbd->objs_.size(); ++m)
	gradient += -mbd->ymc_(c,m) * responses(c,m) * weights(c,m);
      
    // -- Compute Hessian.
    double hess = 0;
    for(size_t c=0; c<class_map_.size(); ++c)
      for(size_t m=0; m<mbd->objs_.size(); ++m)
	hess += weights(c,m) * responses(c,m) * responses(c,m);

    // -- Find the step direction.
    double dir = gradient / hess;
    double t = 1; // Step size for backtracking line search.
    double new_x = x - t*dir;

    // -- Compute the new weights and new objective for this t.
    MatrixXf new_weights = MatrixXf::Zero(class_map_.size(), mbd->objs_.size());
    for(size_t c=0; c<class_map_.size(); ++c)
      for(size_t m=0; m<mbd->objs_.size(); ++m)
	new_weights(c,m) = exp(-mbd->ymc_(c,m) * (prior_(c) + new_x*responses(c,m)));
    double new_obj = new_weights.sum();

    while(new_obj > obj + alpha * t * gradient * dir) {
      t = beta * t;
      new_x = x - t*dir;
      for(size_t c=0; c<class_map_.size(); ++c)
	for(size_t m=0; m<mbd->objs_.size(); ++m)
	  new_weights(c,m) = exp(-mbd->ymc_(c,m) * (prior_(c) + new_x*responses(c,m)));
      new_obj = new_weights.sum();
    }
    x = new_x;
    weights = new_weights;
    assert(new_obj <= obj);
    delta = new_obj - obj;
    obj = new_obj;   
  }

  // -- Apply the step size.
  for(size_t t=0; t<pwcs_.size(); ++t) {
    pwcs_[t]->vals_ *= x;
  }
}
    
void MultiBooster::learnPrior() {
  assert(total_objs_seen_.size() == class_map_.size());
  assert(total_objs_seen_.size() != 0);

  // -- Priors are on.  TODO: Resolve whether this is what I should do or not.
  prior_ = VectorXf::Zero(class_map_.size());
  //return;

  
  size_t total_objs = 0;
  total_objs += num_bg_seen_;
  for(size_t c=0; c<total_objs_seen_.size(); ++c) {
    total_objs += total_objs_seen_[c];
  }
  
  // -- For each class, set the prior with Newton's method.
  double threshold = 1e-6;
  prior_ = VectorXf::Zero(class_map_.size());  
  for(size_t c=0; c<total_objs_seen_.size(); ++c) {
    double positives = total_objs_seen_[c];
    double negatives = total_objs - positives;
    double zpos = 1;
    double zneg = 1;
    double obj = (zneg*positives + zpos*negatives) / (double)total_objs;
    double delta = -1000;
    while(abs(delta) > threshold) {
      // -- Backtracking line search.
      double t = 1;
      double alpha = .3;
      double beta = .8;
      double gradient = zneg*positives - zpos*negatives;
      double hess = zneg*positives + zpos*negatives;
      double dir = gradient / hess;
      double new_prior = prior_(c) + t * dir; 
      zpos = exp(new_prior);
      zneg = exp(-new_prior);
      double new_obj = (zneg*positives + zpos*negatives) / (double)total_objs;
      while(new_obj > obj + alpha * t * gradient * dir) {
	t = beta * t;
	new_prior = prior_(c) + t * dir;
	zpos = exp(new_prior);
	zneg = exp(-new_prior);
	new_obj = (zneg*positives + zpos*negatives) / (double)total_objs;
      }
      prior_(c) = new_prior;
      delta = new_obj - obj;
      //cout << "obj: " << obj << ", delta: " << delta << endl;
      assert(new_obj < obj + 1e15); //Double precision tolerances.
      obj = new_obj;
    }

    // -- Make sure we're at the min.
    for(int i=0; i<1000; ++i) {
      double jiggle = 2.0 * (((double)rand() / (double) RAND_MAX) - 0.5) / 100.0;
      zpos = exp(prior_[c] + jiggle);
      zneg = exp(-prior_[c] - jiggle);
      double jiggled_obj = (zneg*positives + zpos*negatives) / (double)total_objs;
      if(obj - jiggled_obj > 1e-6) {
	//cout << "obj: " << obj << ", jiggled_obj " << jiggled_obj << endl;
      }
      assert(obj - jiggled_obj <= 1e-6);
    }
  }
}

   
bool MultiBooster::save(string filename)
{
  ofstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  f << serialize() << endl;
  f.close();
  return true;
}


WeakClassifier::WeakClassifier(istream& is) :
  serialization_version_(WC_VERSION)
{
  string line;
  is >> line;
  assert(line.compare("weak_classifier") == 0);
  is >> line;
  assert(line.compare("serialization_version") == 0);
  size_t sv;
  is >> sv;
  if(sv != serialization_version_) {
    cerr << "Wrong version of WeakClassifier.  Expected " << serialization_version_ << ", got " << sv << "." << endl;
    throw 1;
  }
  is >> line;
  assert(line.compare("id") == 0);
  is >> id_;
  is >> line;
  is >> fsid_;
  getline(is, line);
  
  getline(is, line);
  assert(line.compare("theta") == 0);
  is.read((char*)&theta_, sizeof(float));
  getline(is, line);
  
  getline(is, line);
  assert(line.compare("length_squared") == 0);
  is.read((char*)&length_squared_, sizeof(float));

  is >> line;
  assert(line.compare("center") == 0);
  size_t num_rows;
  is >> num_rows;
  getline(is, line); // Get off the line before.  >> does not appear to eat the newline at the end, but read() does.
  float* buf = (float*)malloc(sizeof(float)*num_rows);
  is.read((char*)buf, sizeof(float)*num_rows);
  VectorXf tmp = Eigen::Map<VectorXf>(buf, num_rows); //TODO: This copy is probably not necessary.
  center_ = tmp;
  free(buf);

  
  is >> line;
  assert(line.compare("vals") == 0);
  is >> num_rows;
  getline(is, line); 
  float* buf2 = (float*)malloc(sizeof(float)*num_rows);
  is.read((char*)buf2, sizeof(float)*num_rows);
  tmp = Eigen::Map<VectorXf>(buf2, num_rows);  //TODO: This copy is probably not necessary.
  vals_ = tmp;
  free(buf2);
  
  getline(is, line); // Consume the final end-line.
}

string WeakClassifier::serialize() {
  float fbuf = 0;

  ostringstream oss;
  oss << "weak_classifier" << endl;
  oss << "serialization_version" << endl;
  oss << serialization_version_ << endl;
  oss << "id" << endl;
  oss << id_ << endl;
  oss << "fsid" << endl;
  oss << fsid_ << endl;
  oss << "theta" << endl;
  oss.write((char*)&theta_, sizeof(float));
  oss << endl;
  //  oss << setprecision(20) << theta_ << endl;
  oss << "length_squared" << endl;
  //oss << setprecision(20) << length_squared_ << endl;
  oss.write((char*)&length_squared_, sizeof(float));
  oss << endl;
  oss << "center" << endl;
  oss << center_.rows() << endl;
  for(int i=0; i<center_.rows(); i++) {
    fbuf = center_(i);
    oss.write((char*)&fbuf, sizeof(float));
  }
  oss << endl;
  oss << "vals" << endl << vals_.rows() << endl;
  for(int i=0; i<vals_.rows(); ++i) {
    fbuf = vals_(i);
    oss.write((char*)&fbuf, sizeof(float));
  }
  oss << endl;
  return oss.str();
}


string MultiBooster::serialize() {
  ostringstream oss;
  oss << version_string_ << endl;
  oss << class_map_.serialize() << endl;
  oss << feature_map_.serialize() << endl;

  oss << "total_objs_seen" << endl;
  for(size_t c=0; c<total_objs_seen_.size(); ++c) {
    oss << total_objs_seen_[c] << endl;
  }
  oss << "num_bg_seen" << endl;
  oss << num_bg_seen_ << endl;
  oss << "prior" << endl;
  oss << serializeVector(prior_) << endl;
  
  oss << "num_wcs" << endl;
  oss << pwcs_.size() << endl;
  for(size_t t=0; t<pwcs_.size(); ++t) {
    assert(pwcs_[t]->id_ == t);
    oss << pwcs_[t]->serialize() << endl;
  }
  oss << endl;
  return oss.str();
}


bool MultiBooster::deserialize(istream& is) {
  // -- Reset everything.
  mbd_ = NULL;
  battery_.clear();
  pwcs_.clear(); //TODO: this could be a mem leak if you are doing something weird.  Only call this function from the file constructor.
  //log_weights_ = MatrixXd(0,0); // TODO: Again, this could cause bad behavior if log_weights_ is set, then a new classifier is deserialized here - things wouldn't match up.

  string line;
  getline(is, line);
  if(line.compare(version_string_) != 0) {
    cerr << "Log is of the wrong type!" << endl;
    cerr << "First line is: " << line << " instead of " << version_string_ << endl;
    return false;
  }

  // -- Extract the class and feature maps.
  class_map_ = NameMapping(is);
  feature_map_ = NameMapping(is);
  getline(is, line); // Eat the newline that << doesn't get.

  
  // -- Extract statistics.
  total_objs_seen_ = vector<size_t>(class_map_.size());
  is >> line;
  assert(line.compare("total_objs_seen") == 0);
  for(size_t c=0; c<class_map_.size(); ++c) {
    is >> total_objs_seen_[c];
    //    cout << "class " << c << ": " << total_objs_seen_[c] << endl;
  }
  is >> line;
  assert(line.compare("num_bg_seen") == 0);
  is >> num_bg_seen_;
  
  // -- Extract prior.
  is >> line;
  assert(line.compare("prior") == 0);
  deserializeVector(is, &prior_);
  //  cout << "Extracted prior: " << prior_.transpose() << endl;
  assert((size_t)prior_.rows() == class_map_.size());
  assert(prior_.cols() == 1);
  
  // -- Get the weak classifiers.
  size_t num_wcs;
  is >> line;
  assert(line.compare("num_wcs") == 0);
  is >> num_wcs;
  //  cout << num_wcs << endl;
  pwcs_.resize(num_wcs);
  for(size_t t=0; t<num_wcs; ++t) {
    pwcs_[t] = new WeakClassifier(is);
  }

  // -- Post-processing: construct battery_ and centers_
  battery_ = vector< vector<WeakClassifier*> >(feature_map_.size());
  //  centers_ = vector<MatrixXf>(feature_map_.size());
  for(size_t t=0; t<num_wcs; ++t) {
    battery_[pwcs_[t]->fsid_].push_back(pwcs_[t]);
  }
//   for(size_t i=0; i<battery_.size(); ++i) {
//     assert(!battery_[i].empty());
//     centers_[i] = MatrixXf::Zero(battery_[i][0]->center_.rows(), battery_[i].size());
//     for(size_t j=0; j<battery_[i].size(); ++j) {
//       centers_[i].col(j) = battery_[i][j]->center_;
//     }
//   }

  // -- Initialize Hessian and gradient accumulators.  TODO: Make this save and load.
//   assert(H_pos_.empty() && g_pos_.empty() && g_neg_.empty() && H_neg_.empty());
//   H_neg_.reserve(class_map_.size());
//   H_pos_.reserve(class_map_.size());
//   g_neg_.reserve(class_map_.size());
//   g_pos_.reserve(class_map_.size());
//   for(size_t c=0; c<class_map_.size(); ++c) {
//     H_neg_.push_back(MatrixXd::Zero(pwcs_.size(), pwcs_.size()));
//     H_pos_.push_back(MatrixXd::Zero(pwcs_.size(), pwcs_.size()));
//     g_neg_.push_back(VectorXd::Zero(pwcs_.size()));
//     g_pos_.push_back(VectorXd::Zero(pwcs_.size()));
//   }

  // -- Make sure the saved prior and the learned one match.
//   VectorXf saved = prior_;
//   learnPrior();
//   cout << saved.transpose() << endl << prior_.transpose() << endl;
  //assert(saved == prior_);

  computeTrees();
  return true;
}



void MultiBooster::findActivatedWCs(const Object& obj, size_t fsid, vector<WeakClassifier*>* activated) {
  assert(activated->empty());

  vector<WeakClassifier*> &wcs = battery_[fsid];
  activated->reserve(wcs.size());

  if(!obj.descriptors_[fsid].vector || wcs.empty())
    return;

  const descriptor& desc = obj.descriptors_[fsid];

  assert(desc.vector->rows() == wcs[0]->center_.rows());
  
  
  for(size_t t=0; t<wcs.size(); t++) {
    float sd = fastEucSquared(*desc.vector, wcs[t]->center_, desc.length_squared, wcs[t]->length_squared_);

    if(desc.length_squared == 0)
      assert(desc.vector->sum() == 0);
    if(wcs[t]->length_squared_ == 0) {
      assert(wcs[t]->center_.sum() == 0);
    }

    if(sd <= wcs[t]->theta_) {
      activated->push_back(wcs[t]);
    }
  }



  // -- Test 1d random projection heuristic.   // TODO: Remove this.  
  if(false) {   
    vector<bool> consider(wcs.size(), true);

    size_t num_proj = 10;
    for(size_t iproj = 0; iproj < num_proj; ++iproj) { 
      // Get a random 1d subspace.
      VectorXf proj = VectorXf::Zero(wcs[0]->center_.size());
      for(int i=0; i<proj.rows(); ++i) { 
	proj(i) = sampleFromGaussian(1);
      }
      proj.normalize();
    
      // Project the obj & wcs to the 1d space.
      // See how many of the wcs can be eliminated.
      float obj_proj = proj.dot(*desc.vector);
      vector<float> wc_proj(wcs.size());

      for(size_t i=0; i<wcs.size(); ++i) {
	wc_proj[i] = wcs[i]->center_.dot(proj);
	double dist = abs(wc_proj[i] - obj_proj);

	if(dist > sqrt(wcs[i]->theta_)) {
	  consider[i] = false;

	  for(size_t j=0; j<activated->size(); ++j) {
	    WeakClassifier* wc = (*activated)[j];
	    assert(wc == pwcs_[wc->id_]);
	    if(wc->id_ == wcs[i]->id_) {
	      cout << "Bad elimination. " << obj_proj << " " << wc_proj[i] << ", dist: " << abs(obj_proj - wc_proj[i]) << ", sqrt(theta): " << sqrt(wcs[i]->theta_);
	      cout << ", euc: " << sqrt(fastEucSquared(*desc.vector, wc->center_, desc.length_squared, wc->length_squared_)) << endl;
	    }
	  }
	}
      
      }
    }

    // Make sure we didn't eliminate any that actually activate.
    for(size_t i=0; i<wcs.size(); ++i) {
      if(consider[i])
	continue;
    
      for(size_t j=0; j<activated->size(); ++j) {
	WeakClassifier* wc = (*activated)[j];
	if(wc->id_ == wcs[i]->id_) {
	  cout << "Found bad elim after the fact." << endl;
	  cout << "dist & theta: " << fastEucSquared(*desc.vector, wc->center_, desc.length_squared, wc->length_squared_) << " " << wc->theta_ << endl;
	  cout << wc->status(class_map_, feature_map_) << endl;
	  assert(0);
	}
      }
    }

    // Count up how many we have to consider.
    size_t num_consider = 0;
    for(size_t i=0; i<consider.size(); ++i) {
      if(consider[i])
	num_consider++;
    }

    static double accum = 0;
    static double num = 0;
    num++;
    accum += (double)(wcs.size() - num_consider) / (double) wcs.size();
    double pct = accum / num;
  
    cout << "Random projection x" << num_proj << " eliminates " << pct  << " percent of wcs on average" << endl;
  }
}


void sigint(int none) {
  cout << "Caught user signal." << endl;
  g_int = true;
}

//! Relearn the responses for all weak classifiers using a new dataset.  Weak classifier utilities will be 
//! recomputed for this dataset, and those with utility less than min_util will be thrown out.
void MultiBooster::relearnResponses(double min_util, int max_wcs) {
  assert(mbd_);
  assert(mbd_->class_map_.size() == class_map_.size());

  vector<Object*>& objs = mbd_->objs_;
  if(verbose_)
    cout << "Objective before response relearning: " << classify(mbd_) << endl;

  vector<float> utils(pwcs_.size());
  for(size_t t=0; t<pwcs_.size(); ++t) {
    WeakClassifier& wc = *pwcs_[t];
    assert(wc.id_ == t);

    VectorXd numerators = VectorXd::Zero(class_map_.size());
    VectorXd denominators = VectorXd::Zero(class_map_.size());
    
    // -- Find which training examples fall in the hypersphere and increment numerators, denominators, responses, and utility.
    vector<size_t> inside;
    inside.reserve(objs.size());
    VectorXd sum_weights_pos = VectorXd::Zero(class_map_.size());
    VectorXd sum_weights_neg = VectorXd::Zero(class_map_.size());
    for(size_t m=0; m<objs.size(); ++m) {
      descriptor& desc = objs[m]->descriptors_[wc.fsid_];
      
      // -- If no feature of this type, ignore.
      if(!desc.vector)
	continue;
      
      // -- If not in the hypersphere, ignore.
      if(fastEucSquared(wc.center_, *desc.vector, wc.length_squared_, desc.length_squared) > wc.theta_) 
	continue;

      inside.push_back(m);

      for(size_t c=0; c<class_map_.size(); ++c) {
	double weight = exp(log_weights_(c,m));
	numerators(c) += weight * mbd_->ymc_(c,m);
	denominators(c) += weight;
      }
    }

    // -- Set the response values for this weak classifier.
    for(size_t c=0; c<class_map_.size(); ++c) {
      if(denominators(c) == 0)
	wc.vals_(c) = 0;
      else
	wc.vals_(c) = numerators(c) / denominators(c);

      // -- Update the weights for each training example inside the hypersphere.
      for(size_t m=0; m<inside.size(); ++m) {
	size_t idx = inside[m];
	log_weights_(c,idx) += -mbd_->ymc_(c,idx) * wc.vals_(c);
      }
    }

    // -- Compute the utility.
    utils[t] = 0;
    for(size_t c=0; c<class_map_.size(); ++c) {
      for(size_t m=0; m<inside.size(); ++m) { 
	size_t idx = inside[m];
	if(mbd_->ymc_(c,idx) == 1) 
	  sum_weights_pos(c) += exp(log_weights_(c,idx));
	else if(mbd_->ymc_(c,idx) == -1) 
	  sum_weights_neg(c) += exp(log_weights_(c,idx));
	else {
	  cout << "ymc must be in -1, +1" << endl;
	  assert(0);
	}
      }

      utils[t] += (exp(wc.vals_(c)) - 1) * sum_weights_pos(c);
      utils[t] += (exp(-wc.vals_(c)) - 1) * sum_weights_neg(c);
    }

  }

  double objective = classify(mbd_);
  if(verbose_)
    cout << "Objective after response relearning: " << objective << endl;
//  cout << "Objective after response updating: " << computeObjective() << endl;

  // -- Pruning.
  if(min_util > 0) { 
    cout << "WARNING: Minimum utility pruning is not yet implemented." << endl;
  }
  if(max_wcs > 0) {
    vector< pair<double, int> > util_idx(pwcs_.size());
    for(size_t t=0; t<pwcs_.size(); ++t) {
      util_idx[t].first = utils[t];
      util_idx[t].second = t;
    }
    greater< pair<double, int> > emacs = greater< pair<double, int> >();
    sort(util_idx.begin(), util_idx.end(), emacs); //Descending.

    if(verbose_)
      cout << "Pruning " << pwcs_.size() - max_wcs << " weak classifiers." << endl;

    // -- Make new pwcs_ and battery_.
    vector<WeakClassifier*> pwcs_new;
    vector< vector<WeakClassifier*> > battery_new(feature_map_.size());
    pwcs_new.reserve(max_wcs);
    for(size_t t=0; t<pwcs_.size(); ++t) {
      if((int)t < max_wcs) { 
	//cout << "keeping wc " << util_idx[t].second << " with util " << util_idx[t].first << endl;
	WeakClassifier* pwc = pwcs_[util_idx[t].second];
	pwc->id_ = t;

	pwcs_new.push_back(pwc);
	battery_new[pwc->fsid_].push_back(pwc);
      }
      else {
	//cout << "deleting wc " << util_idx[t].second << " with util " << util_idx[t].first << endl;
	delete pwcs_[util_idx[t].second];
      }
    }


    pwcs_ = pwcs_new;
    battery_ = battery_new;


	 
    double objective_after_pruning = classify(mbd_);
//    cout << "same: " << objective_after_pruning - objective << " " << min_util_in_classifier << endl;
    if(verbose_) { 
      cout << "Objective after pruning: " << objective_after_pruning << endl;    	 
      cout << status() << endl;
    }
    //cout << "Relearning responses." << endl;
    //    relearnResponses(mbd); // TODO: Should instead be able to decrement log_weights_ for those pruned, then recompute responses without redoing euc distances.
  }
}

void MultiBooster::recomputeLogWeights() {
  assert(mbd_);
  vector<Object*>& objs = mbd_->objs_;
  assert(class_map_.size() > 0);
  assert(objs.size() > 0);
  log_weights_ = MatrixXd::Zero(class_map_.size(), objs.size());
  for(size_t m=0; m<objs.size(); ++m) {
    VectorXf response = classify(*objs[m]);
    for(size_t c=0; c<class_map_.size(); ++c) {
      log_weights_(c,m) = -response(c) * mbd_->ymc_(c,m);
    }
  }
}

void MultiBooster::resumeTraining(int num_candidates, int max_secs, int max_wcs, double min_util, void (*debugHook)(WeakClassifier)) {
  assert(mbd_);
  recomputeLogWeights();
  train(num_candidates, max_secs, max_wcs, min_util, debugHook);
}


void MultiBooster::train(int num_candidates, int max_secs, int max_wcs, double min_util, void (*debugHook)(WeakClassifier)) {
  signal(SIGINT,sigint);
  time_t start, end;
  time(&start);
  float obj, obj2;
  
  obj = computeObjective();
  obj2 = classify(mbd_);

  if(verbose_) {
    cout << setprecision(20) << "Objective (from log_weights_): " << obj << endl;
    cout << setprecision(20) << "Objective (from classify())  : " << obj2 << endl;
  }
  assert(abs(obj - obj2) / min(obj, obj2) < 1e-3);
  

  int wcs=0;
  double checkpoint_interval = 1800; // Seconds.
  int checkpoint_num = 1;
  while(true) {
    float util = 0;
    learnWC(num_candidates, &util);
    wcs++;
    time(&end);
    if(debugHook != NULL)
      debugHook(*pwcs_.back());

    obj2 = computeObjective();

    //cout << wcs << " wcs.  " << obj << " " << obj2 << endl;
    assert(obj2 < obj);

    if(verbose_) {
      cout << "Total weak classifiers: " << pwcs_.size() << endl;
      cout << "Objective (from log_weights_): " << computeObjective() << endl;
    }

    if(max_secs != 0 && difftime(end,start) > max_secs) {
      if(verbose_) 
	cout << "Ending training because max time has been reached." << endl;
      break;
    }
    if(max_wcs != 0 && wcs >= max_wcs) {
      if(verbose_) 
	cout << "Ending training because max number of weak classifiers has been reached." << endl;
      break;
    }
    if(g_int) {
      if(verbose_) 
	cout << "Ending training because of user control-c." << endl;
      break;
    }
    if(util < min_util) {
      if(verbose_) 
	cout << "Ending training because of min utility criterion: " << util << " < min util of " << min_util << endl;
      break;
    }

    // -- Autosave every half hour.
    if(floor((double)difftime(end,start) / checkpoint_interval) == checkpoint_num) {
      checkpoint_num++;
      string savename = "multibooster-autosave.d";
      if(verbose_) 
	cout << "Autosaving classifier to " << savename << endl;
      save(savename);
    }
  }

  computeTrees();
  
  if(verbose_) {
    MatrixXd log_weights;
    cout << "Objective (from classify()): " << classify(mbd_, 0.0, NULL, &log_weights) << endl;
    //    cout << (log_weights_ - log_weights).transpose() << endl;
    cout << "Done training." << endl;
  }

  // -- Test response balancing.
//   updateGradientAndHessian(mbd_);
//   balanceResponses();
//   cout << "Objective (from classify(), after response balancing): " << classify(mbd_) << endl;
//   balanceResponsesLineSearch(mbd_);
//   cout << "Objective (from classify(), after line search): " << classify(mbd_) << endl;

//   save("tmp.d");
//   MultiBooster mb;
//   mb.load("tmp.d");
//   cout << "Objective (from classify() of saved and loaded): " << mb.classify(*mbd_) << endl;
//   cout << "Compare? " << compare(mb, true) << endl;
//   cout << status() << endl;
//   cout << mb.status() << endl;
}

bool WeakClassifier::compare(const WeakClassifier& wc, bool verbose) {
  double tol = 1e-4;
  if(serialization_version_ != wc.serialization_version_)
    return false;
  if(fsid_ != wc.fsid_)
    return false;

  if(abs(length_squared_ - wc.length_squared_) > tol) {
    cout << "length_squared_ " << length_squared_ << " " << wc.length_squared_ << endl;
    return false;
  }
  if(abs((center_ - wc.center_).dot(center_ - wc.center_)) > tol)
    return false;
  
  if(abs(theta_ - wc.theta_) > tol) {
    if(verbose) {
      cout << "theta for wc " << id_ << ": " << theta_ << endl;
      cout << "theta for wc " << wc.id_ << ": "  << wc.theta_ << endl;
      cout << "Difference: " << theta_ - wc.theta_ << endl;
    }
    return false;
  }

  if(vals_.rows() != wc.vals_.rows()) {
    if(verbose)
      cout << "size of vals" << endl;
    return false;
  }
  
  for(size_t c=0; c<(size_t)vals_.rows(); ++c) {
    if(abs(vals_(c) - wc.vals_(c)) > tol) {
      if(verbose) {
	cout << "Vals are different." << endl;
	cout << "Orig: " << vals_.transpose() << endl;
	cout << "Other: " << wc.vals_.transpose() << endl;
      }
      return false;
    }
  }
  if(id_ != wc.id_) {
    if(verbose)
      cout << "ids: " << id_ << " " << wc.id_ << endl;
    return false;
  }
  return true;
}

//! Comprehensive with high probability, but not guaranteed.
bool MultiBooster::compare(const MultiBooster& mb, bool verbose) {
  if(pwcs_.size() != mb.pwcs_.size()) {
    if(verbose)
      cout << "Different number of weak classifiers!" << endl;
    return false;
  }

  if(!class_map_.compare(mb.class_map_))
    return false;
  if(!feature_map_.compare(mb.feature_map_))
    return false;

  if(num_bg_seen_ != mb.num_bg_seen_) {
    if(verbose)
      cout << "Different num_bg_seen_" << endl;
    return false;
  }
  
  for(size_t c=0; c<class_map_.size(); ++c) {
    if(total_objs_seen_[c] != mb.total_objs_seen_[c]) {
      if(verbose)
	cout << "Different total_objs_seen_" << endl;
      return false;
    }
    if(!close(prior_[c], mb.prior_[c], 1e-3)) {
      if(verbose)
	cout << "Different priors.  " << prior_.transpose() << ", " << mb.prior_.transpose() << endl;
      return false;
    }
  }
   
  for(size_t i=0; i<pwcs_.size(); ++i) {
    if(!pwcs_[i]->compare(*mb.pwcs_[i], verbose)) {
      if(verbose) {
	cout << "Different wcs." << endl;
	cout << pwcs_[i]->status(class_map_, feature_map_) << endl;
	cout << mb.pwcs_[i]->status(mb.class_map_, mb.feature_map_) << endl;
	cout << pwcs_[i]->serialize() << endl;
	cout << mb.pwcs_[i]->serialize() << endl;
      }
      return false;
    }
  }
  return true;
}

vector< shared_ptr<WeakClassifier> > MultiBooster::createRandomWeakClassifiers(int num_candidates) {
  assert(mbd_);

  // -- Get the weights matrix.
  MatrixXd weights = log_weights_.array().exp().matrix();

  // -- Float accuracy makes these numbers match only to 3 digits.  Double -> 12 digits.
//   double total_weight = 0;
//   for(int i=0; i<weights.cols(); i++) {
//     total_weight += weights.col(i).sum();
//   }
//   cout << "sum: " << weights.sum() << ", total_weight: " << total_weight << endl;
    
  // -- Choose wc candidates from the weights distribution.
  vector< shared_ptr<WeakClassifier> > cand;
  for(int iCand=0; iCand<num_candidates; iCand++) {
    double dice01 = ((double)rand() / (double)RAND_MAX);
    double dice =  dice01 * weights.sum(); //The weights aren't necessarily normalized.
    int obj_id=-1;

    for(int i=0; i<weights.cols(); i++) {
      double colsum = weights.col(i).sum();
      dice -= colsum;
      
      if(dice <= 0) {
	obj_id = i;
	break;
      }
    }

    // -- Handle (very rare) case of roundoff causing no weak classifier center to be chosen. 
    if(obj_id == -1)
      obj_id = weights.cols() - 1;
    
    
    // cout << "obj id " << obj_id << ", objs: " << weights.cols() << ", dice: " << dice << ", dice01: " << dice01 << endl;
    // assert(obj_id >= 0 && obj_id < weights.cols());
    
    
    // -- If the object doesn't have any descriptors, move on.
    Object& obj = *mbd_->objs_[obj_id];
    bool valid = false;
    for(size_t i=0; i<obj.descriptors_.size(); ++i) {
      if(obj.descriptors_[i].vector) {
	valid = true;
	break;
      }
    }
    if(!valid) {
      //cout << "Warning: Object " << obj_id << " has no descriptors." << endl;
      iCand--;
      continue;
    }

    // -- Get a random feature space from the object.
    VectorXf* v = NULL;
    size_t fsid = 0;
    while(!v) {
      fsid = rand() % obj.descriptors_.size();
      v = obj.descriptors_[fsid].vector;
    }

    //Make the weak classifier.
    shared_ptr<WeakClassifier> wc(new WeakClassifier());
    wc->fsid_ = fsid;
    wc->center_ = *v;
    wc->theta_ = 0;
    wc->length_squared_ = 0;
    wc->vals_ = VectorXf::Zero(mbd_->class_map_.size());
    wc->id_ = -1;
    cand.push_back(wc);
  }
  return cand;
}


std::string MultiBooster::status(bool verbose)
{
  char tmp[1000];
  string st("Classifier Status: \n");
  if(wc_limiter_ != 0) { 
    sprintf(tmp, "  ** Using the first %d weak classifiers\n", wc_limiter_);
    st.append(tmp);
  }

  st.append("  nClasses: ");
  sprintf(tmp, "%d \n", class_map_.size());
  st.append(tmp);

  st.append("  Class priors:\n");
  for(size_t i=0; i<class_map_.size(); ++i) {
    sprintf(tmp, "    %f %s \n", prior_[i], class_map_.toName(i).c_str());
    st.append(tmp);
  }
  
  st.append("  Number of weak classifiers: ");
  sprintf(tmp, "%zd \n", pwcs_.size());
  st.append(tmp);

  if(pwcs_.size() == 0)
    return st;

  vector< pair<size_t, size_t> > nwcs_idx(feature_map_.size());
  for(size_t i=0; i<battery_.size(); ++i) {
    nwcs_idx[i].first = battery_[i].size();
    nwcs_idx[i].second = i;
  }
  greater< pair<double, int> > emacs = greater< pair<double, int> >();
  sort(nwcs_idx.begin(), nwcs_idx.end(), emacs); // Descending.

  st.append("  nWeakClassifiers \t Name \n");

  // -- Print descriptor spaces in order of most number of weak classifiers.
  for(size_t i=0; i<nwcs_idx.size(); ++i) {
    sprintf(tmp, "  %zd \t\t\t %s \n", nwcs_idx[i].first, feature_map_.toName(nwcs_idx[i].second).c_str());  
    st.append(tmp);
  }

  if(verbose) {
    for(size_t i=0; i<pwcs_.size(); ++i) {
      cout << pwcs_[i]->status(class_map_, feature_map_) << endl;
    }
  }
 
  return st;
}

void MultiBooster::learnWC(int num_candidates, float* util) {
  assert(mbd_);

  time_t start, end;
  time(&start);

  vector< shared_ptr<WeakClassifier> > cand = createRandomWeakClassifiers(num_candidates);
  if(verbose_) 
    cout << "Added " << num_candidates << " candidate wcs" << endl;

  // -- Create pipeline nodes for each candidate.
  vector< shared_ptr<pipeline::ComputeNode> > plnodes(cand.size());
  vector< shared_ptr<WCEvaluator> > nodes(cand.size());
  for(size_t i = 0; i < cand.size(); ++i) {
    nodes[i] = shared_ptr<WCEvaluator>(new WCEvaluator(this, mbd_, cand[i]));
    plnodes[i] = nodes[i];
  }

  // -- Evaluate all candidates in parallel.
  int num_threads = 1;
  if(getenv("NUM_THREADS"))
    num_threads = atoi(getenv("NUM_THREADS"));
  pipeline::Pipeline pl(plnodes, num_threads);
  pl.compute();
  if(verbose_)
    cout << endl << endl;

  // -- Choose the best one.
  size_t best_idx = 0;
  float max_utility = -FLT_MAX;
  for(size_t i = 0; i < nodes.size(); ++i) {
    if(nodes[i]->utility_ > max_utility) { 
      max_utility = nodes[i]->utility_;
      best_idx = i;
    }
  }

  // -- Add the new weak classifier.
  if(verbose_) 
    cout << "Found weak classifier with utility " << max_utility << endl;
  assert(max_utility > 0);
  WeakClassifier* best = new WeakClassifier(*nodes[best_idx]->wc_);
  best->length_squared_ = best->center_.dot(best->center_);
  best->id_ = pwcs_.size();
  battery_[best->fsid_].push_back(best);
  pwcs_.push_back(best);

  // -- Compute the new log weights.
  for(int m=0; m<nodes[best_idx]->num_contained_tr_ex_; ++m) {
    int idx = (*nodes[best_idx]->distance_idx_)[m].second;
    for(size_t c=0; c<mbd_->class_map_.size(); ++c) {
      log_weights_(c, idx) += -mbd_->ymc_(c, idx) * best->vals_(c);
    }
  }
  
  
  // -- Display stats about the weak classifier.
  time(&end);
  if(verbose_) {
    cout << "Took " << difftime(end,start) << " seconds to try " << num_candidates << " wcs." << endl;
    cout << best->status(class_map_, feature_map_) << endl;
    cout << "WC encompasses at least one point from " << nodes[best_idx]->num_contained_tr_ex_ << " out of " << mbd_->objs_.size() << " objects." << endl;
  }

  *util = max_utility;
}


WeakClassifier::WeakClassifier() :
  serialization_version_(WC_VERSION)
{
}

WeakClassifier::WeakClassifier(const WeakClassifier& wc) :
  serialization_version_(wc.serialization_version_),
  fsid_(wc.fsid_),
  center_(wc.center_),
  length_squared_(wc.length_squared_),
  theta_(wc.theta_),
  vals_(wc.vals_),
  id_(wc.id_)
{
}

string WeakClassifier::status(const NameMapping& class_map, const NameMapping& feature_map) const {
  ostringstream oss(ostringstream::out);
  oss << feature_map.toName(fsid_) << " feature." << endl;
  oss << "Id: " << id_ << endl;
  oss << "Theta: " << theta_ << endl;
  oss << "Vals: " << endl;
  for(size_t i=0; i<class_map.size(); ++i)
    oss << "  " << vals_(i) << "  " << class_map.toName(i) << endl;
  
  return oss.str();
}


double MultiBooster::computeObjective() {
  double obj = 0; //log_weights_.cwise().exp().sum() / (log_weights_.rows() * log_weights_.cols());
  assert((size_t)log_weights_.cols() == mbd_->objs_.size());
  assert((size_t)log_weights_.rows() == class_map_.size());
  for(size_t m=0; m<mbd_->objs_.size(); ++m) {
    for(size_t c=0; c<class_map_.size(); ++c) {
      obj += exp(log_weights_(c,m)) / (double)(mbd_->objs_.size() * class_map_.size());
    }
  }
  //  obj /=  (double)(mbd_->objs_.size() * class_map_.size());
//   cout << log_weights_.rows() << " " << log_weights_.cols() << ". obj " << obj << endl;
//   cout << log_weights_ << endl;
  return obj;
}


VectorXf MultiBooster::classify(Object &obj) {
  VectorXf response = prior_;
  
  for(size_t d=0; d<battery_.size(); ++d) {
    vector<WeakClassifier*> act;
    findActivatedWCs(obj, d, &act);
    for(size_t a = 0; a<act.size(); a++) {
      if(wc_limiter_ != 0 && act[a]->id_ > wc_limiter_)
	continue;
      response += act[a]->vals_;
    }
    //cout << "regular (" << d << "): " << act.size() << endl;
  }
  

  return response;
}

VectorXf MultiBooster::treeClassify(Object &obj, int* num_euc, int* num_dot, vector<WeakClassifier*>* activations) {
  if(trees_.empty()) {
    assert(pwcs_.size() == 0);
  }
  VectorXf response = prior_;  

  if(num_euc && num_dot) {
    *num_euc = 0;
    *num_dot = 0;
  }

  assert(trees_.size() == obj.descriptors_.size());
  assert(trees_.size() == battery_.size());
  for(size_t d=0; d<trees_.size(); ++d) {
    assert(d <= obj.descriptors_.size() - 1);
    descriptor& desc = obj.descriptors_[d];
    if(!desc.vector)
      continue;
    if(!trees_[d])
      continue;
    
    int this_num_euc = 0;
    int this_num_dot = 0;
    vector<WeakClassifier*> act;
    trees_[d]->query(*desc.vector, desc.length_squared, &act, &this_num_euc, &this_num_dot);
    if(num_euc && num_dot) {
      *num_euc += this_num_euc;
      *num_dot += this_num_dot;
    }
    for(size_t a = 0; a<act.size(); a++) {
      if(wc_limiter_ != 0 && act[a]->id_ > wc_limiter_)
	continue;
      response += act[a]->vals_;
    }

    // -- Return the activated weak classifiers.
    if(activations)
      *activations = act;
    
  }
  
  return response;
}


void MultiBooster::computeStatisticsForDataset(MultiBoosterDataset* mbd) {
  augmentAndTranslate(mbd);

  // -- Set up log_weights with prior.  Note this is NOT using log_weights_ in the class.
  MatrixXd log_weights = MatrixXd::Zero(class_map_.size(), mbd->objs_.size());
  for(size_t c=0; c<class_map_.size(); ++c) {
    for(size_t m=0; m<mbd->objs_.size(); ++m) {
      log_weights(c,m) = -mbd->ymc_(c,m) * prior_[c];
    }
  }

  
  // -- Get number of training examples inside each weak classifier and build log_weights.
  vector<double> inside(pwcs_.size());

  for(size_t t=0; t<inside.size(); ++t) {
    for(size_t m=0; m<mbd->objs_.size(); ++m) {
      descriptor& desc = mbd->objs_[m]->descriptors_[pwcs_[t]->fsid_];
      if(desc.vector) {
	float sd = fastEucSquared(*desc.vector, pwcs_[t]->center_, desc.length_squared, pwcs_[t]->length_squared_);
	if(sd <= pwcs_[t]->theta_) {
	  inside[t]++;
	  for(size_t c=0; c<class_map_.size(); ++c) { 
	    log_weights(c,m) += -mbd->ymc_(c,m) * pwcs_[t]->vals_(c);
	  }
	}
      }
    }
    //    cerr << inside[t] << " " << endl; // TODO: make this not a hack.
  }


  // -- Set up storage for indices.
  vector< pair<double, size_t> > wc_idx(pwcs_.size());
  for(size_t t=0; t<inside.size(); ++t) {
    wc_idx[t].first = 0;
    wc_idx[t].second = t;
  }

  // -- Increment utilities.
  for(size_t t=0; t<inside.size(); ++t) {
    for(size_t m=0; m<mbd->objs_.size(); ++m) {
      descriptor& desc = mbd->objs_[m]->descriptors_[pwcs_[t]->fsid_];
      if(desc.vector) {
	float sd = fastEucSquared(*desc.vector, pwcs_[t]->center_, desc.length_squared, pwcs_[t]->length_squared_);
	if(sd <= pwcs_[t]->theta_) {
	  for(size_t c=0; c<class_map_.size(); ++c) { 
	    double util = exp(log_weights(c,m)) * (exp(mbd->ymc_(c,m) * pwcs_[t]->vals_(c)) - 1) / (class_map_.size() * mbd->objs_.size());
	    wc_idx[t].first += util;
	  }
	}
      }
    }
  }

  // -- Sort by utility.
  greater< pair<double, size_t> > emacs = greater< pair<double, size_t> >();
  sort(wc_idx.begin(), wc_idx.end(), emacs); //Descending.
  
  // -- Print out the top 10 weak classifiers.
//   size_t top_k = 10;
//   for(size_t t=0; t<top_k; ++t) {
//     cout << " ----- " << endl;
//     cout << "Weak classifier " << t << " with utility " << wc_idx[t].first << endl;
//     cout << pwcs_[wc_idx[t].second]->status(class_map_, feature_map_) << endl;
//   }  
}

double computeDensity(const vector<WeakClassifier*>& pwcs) {
  MatrixXf overlap = MatrixXf::Zero(pwcs.size(), pwcs.size());
  for(size_t i=0; i<pwcs.size(); ++i) {
    for(size_t j=0; j<pwcs.size(); ++j) {
      double dist = fastEucSquared(pwcs[j]->center_, pwcs[i]->center_, pwcs[j]->length_squared_, pwcs[i]->length_squared_);
      dist = sqrt(dist);
      if(dist <= sqrt(pwcs[i]->theta_) + sqrt(pwcs[j]->theta_)) {
	overlap(i,j) = 1;
	overlap(j,i) = 1;
      }
    }
  }

  double density = overlap.sum() / (double)(pwcs.size() * pwcs.size());
  return density;
}

void MultiBooster::computeTrees() {
  clock_t start = clock();
  trees_.clear();
  trees_.resize(battery_.size(), NULL);
  for(size_t i=0; i<battery_.size(); ++i) {
    if(battery_[i].empty())
      continue;
    else 
      trees_[i] = new WeakClassifierTree(battery_[i], 10);
  }
  cout << "Took " << (double)(clock() - start) / (double)CLOCKS_PER_SEC << " seconds to build " << trees_.size() << " trees." << endl;

//   for(size_t i=0; i<battery_.size(); ++i) {
//     if(!battery_[i].empty())
//       cout << "Density of overlap matrix with dim " << battery_[i][0]->center_.rows() << " is " << computeDensity(battery_[i]) << endl;
//  }
}


double MultiBooster::classify(MultiBoosterDataset* mbd, float threshold, PerfStats* results, MatrixXd* log_weights, bool tree) {
  // -- Make sure that all the names agree, etc. 
  augmentAndTranslate(mbd);

  if(log_weights)
    *log_weights = MatrixXd::Zero(class_map_.size(), mbd->objs_.size());
  
  // -- Compute the responses for all objects.
  double objective=0.0;
  VectorXf response = VectorXf::Zero(class_map_.size());
  for(size_t m=0; m<mbd->objs_.size(); ++m) {
    if(mbd->objs_[m]->label_ <= -2) { 
      cerr << "Getting an objective function score on a dataset with unlabeled data makes no sense." << endl;
      throw 1;
    }
    if(tree)
      response = treeClassify(*mbd->objs_[m]);
    else
      response = classify(*mbd->objs_[m]);
    
    if(results)
      results->incrementStats(mbd->objs_[m]->label_, (response.array() - threshold).matrix());
    if(log_weights)
      log_weights->col(m) += (response.cast<double>().array() * -mbd->ymc_.col(m).cast<double>().array()).matrix();
    
    assert((int)class_map_.size() == response.rows());
    for(size_t c=0; c<class_map_.size(); ++c) {
      objective += exp(-mbd->ymc_(c, m) * response(c)) / (double)(mbd->objs_.size() * class_map_.size());
    }
  }
  //objective /= mbd->objs_.size() * class_map_.size();

  return objective;
}

void MultiBooster::classifySpeedTest(MultiBoosterDataset* mbd, PerfStats* results, PerfStats* treeResults, PerfStats* localResults) {
  // -- Make sure that all the names agree, etc. 
  augmentAndTranslate(mbd);
  
  // -- Compute the responses for all objects.
  cout << "Starting regular." << endl;
  clock_t start = clock();
  VectorXf response = VectorXf::Zero(class_map_.size());
  for(size_t m=0; m<mbd->objs_.size(); ++m) {
    if(mbd->objs_[m]->label_ <= -2) { 
      cerr << "Getting an objective function score on a dataset with unlabeled data makes no sense." << endl;
      throw 1;
    }
    response = classify(*mbd->objs_[m]);
    if(results)
      results->incrementStats(mbd->objs_[m]->label_, response);
  }
  cout << "Regular: " << (double)(clock() - start) / (double)CLOCKS_PER_SEC << " sec." << endl;
  
  start = clock();
  double av_euc = 0;
  double av_dot = 0;
  for(size_t m=0; m<mbd->objs_.size(); ++m) {
    if(mbd->objs_[m]->label_ <= -2) { 
      cerr << "Getting an objective function score on a dataset with unlabeled data makes no sense." << endl;
      throw 1;
    }
    int num_euc = 0, num_dot = 0;
    response = treeClassify(*mbd->objs_[m], &num_euc, &num_dot);
    av_euc += num_euc;
    av_dot += num_dot;
    if(treeResults)
      treeResults->incrementStats(mbd->objs_[m]->label_, response);
  }
  cout << "Tree: " << (double)(clock() - start) / (double)CLOCKS_PER_SEC << " sec." << endl;
  av_euc /= (double) mbd->objs_.size();
  av_dot /= (double) mbd->objs_.size();
  cout << "Average number of euc distance computations: " << av_euc << " (vs " << pwcs_.size() << ")." << endl;
  cout << "Average number of dot prods: " << av_dot << endl;
}






    
void deserializeMatrix(istream& is, MatrixXf* target) {
  int rows;
  int cols;
  string str;
  is >> rows;
  is >> cols;
  getline(is, str);
  float* buf = (float*)malloc(sizeof(float)*rows*cols);
  is.read((char*)buf, sizeof(float)*rows*cols);
  MatrixXf tmp = Eigen::Map<MatrixXf>(buf, rows, cols); //TODO: This copy is probably not necessary.
  *target = tmp;
  free(buf);
  getline(is, str);
}

string serializeMatrix(const MatrixXf& mat) {
  ostringstream oss;
  oss << mat.rows() << endl;
  oss << mat.cols() << endl;
  float fbuf = 0;
  for(int i=0; i<mat.cols(); ++i) { 
    for(int j=0; j<mat.rows(); ++j) {
      fbuf = mat(j, i);
      oss.write((char*)&fbuf, sizeof(float));
    }
  }
  oss << endl;
  return oss.str();
}

void deserializeVector(istream& is, VectorXf* target) {
  int rows;
  string str;
  is >> rows;
  getline(is, str);
  float* buf = (float*)malloc(sizeof(float)*rows);
  is.read((char*)buf, sizeof(float)*rows);
  VectorXf tmp = Eigen::Map<VectorXf>(buf, rows); //TODO: This copy is probably not necessary.
  *target = tmp;
  free(buf);
  getline(is, str);
}

string serializeVector(const VectorXf& vec) {
  ostringstream oss;
  oss << vec.rows() << endl;
  float fbuf = 0;
  for(int j=0; j<vec.rows(); ++j) {
    fbuf = vec(j);
    oss.write((char*)&fbuf, sizeof(float));
  }
  oss << endl;
  return oss.str();
}


void PerfStats::incrementStats(int label, const VectorXf& response) {
  assert((size_t)response.rows() == class_map_.size());
  total_test_examples_++;

  assert(label >= -1);  // -2 == unlabeled, and that makes no sense here.
  
  if(label == -1) {
    num_bg_test_examples_++;
  }
  else { 
    num_test_examples_[label]++;
    total_response_[label] += response(label);
  }

  // -- The prediction is the max response, unless no classes get responses greater than 0.
  int prediction = -1;
  float max_response = response.maxCoeff(&prediction);
  if(max_response <= 0)
    prediction = -1;

  if(prediction == label)
    total_correct_++;

  // -- Update confusion matrix, where BG is the last row / col.
  int row = prediction;
  int col = label;
  if(row == -1)
    row = confusion_.rows() - 1;
  if(col == -1)
    col = confusion_.cols() - 1;
  confusion_(row, col)++;
  assert((int)confusion_.diagonal().sum() == total_correct_);
  
  // -- Update {true, false} {positives, negatives}.
  for(int c=0; c<(int)class_map_.size(); ++c) {
    if(label == c && prediction == c)
      tp_[c]++;
    if(label != c && prediction == c)
      fp_[c]++;
    if(label != c && prediction != c)
      tn_[c]++;
    if(label == c && prediction != c)
      fn_[c]++;
  }
}


string PerfStats::statString() {
  ostringstream oss(ostringstream::out);

  // -- Print overall statistics.
  oss << "Total test examples:\t\t" << total_test_examples_ << endl;
  oss << endl << "Total accuracy (correct / total): " << (double)total_correct_ / (double)total_test_examples_ << endl;
  
  // -- Print per-class statistics.
  for(int c=0; c<(int)class_map_.size(); ++c) {
    oss << endl << "--- Class: " << class_map_.toName(c) << endl;
    oss << "Test examples:\t\t\t\t" << num_test_examples_[c] << endl;
    oss << "Average response:\t\t\t" << total_response_[c] / num_test_examples_[c] << endl;
    oss << "True positives:\t\t\t\t" << tp_[c] << endl;
    oss << "True negatives:\t\t\t\t" << tn_[c] << endl;
    oss << "False positives:\t\t\t" << fp_[c] << endl;
    oss << "False negatives:\t\t\t" << fn_[c] << endl;
    assert((int)(tp_[c] + tn_[c] + fp_[c] + fn_[c]) == total_test_examples_);
    oss << "Accuracy:\t\t\t\t" << (tp_[c] + tn_[c]) / (double)total_test_examples_ << endl;
    oss << "Precision (tp/(tp+fp)):\t\t" << tp_[c] / (tp_[c] + fp_[c]) << endl;
    oss << "Recall (tp/(tp+fn)):\t\t" << tp_[c] / (tp_[c] + fn_[c]) << endl;
  }

  // -- Print confusion matrix.
  oss << endl << "Confusion Matrix: " << endl;
  for(size_t c=0; c<class_map_.size(); ++c) {
    oss << class_map_.toName(c) << " ";
  }
  oss << "background " << endl;
  oss << confusion_ << endl;
  return oss.str();
}


double sampleFromGaussian(double stdev) {
  double sum = 0;
  for(size_t i=0; i<12; ++i) {
    sum += 2 * stdev * (double)rand() / (double)RAND_MAX - stdev;
  }
  return 0.5 * sum;
}

 

WCEvaluator::WCEvaluator(MultiBooster *mb, MultiBoosterDataset *mbd, shared_ptr<WeakClassifier> wc) :
  mbd_(mbd),
  mb_(mb),
  wc_(wc),
  utility_(-FLT_MAX),
  num_contained_tr_ex_(-1)
{
}

string WCEvaluator::_getName() const {
  return "WCEvaluator";
}

void WCEvaluator::_flush() {
  mbd_ = NULL;
  mb_ = NULL;
  wc_.reset();

  utility_ = -FLT_MAX;
  distance_idx_.reset();
  num_contained_tr_ex_ = -1;
}

void WCEvaluator::_compute() {
  WeakClassifier wc = *wc_; //wc is the working copy.
  size_t num_classes = mb_->class_map_.size();
  MatrixXd& log_weights = mb_->log_weights_;
  
  // -- Get all distances and sort.
  distance_idx_ = mbd_->computeSortedDistances(wc.fsid_, wc.center_);  // (*distance_idx)[j] is distance, idx (i.e. objs_[idx])
  
  // -- Setup vars for evaluating weak classifiers efficiently.
  vector<double> weight_sum_pos(num_classes); // sum of weight inside hypersphere for each class
  vector<double> weight_sum_neg(num_classes); // sum of weight inside hypersphere for all y_m^c == -1
  VectorXd numerators = VectorXd::Zero(num_classes);
  VectorXd denominators = VectorXd::Zero(num_classes);
    
  // -- For all training examples in order of distance.
  utility_ = -FLT_MAX;
  for(size_t m=0; m<distance_idx_->size(); ++m) {
    if(m>0)
      assert((*distance_idx_)[m-1].first <= (*distance_idx_)[m].first);
      
    int idx = (*distance_idx_)[m].second; // Object idx of the mth closest training example
    double util_this_theta = 0;
    for(size_t c=0; c<num_classes; ++c) {
      // -- Update the response.
      denominators(c) += exp(log_weights(c,idx));
      numerators(c) += exp(log_weights(c,idx)) * mbd_->ymc_(c,idx);
      if(denominators(c) == 0)
	wc.vals_(c) = 0;
      else
	wc.vals_(c) = numerators(c) / denominators(c);

      // -- Update the sum of weights in the hypersphere.
      assert(mbd_->ymc_(c,idx) == 1 || mbd_->ymc_(c,idx) == -1);
      if(mbd_->ymc_(c,idx) == 1) 
	weight_sum_pos[c] += exp(log_weights(c,idx));
      else
	weight_sum_neg[c] += exp(log_weights(c,idx));
	
      util_this_theta += ((1-exp(-wc.vals_(c))) * weight_sum_pos[c] + (1-exp(wc.vals_(c))) * weight_sum_neg[c]) / (float)(mbd_->objs_.size() * num_classes);
    }
    // Doing the division here makes 32bit vs 64bit give different answers.
    // util_this_theta /= objs.size() * class_map_.size();

    // -- If the next object has exactly the same distance, include it too, since it will be included when we call compute.
    if(m < distance_idx_->size() - 1) {
      if(distance_idx_->at(m).first == distance_idx_->at(m+1).first)
	continue;
    }
    // -- If this is the best so far, copy in to wc_.
    if(util_this_theta > utility_) {
      utility_ = util_this_theta;
      //	wc.theta_ = (*distance_idx_)[m].first; // Set theta to be exactly the distance^2 to this training example.
      if(m < distance_idx_->size() - 1) {
	//double dist = (sqrt((*distance_idx_)[m].first) + sqrt((*distance_idx_)[m+1].first)) / 2.0;
	//	  wc.theta_ = dist * dist;
	//wc.theta_ = distance_idx_->at(m).first / 2.0 + distance_idx_->at(m+1).first / 2.0;
	wc.theta_ = distance_idx_->at(m).first + FLT_MIN;
	*wc_ = wc;
	// 	  cout << "mean dist: " << dist << ", dist1^2 " << distance_idx_->at(m).first << ", dist2^2 " << distance_idx_->at(m+1).first << endl;m
	// 	  cout << wc.theta_ << " " << distance_idx_->at(m).first << endl;
	  
	assert(wc_->theta_ >= distance_idx_->at(m).first);
	assert(wc_->theta_ < distance_idx_->at(m+1).first);
      }
      else { 
	double diff = (sqrt((*distance_idx_)[m].first) - sqrt((*distance_idx_)[m-1].first)) / 2.0;
	double dist = sqrt((*distance_idx_)[m].first) + diff;
	wc.theta_ = dist * dist;
	*wc_ = wc;
      }

      num_contained_tr_ex_ = m+1;
    }
  }

  if(mb_->verbose_) {
    cout << "."; cout.flush();
  }
}
