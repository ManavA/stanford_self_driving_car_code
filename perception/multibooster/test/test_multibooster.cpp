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
#include <multibooster/synth_data.h>
#include <gtest/gtest.h>

using namespace Eigen;
using namespace std;

#define NUM_CLASSES 10


TEST(MultiBooster, long_training) {
  SynthDatasetGenerator sdg;
  sdg.num_classes_ = 5;
  sdg.num_feature_spaces_ = 10;
  sdg.num_objs_ = 5000;
  sdg.num_bg_ = 5000;
  sdg.feature_probability_ = 1;
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);
  mb.verbose_ = true;
  clock_t start = clock();
  mb.train(500, 0, 1);
  cout << "Training took " << (double)(clock() - start) / (double)CLOCKS_PER_SEC << " seconds." << endl;
}  

TEST(MultiBooster, produces_same_classifier_every_time) {
//   srand(0);
//   SynthDatasetGenerator sdg;
//   sdg.num_classes_ = NUM_CLASSES;
//   MultiBoosterDataset* mbd = sdg.generateDataset();
//   mbd->save("test/example.mbd"); //Done once.

  MultiBoosterDataset *mbd2 = new MultiBoosterDataset("test/example.mbd");
  MultiBooster mb(mbd2);
  srand(0);
  mb.train(10, 0, 10);
  //mb.save("test/example.mb"); //Done once.

  MultiBooster mb2("test/example.mb");
  EXPECT_TRUE(mb.compare(mb2, true));
  delete mbd2;
}

WeakClassifier getWC() {
  WeakClassifier wc;
  wc.id_ = 12;
  wc.fsid_ = 13;
  wc.center_ = VectorXf::Random(10);
  wc.length_squared_ = 33.123125;
  wc.theta_ = 24523.2341;
  wc.vals_ = VectorXf::Random(13);
  return wc;
}

TEST(WeakClassifierTree, correctness) {
  SynthDatasetGenerator sdg;
  sdg.num_classes_ = 2;
  sdg.num_feature_spaces_ = 1;
  sdg.num_objs_ = 5000;
  sdg.num_bg_ = 5000;
  sdg.feature_probability_ = 1;
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);
  //mb.verbose_ = true;
  mb.train(1, 0, 200);

  MultiBoosterDataset* mbd2 = sdg.generateDataset();
  for(size_t i=0; i<mbd2->objs_.size(); ++i) {
    VectorXf result1 = mb.classify(*mbd2->objs_[i]);
    VectorXf result2 = mb.treeClassify(*mbd2->objs_[i]);
    //cout << result1.transpose() - result2.transpose() << endl;
    for(int j=0; j<result1.rows(); ++j)
      ASSERT_NEAR(result1(j), result2(j), 1e-4);
  }
  delete mbd;
  delete mbd2;
}

TEST(helperFns, eigenSerialization) {
  MatrixXf mat = MatrixXf::Random(10,10);
  string matstr = serializeMatrix(mat);
  istringstream iss(matstr);
  MatrixXf mat2;
  deserializeMatrix(iss, &mat2);
  EXPECT_TRUE(mat2 == mat);
//   cout << "--------" << endl;
//   cout << mat << endl;
//   cout << "--------" << endl;
//   cout << mat2 << endl;
//   cout << "--------" << endl;

  VectorXf vec = VectorXf::Random(10);
  string vecstr = serializeVector(vec);
  istringstream iss2(vecstr);
  VectorXf vec2;
  deserializeVector(iss2, &vec2);
  EXPECT_TRUE(vec2 == vec);
//   cout << "--------" << endl;
//   cout << vec << endl;
//   cout << "--------" << endl;
//   cout << vec2 << endl;
//   cout << "--------" << endl;
}

TEST(WeakClassifier, serialization) {
  WeakClassifier wc = getWC();
  string serial = wc.serialize();
  //  cout << serial << endl;
  istringstream iss(serial);
  WeakClassifier wc2(iss);
  EXPECT_TRUE(wc.compare(wc2, true));
  //  cout << wc.serialize() << endl;
  //  cout << wc2.serialize() << endl;
}

TEST(WeakClassifier, copyAndCompare) {
  WeakClassifier wc = getWC();
  WeakClassifier wc2(wc);
  EXPECT_TRUE(wc.compare(wc2));
  wc.theta_ = wc.theta_+1;
  EXPECT_FALSE(wc.compare(wc2));
}


TEST(MultiBooster, objectiveComputationConsistency) {
  srand(0);
  SynthDatasetGenerator sdg;
  sdg.num_classes_ = NUM_CLASSES;
  //  sdg.num_objs_ = 10;
  //  sdg.num_bg_ = 10;
  
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);

  mb.train(10, 0, 10);
  double obj1 = mb.computeObjective();
  double obj2 = mb.classify(mbd);
  cout << setprecision(10) << "classify vs log_weights_: " << obj2 << " " << obj1 << endl;
  EXPECT_FLOAT_EQ(obj1, obj2);
  delete mbd;
}

TEST(MultiBooster, loadAndSave) {  
  // -- Get a small dataset and train.
  srand(0);
  SynthDatasetGenerator sdg;
  sdg.num_classes_ = NUM_CLASSES;
  //  sdg.num_objs_ = 10;
  //  sdg.num_bg_ = 10;
  
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);
//   cout << "classifier map" << endl;
//   cout << mb.class_map_.serialize() << endl;
//   cout << "dataset map" << endl;
//   cout << mbd->class_map_.serialize() << endl;
//   cout << "prior" << endl;
//   cout << mb.prior_ << endl;
//   cout << "num_objs_of_class_" << endl;
//   for(size_t c=0; c<mbd->num_objs_of_class_.size(); ++c) { 
//     cout << mbd->num_objs_of_class_[c] << endl;
//   }

  mb.train(10, 0, 10);
  mb.save("test.mb");
  MultiBooster mb2("test.mb");
  EXPECT_TRUE(mb.compare(mb2, true));
  double obj = mb.classify(mbd);
  double obj2 = mb2.classify(mbd);
  EXPECT_FLOAT_EQ(obj, obj2);
  delete mbd;
}

TEST(MultiBooster, classifyUnlabeled) {
  MultiBooster mb("test.mb");
  SynthDatasetGenerator sdg;
  sdg.labeled_ = false;
  MultiBoosterDataset* mbd = sdg.generateDataset();

  bool exploded = false;
  try {
    mb.useDataset(mbd);
  }
  catch (int e) {
    exploded = true;
  }
  EXPECT_TRUE(exploded);

  exploded = false;
  try {
    mb.classify(mbd);
  }
  catch (int e) {
    exploded = true;
  }
  EXPECT_TRUE(exploded);

  mb.classify(*mbd->objs_[0]); //Just make sure this doesn't crash.
  delete mbd;
}


TEST(MultiBooster, copyAndCompare) {
  // -- Get a small dataset and train.
  srand(0);
  SynthDatasetGenerator sdg;
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);
  mb.train(10, 0, 10);

  MultiBooster mb2(mb);
  EXPECT_TRUE(mb.compare(mb2));
  delete mbd;
}


TEST(MultiBooster, learnOnEasyDataset) {
  srand(0);
  SynthDatasetGenerator sdg;
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);
  mb.train(100, 0, 20);
  cout << mb.status() << endl;
  EXPECT_TRUE(mb.classify(mbd) < 0.1);
  delete mbd;
}

// TEST(MultiBooster, trainingWithDifferentSizeNamespaces) {
//   srand(0);
//   SynthDatasetGenerator sdg;
//   sdg.num_classes_ = 2;
//   sdg.num_feature_spaces_ = 2;
//   MultiBoosterDataset* mbd = sdg.generateDataset();
//   sdg.num_classes_ = 3;
//   sdg.num_feature_spaces_ = 3;
//   MultiBoosterDataset* mbd2 = sdg.generateDataset();
//   mbd2->join(*mbd);
//   MultiBooster mb(mbd);
//   mb.train(5, 0, 50);
  
  

TEST(MultiBooster, resumeLearningWithDifferentNameMapping) {
  srand(0);
  SynthDatasetGenerator sdg;
  sdg.num_feature_spaces_ = 8;
  sdg.num_classes_ = 2;
  sdg.num_bg_ = 100;
  MultiBoosterDataset* mbd = sdg.generateDataset(); 
  MultiBooster mb(mbd);
  //  mb.verbose_ = true;
  mb.train(10, 0, 100);
  EXPECT_TRUE(mb.classify(mbd) < .001);
  cout << "Original performance on dataset 1: " << mb.classify(mbd) << endl;
  
  sdg.num_classes_ = 3; //Add a new class.
  MultiBoosterDataset* mbd2 = sdg.generateDataset();
  mbd2->join(*mbd);
  EXPECT_FALSE(mbd->feature_map_.compare(mbd2->feature_map_));

  mb.useDataset(mbd2);
  mb.relearnResponses();
  mb.resumeTraining(10, 0, 100);

  EXPECT_TRUE(mb.classify(mbd2) < .001);
  cout << "Original performance on dataset 2: " << mb.classify(mbd2) << endl;

  EXPECT_TRUE(mb.classify(mbd) < .001);
  cout << "New performance on dataset 1: " << mb.classify(mbd) << endl;
  delete mbd;
  delete mbd2;
}

TEST(MultiBooster, relearnResponses) {
  // -- Get a small dataset and train.
  srand(0);
  SynthDatasetGenerator sdg;
  sdg.num_classes_ = 10;
  sdg.num_feature_spaces_ = 10;
  sdg.num_objs_ = 30;
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);
  mb.train(10, 0, 100);

  // -- Get the *same* dataset, but with more data, and higher variance.
  srand(0);
  sdg.num_objs_ = 1000;
  sdg.variance_ = 10;
  MultiBoosterDataset* mbd2 = sdg.generateDataset();
  EXPECT_TRUE(mbd->feature_map_.compare(mbd2->feature_map_));
  EXPECT_TRUE(mbd->class_map_.compare(mbd2->class_map_));

  // -- Mix up the names in the new dataset.
  vector<string> classes = mbd2->class_map_.getIdToNameMapping();
  permuteNames(&classes);
  NameMapping cm(classes);
  vector<string> feature_spaces = mbd2->feature_map_.getIdToNameMapping();
  permuteNames(&feature_spaces);
  NameMapping fm(feature_spaces);
  mbd2->applyNewMappings(cm, fm);
  EXPECT_FALSE(mbd->feature_map_.compare(mbd2->feature_map_));
  EXPECT_FALSE(mbd->class_map_.compare(mbd2->class_map_));
//   cout << mbd->class_map_.serialize() << endl;
//   cout << mbd2->class_map_.serialize() << endl;

  // -- Make sure results are good on the small dataset and bad on the large one.
  float obj = mb.classify(mbd);
  EXPECT_TRUE(obj < .001);
  float obj2 = mb.classify(mbd2);
  EXPECT_TRUE(obj2 > .1);


  // -- Relearn responses on the large dataset, and make sure the results are good for both datasets afterwards.
  cout << " Small dataset: " << obj << endl;
  cout << " Large dataset: " << obj2 << endl;
  cout << "Objective on large dataset before learning prior: " << mb.classify(mbd2) << endl;
  cout << "Relearning responses." << endl;
  mb.useDataset(mbd2);
  mb.relearnResponses();
  obj = mb.classify(mbd);
  obj2 = mb.classify(mbd2);
  cout << " Small dataset: " << obj << endl;
  cout << " Large dataset: " << obj2 << endl;
  EXPECT_TRUE(obj < .01);
  EXPECT_TRUE(obj2 < .01);

  delete mbd;
  delete mbd2;
}  


TEST(MultiBooster, classifyDatasetWithSubsetOfKnownClasses) {
  // -- Train on a 3-class dataset.
  srand(0);
  SynthDatasetGenerator sdg;
  MultiBoosterDataset* mbd3 = sdg.generateDataset();
  sdg.num_classes_ = 2;
  MultiBoosterDataset* mbd2 = sdg.generateDataset();
  mbd3->join(*mbd2);
  MultiBooster mb(mbd3);
  mb.train(5, 0, 200);

  float obj1 = mb.classify(mbd3);
  cout << "Performance on 3-class, full dataset: " << obj1 << endl;
  EXPECT_TRUE(obj1 < 0.01);
  
  // -- Classify a 2-class subset of the original dataset.
  float obj2 = mb.classify(mbd2);
  cout << "Performance on 2-class subset: " << obj2 << endl;
  EXPECT_TRUE(obj2 < 0.01);

  delete mbd2;
  delete mbd3;
}
  
// TEST(MultiBooster, prior) {
//   // -- Get a small dataset and train.
//   srand(0);
//   SynthDatasetGenerator sdg;
//   sdg.num_bg_ = 0;
//   sdg.num_classes_ = 2;
//   MultiBoosterDataset* mbd = sdg.generateDataset();
//   MultiBooster mb(mbd);
//   mb.train(10, 0, 10);

//   // -- Now get the same with more background.
//   srand(0);
//   sdg.num_bg_ = 4000;
//   MultiBoosterDataset* mbd2 = sdg.generateDataset();
//   cout << mb.classify(mbd2) << endl;
//   EXPECT_TRUE(mb.classify(mbd2) > .5);
//   mb.useDataset(mbd2); // This will learn the prior.
//   cout << mb.classify(mbd2) << endl;
//   EXPECT_TRUE(mb.classify(mbd2) < .5);

//   delete mbd2;
//   delete mbd;
// }

TEST(MultiBooster, emptyObj) {
  MultiBooster mb("test.mb");
  SynthDatasetGenerator sdg;
  sdg.num_classes_ = NUM_CLASSES;
  sdg.num_bg_ = 0;
  sdg.num_objs_ = 1;
  sdg.feature_probability_ = 0;
  MultiBoosterDataset* mbd = sdg.generateDataset();
  VectorXf res = mb.classify(*mbd->objs_[0]);
  EXPECT_TRUE((size_t)res.rows() == sdg.num_classes_);
  EXPECT_TRUE(res == mb.prior_);

  delete mbd;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
