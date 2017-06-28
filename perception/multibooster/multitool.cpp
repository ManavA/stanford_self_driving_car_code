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

int main(int argc, char** argv) {
  // -- Get env var options.  
  int max_wcs = 0;
  if(getenv("MAX_WCS") != NULL)
    max_wcs = atoi(getenv("MAX_WCS"));
  
  int max_secs = 0;
  if(getenv("MAX_SECS") != NULL)
    max_secs = atoi(getenv("MAX_SECS"));
  
  size_t num_candidates = 10;
  if(getenv("NCANDIDATES") != NULL)
    num_candidates = atoi(getenv("NCANDIDATES"));

  double min_util = 0;
  if(getenv("MIN_UTIL") != NULL) 
    min_util = atof(getenv("MIN_UTIL"));

  double threshold = 0;
  if(getenv("THRESHOLD") != NULL) 
    threshold = atof(getenv("THRESHOLD"));

  // -- Parse cmd line args.


  
  if(argc == 4 && !strcmp(argv[1], "--train")) {
    cout << "Training new classifier on " << argv[2] << ", saving with name " << argv[3] << " using " << num_candidates 
	 << " candidates, training until " << max_secs << " seconds, " << max_wcs << " wcs, or min utility " << min_util << endl;
    MultiBoosterDataset mbd(argv[2]);
    MultiBooster mb(&mbd);
    mb.verbose_ = true;
    mb.train(num_candidates, max_secs, max_wcs, min_util);
    mb.save(argv[3]);
  }

  
  
  else if(argc == 4 && !strcmp(argv[1], "--classify")) {
    cout << "Running classifier " << argv[3] <<  " on dataset "  << argv[2] << endl;
    MultiBooster mb(argv[3]);
    MultiBoosterDataset mbd(argv[2]);
    PerfStats results(mb.class_map_);
    cout << "Objective: " << mb.classify(&mbd, threshold, &results) << endl << endl;
    cout << results.statString() << endl;

    //mb.computeStatisticsForDataset(&mbd);
  }

  else if(argc == 4 && !strcmp(argv[1], "--classifySpeedTest")) {
    cout << "Running classifier " << argv[3] <<  " on dataset "  << argv[2] << endl;
    MultiBoosterDataset mbd(argv[2]);
    MultiBooster mb(argv[3]);
    PerfStats results(mb.class_map_);
    PerfStats treeResults(mb.class_map_);
    PerfStats localResults(mb.class_map_);
    mb.classifySpeedTest(&mbd, &results, &treeResults, &localResults);
    cout << "===== Regular results =====" << endl << endl;
    cout << results.statString() << endl << endl;
    cout << "===== Tree results =====" << endl << endl;
    cout << treeResults.statString() << endl;
  }


  
  else if(argc > 4 && !strcmp(argv[1], "--balanceResponses")) {
    cout << "Balancing responses of classifier " << argv[2] << " on dataset " << argv[4] << endl;
    cout << "Saving new classifier as " << argv[3] << endl;

    MultiBoosterDataset mbd(argv[4]);
    MultiBooster mb(argv[2]);
    cout << "Original classification results: " << mb.classify(&mbd) << endl;
    mb.updateGradientAndHessian(&mbd);
    for(int i=5; i<argc; ++i) {
      cout << "Using additional dataset " << argv[i] << endl;
      MultiBoosterDataset mbd_additional(argv[i]);
      mb.updateGradientAndHessian(&mbd_additional);
    }

    mb.balanceResponses();
    cout << "Post-balancing classification results: " << mb.classify(&mbd) << endl;
//     mb.balanceResponsesLineSearch(&mbd);
//     cout << "Post-line-search classification results: " << mb.classify(&mbd) << endl;
    mb.save(argv[3]);
  }

  
  else if(argc == 5 && !strcmp(argv[1], "--relearnResponses")) {
    cout << "Relearning responses on classifier " << argv[3] << " using dataset " << argv[2] 
	 << ", saving new classifier with name " << argv[4] << endl;
    MultiBoosterDataset mbd(argv[2]);
    MultiBooster mb(argv[3]);
    mb.verbose_ = true;
    mb.useDataset(&mbd);
    mb.relearnResponses(0, max_wcs);
    mb.save(argv[4]);
  }

  else if(argc > 4 && !strcmp(argv[1], "--dropFeatures")) {
    cout << "Dropping features from " << argv[2] << ", saving as " << argv[3] << endl;
    MultiBoosterDataset mbd(argv[2]);
    vector<string> droplist;
    cout << "Features to drop: " << endl;
    for(int i=0; i<argc-4; ++i) { 
      cout << argv[4+i] << endl;
      droplist.push_back(argv[4+i]);
    }
    mbd.dropFeatures(droplist);
    mbd.save(argv[3]);
  }

  else if(argc > 4 && !strcmp(argv[1], "--keepFeatures")) {
    cout << "Dropping features from " << argv[2] << ", saving as " << argv[3] << endl;
    MultiBoosterDataset mbd(argv[2]);
    vector<string> keeplist;
    cout << endl << "Features to keep: " << endl;
    for(int i=0; i<argc-4; ++i) { 
      cout << argv[4+i] << endl;
      keeplist.push_back(argv[4+i]);
    }

    // -- Give an error if we are trying to keep features that don't exist.
    vector<string> features = mbd.feature_map_.getIdToNameMapping();
    for(size_t i=0; i<keeplist.size(); ++i) {
      if(find(features.begin(), features.end(), keeplist[i]) == features.end()) { 
	cerr << "Trying to keep " << keeplist[i] << ", which does not exist in the classifier!" << endl;
	return 1;
      }
    }
    
    // -- Build the drop list.
    vector<string> droplist;
    for(size_t i=0; i<features.size(); ++i) {
      if(find(keeplist.begin(), keeplist.end(), features[i]) == keeplist.end())
	droplist.push_back(features[i]);
    }


    if(droplist.empty())
      cout << endl << "Dropping zero features." << endl;
    else
      cout << endl << "Features to drop: " << endl;
    for(size_t i=0; i<droplist.size(); ++i) {
      cout << droplist[i] << endl;
    }

    mbd.dropFeatures(droplist);
    mbd.save(argv[3]);
  }

  
  else if(argc == 5 && !strcmp(argv[1], "--resumeTraining")) {
    cout << "Resuming learning of weak classifiers on " << argv[3] << " using dataset " << argv[2] 
	 << ", saving new classifier with name " << argv[4] << endl;

    MultiBoosterDataset mbd(argv[2]);
    MultiBooster mb(argv[3]);
    mb.useDataset(&mbd);
    mb.resumeTraining(num_candidates, max_secs, max_wcs, min_util);
    mb.save(argv[4]);
  }

  else if(argc == 5 && !strcmp(argv[1], "--cropDataset")) {
    float crop_percent = atof(argv[2]);
    if(crop_percent >= 1 || crop_percent <= 0) {
      cout << "Invalid crop value of " << crop_percent << endl;
      return 1;
    }
    MultiBoosterDataset mbd(argv[3]);
    mbd.crop(crop_percent);
    mbd.save(argv[4]);
  }

  else if(argc == 5 && !strcmp(argv[1], "--decimateDataset")) {
    float decimation = atof(argv[2]);
    if(decimation >= 1 || decimation <= 0) {
      cout << "Invalid decimation value of " << decimation << endl;
      return 1;
    }
    MultiBoosterDataset mbd(argv[3]);
    mbd.decimate(decimation);
    mbd.save(argv[4]);
  }

  else if(argc == 3 && !strcmp(argv[1], "--status")) {
    cout << "Examining " << argv[2] << endl;

    // -- Figure out what it is.
    ifstream file(argv[2]);
    if(!file.is_open()) {
      cout << "Could not open " << argv[2] << endl;
      return 1;
    }
    string line;
    getline(file, line);

    bool verbose = false;
    if(getenv("VERBOSE"))
      verbose = true;
    
    if(line.find(CLASSIFIER_VERSION) != string::npos) {
      MultiBooster mb(argv[2]);
      cout << mb.status(verbose) << endl;
    }

    else if(line.find(DATASET_VERSION) != string::npos) {
      MultiBoosterDataset mbd(argv[2]);
      cout << mbd.status() << endl;
    }

    else {
      cout << "This does not appear to be a multibooster classifier or dataset." << endl;
    }
  }

  else if(argc >= 5 && !strcmp(argv[1], "--join")) {
    cout << "Joining ";
    for(int i=2; i<argc-1; ++i) {
      cout << argv[i] << ", ";
    }
    cout << " into " << argv[argc-1] << endl;
    cout << "Adding " << argv[2] << endl;
    MultiBoosterDataset final(argv[2]);

    for(int i=3; i<argc-1; ++i) {
      cout << "Adding " << argv[i] << endl;
      MultiBoosterDataset additional(argv[i]);
      final.join(additional);
    }
        
    final.save(argv[argc-1]);
  }

  // -- Usage.
  else {
    cout << "usage: " << endl;

    cout << argv[0] << " --status DATASET" << endl;
    cout << endl;
    cout << argv[0] << " --status CLASSIFIER" << endl;
    cout << endl;
    cout << argv[0] << " --train DATASET CLASSIFIER_SAVENAME" << endl;
    cout << " C^c to end training manually." << endl;
    cout << " Environment variable options: " << endl;
    cout << "   MAX_WCS=x is the max number of weak classifiers to add to the boosting classifier. Default infinite." << endl;
    cout << "   MAX_SECS=x is the max number of seconds to train for.  Default infinite." << endl;
    cout << "   MIN_UTIL=x is the minimum utility that an added weak classifier must have to continue training.  Default 0." << endl;
    cout << "   NCANDIDATES=x is the max number of weak classifier candidates to use at each stage. Default 10." << endl;
    cout << endl;

    cout << argv[0] << " --relearnResponses DATASET OLD_CLASSIFIER NEW_CLASSIFIER_SAVENAME" << endl;
    cout << "   MAX_WCS=x is the max number of weak classifiers to allow in the final classifier: the rest will be pruned by utility. Default infinite." << endl;
    cout << endl;

    cout << argv[0] << " --resumeTraining DATASET OLD_CLASSIFIER NEW_CLASSIFIER_SAVENAME" << endl;
    cout << " C^c to end training manually." << endl;
    cout << " Environment variable options: " << endl;
    cout << "   MAX_WCS=x is the max number of weak classifiers to add to the boosting classifier. Default infinite." << endl;
    cout << "   MAX_SECS=x is the max number of seconds to train for.  Default infinite." << endl;
    cout << "   MIN_UTIL=x is the minimum utility that an added weak classifier must have to continue training.  Default 0." << endl;
    cout << "   NCANDIDATES=x is the max number of weak classifier candidates to use at each stage. Default 10." << endl;
    cout << endl;

    cout << argv[0] << " --classify DATASET CLASSIFIER" << endl;
    cout << " Runs CLASSIFIER on DATASET and prints the objective." << endl;
    cout << " Environment variable options: " << endl;
    cout << " THRESHOLD=x is the classification threshold.  Default 0." << endl;
    cout << endl;

    cout << argv[0] << " --join DATASET1 DATASET2 [DATASET3...] DATASET_SAVENAME" << endl;
    cout << endl;
  }
}
