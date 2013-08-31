#pragma once

#include <vector>
#include <string>
#include <map>
#include <iostream>

#include "defaults.h"
#include "utils/args.cpp"

#include "adaboost_mh_learner.cpp"
#include "classifier.cpp"
#include "weak_learners/base_learner.cpp" // To get the list of the registered weak learners
#include "io/class_mappings.cpp" // for -classmap option
#include "general.h"

using namespace std;
using namespace MultiBoost;

class MultiBoostLive
{
 public:
	MultiBoostLive(const string &modelFilename, vector1Di &allLabels, int verbose = 0);

	~MultiBoostLive(void);

	int runLive(vector1Df &testData);

 private:
	Classifier* _classifier;

	void showBase();

	void showHelp(nor_utils::Args& args, const vector<string>& learnersList);

	void showOptionalHelp(nor_utils::Args& args);

	void showStaticConfig();
};

