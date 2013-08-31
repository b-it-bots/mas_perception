#include "general.h"

#pragma once

class Initializer
{
 public:
	Initializer(void);
	~Initializer(void);

	void logInExpressions(vector1Ds &active_exp,			//out
	        vector1Di &active_labels,				//out
	        vector<string> expressions,			//in
	        vector1Di all_labels,						//in
	        int* exp_to_train);							//in

	void logInTestExpressions(vector<string> &active_exp,					//out
	        vector<string> &all_expressions,		//in
	        int* exp_to_test);									//in

	void make_path_of_training_data(vector1Ds &path_of_data,		//out
	        vector1Ds &expressions,				//in
	        int setNr);										//in

	void make_path_of_test_data(vector1Ds &path_of_data,		//out
	        vector1Ds &expressions,				//in
	        int setNr);										//in

	void make_path_ground_truth(vector1Ds &path_ground_truth,		//out
	        vector1Ds &expressions);				//in

	void make_path_chosenFeatures(vector1Ds &path_ChosenFeats,			//out
	        vector1Ds &active_ref_exp,				//in
	        int setNr);												//in

	void make_possible_labels(vector1Di active_labels, vector2Di &possible_labels);

	void make_classifier_names(vector1Ds &active_classifier_names, vector1Ds &active_cand_exp, int setNr);

	void make_ground_truth_names(vector1Ds &active_ground_truth_names, vector1Ds &active_exp, string prefix, string suffix);
};
