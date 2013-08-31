#include "initializer.h"

Initializer::Initializer(void)
{
}

Initializer::~Initializer(void)
{

}

/**
 *	@brief: 
 *		Detect, from the vector of all expressions, the expressions that are active.
 *	
 *	@param:
 *		active_exp Contains the subspace of active expressions.
 *
 *	@return:
 *		No return.
 **/
void Initializer::logInExpressions(vector<string> &active_exp,					//out
        vector1Di &active_labels,						//out
        vector<string> all_expressions,			//in
        vector1Di all_labels,								//in
        int* exp_to_train)									//in
{
	int nr_all_expressions = (int) all_expressions.size();

	for (int i = 0; i < nr_all_expressions; i++)
	{
		if (exp_to_train[i] == 1)
		{
			active_exp.push_back(all_expressions[i]);
			active_labels.push_back(all_labels[i]);
		}
	}
}

void Initializer::logInTestExpressions(vector<string> &active_exp,			//out
        vector<string> &all_expressions,		//in
        int* exp_to_test)										//in
{
	int nr_all_expressions = (int) all_expressions.size();

	for (int i = 0; i < nr_all_expressions; i++)
	{
		if (exp_to_test[i] == 1)
			active_exp.push_back(all_expressions[i]);
	}
}

/**
 *	@brief:
 *		Make the path to the .txt files containing the ground truth (i.e., information of what frames are neutral
 *			and what frames are prototypic expression).
 *
 *	@param:
 *		- Input:
 *				expressions Vector of string containing the name of the active expressions.
 *
 *		- Output:
 *				path_ground_truth Vector containing the path to the ground truth files.
 */
void Initializer::make_path_ground_truth(vector1Ds &path_ground_truth, vector1Ds &expressions)
{
	if (!path_ground_truth.empty())
		path_ground_truth.clear();

	if (expressions.empty())
		throw "<Error> Vector of active expressions is empty.";

	int nr_expressions = (int) expressions.size();
	string data_name;

	for (int i = 0; i < nr_expressions; i++)
	{
		data_name = "";
		data_name = "../data/mean_frames_10People/meanFrames_" + expressions[i] + ".txt";
		path_ground_truth.push_back(data_name);
	}
}

/**
 *	@brief:
 *		Create path of the training sets for all expressions.
 *
 *	@param:
 *		-Input:
 *			expressions Vector containing the abbreviation of the expressions to be considered.
 *			setNr Defines which training sets to use (from 0 to 9).
 *
 *		-Output:
 *			path_of_data Vector containing the path of all training datasets to employ.
 */
void Initializer::make_path_of_training_data(vector1Ds &path_of_data,				//out
        vector1Ds &expressions,					//in
        int setNr)											//in
{
	if (!path_of_data.empty())
		path_of_data.clear();

	if (expressions.empty())
		throw "<Error> Vector of expression names is empty.";

	if (setNr < 0)
		throw "<Error> setNr is out of range";

	int nr_expressions = (int) expressions.size();
	string data_name;
	std::stringstream ss;

	ss.clear();
	ss.str("");
	ss << setNr;

	for (int i = 0; i < nr_expressions; i++)
	{
		data_name = "";
		data_name = "../data/trainSet/" + expressions[i] + "/trainingSet_" + expressions[i] + "_" + ss.str() + ".txt";
		path_of_data.push_back(data_name);
	}
}

/**
 *	@brief:
 *		Create path of the test sets for all expressions.
 *
 *	@param:
 *		-Input:
 *			expressions Vector containing the abbreviation of the expressions to be considered.
 *			setNr Defines which test sets to use (from 0 to 9).
 *
 *		-Output:
 *			path_of_data Vector containing the path of all test datasets to employ.
 */
void Initializer::make_path_of_test_data(vector1Ds &path_of_data,		//out
        vector1Ds &expressions,			//in
        int setNr)									//in
{
	if (!path_of_data.empty())
		path_of_data.clear();

	if (expressions.empty())
		throw "<Error> Vector of expression names is empty.";

	if (setNr < 0)
		throw "<Error> setNr is out of range";

	int nr_expressions = (int) expressions.size();
	string data_name;

	std::stringstream ss;
	ss.clear();
	ss.str("");
	ss << setNr;

	for (int i = 0; i < nr_expressions; i++)
	{
		data_name = "";
		data_name = "../data/testSet/" + expressions[i] + "/testSet_" + expressions[i] + "_" + ss.str() + ".txt";
		path_of_data.push_back(data_name);
	}
}

void Initializer::make_path_chosenFeatures(vector1Ds &path_ChosenFeats,			//out
        vector1Ds &active_ref_exp,				//in
        int setNr)												//in
{
	int nActiveExpressions = (int) active_ref_exp.size();

	std::stringstream ss;

	ss.clear();
	ss.str("");
	ss << setNr;

	for (int i = 0; i < nActiveExpressions; i++)
		for (int j = i + 1; j < nActiveExpressions; j++)
			path_ChosenFeats.push_back("../data/chosenFeats/Exp" + ss.str() + "/" + active_ref_exp[i] + "_vs_" + active_ref_exp[j] + "_" + ss.str() + ".txt");
}

void Initializer::make_possible_labels(vector1Di active_labels, vector2Di &possible_labels)
{
	int nr_expressions = (int) active_labels.size();

	vector1Di possible_labels_temp(2, 0);

	for (int i = 0; i < nr_expressions; i++)
	{
		for (int j = i + 1; j < nr_expressions; j++)
		{
			possible_labels_temp[0] = active_labels[i];
			possible_labels_temp[1] = active_labels[j];

			possible_labels.push_back(possible_labels_temp);
		}
	}
}

void Initializer::make_classifier_names(vector1Ds &active_classifier_names, vector1Ds &active_cand_exp, int setNr)
{
	int nActiveExpressions = (int) active_cand_exp.size();
	std::stringstream ss;
	ss << setNr;

	for (int i = 0; i < nActiveExpressions; i++)
		for (int j = i + 1; j < nActiveExpressions; j++)
			active_classifier_names.push_back("../data/chosenFeats" + ss.str() + active_cand_exp[i] + "_vs_" + active_cand_exp[j] + ".txt");
}

void Initializer::make_ground_truth_names(vector1Ds &active_ground_truth_names, vector1Ds &active_exp, string prefix, string suffix)
{
	int nActiveExpressions = (int) active_exp.size();

	if ((int) active_ground_truth_names.size() != 0)
		throw "<Error> Vector that will contain the filenames of the ground truth is not empty";
	else if (nActiveExpressions < 1)
		throw "<Error> Number of active test expressions is less than 1.";

	for (int i = 0; i < nActiveExpressions; i++)
		active_ground_truth_names.push_back(prefix + active_exp[i] + suffix + ".txt");
}
