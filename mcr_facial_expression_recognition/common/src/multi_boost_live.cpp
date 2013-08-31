#include "multi_boost_live.h"

MultiBoostLive::MultiBoostLive(const string &modelFilename, vector1Di &allLabels, int verbose)
{
	int argc = 4;
	char* argv[4];

	argv[0] = "MultiBoost_example.exe";
	argv[1] = "-test";
	argv[2] = "sat.tst";
	argv[3] = (char*) modelFilename.c_str();

	// no need to synchronize with C style stream
	std::ios_base::sync_with_stdio(false);

	//////////////////////////////////////////////////////////////////////////
	// Standard arguments
	nor_utils::Args args;

	args.setArgumentDiscriminator("-");

	args.declareArgument("help");
	args.declareArgument("static");

	args.declareArgument("h", "Help", 1, "<optiongroup>");

	//////////////////////////////////////////////////////////////////////////
	// Basic Arguments

	args.setGroup("Parameters");

	args.declareArgument("train", "Performs training.", 2, "<dataFile> <nInterations>");
	args.declareArgument("traintest", "Performs training and test at the same time.", 3, "<trainingDataFile> <testDataFile> <nInterations>");
	args.declareArgument("test", "Test the model.", 2, "<dataFile> <shypFile>");
	args.declareArgument("test", "Test the model and output the results", 3, "<datafile> <shypFile> <outFile>");
	args.declareArgument("cmatrix", "Print the confusion matrix for the given model.", 2, "<dataFile> <shypFile>");
	args.declareArgument("cmatrixfile", "Print the confusion matrix with the class names to a file.", 3, "<dataFile> <shypFile> <outFile>");

	args.declareArgument("ssfeatures", "Print matrix data for SingleStump-Based weak learners (if numIters=0 it means all of them).", 4,
	                     "<dataFile> <shypFile> <outFile> <numIters>");

	//////////////////////////////////////////////////////////////////////////
	// Options

	args.setGroup("General Options");

	args.declareArgument("verbose", "Set the verbose level 0, 1 or 2 (0=no messages, 1=default, 2=all messages).", 1, "<val>");
	args.declareArgument("outputinfo", "Output informations on the algorithm performances during training, on file <filename>.", 1, "<filename>");

	//////////////////////////////////////////////////////////////////////////
	// Shows the list of available learners
	string learnersComment = "Available learners are:";

	vector<string> learnersList;
	BaseLearner::RegisteredLearners().getList(learnersList);
	vector<string>::const_iterator it;
	for (it = learnersList.begin(); it != learnersList.end(); ++it)
	{
		learnersComment += "\n ** " + *it;
		// defaultLearner is defined in Defaults.h
		if (*it == defaultLearner)
			learnersComment += " (DEFAULT)";
	}

	args.declareArgument("learnertype", "Change the type of weak learner. " + learnersComment, 1, "<learner>");

	//////////////////////////////////////////////////////////////////////////
	//// Declare arguments that belongs to all weak learners
	BaseLearner::declareBaseArguments(args);

	////////////////////////////////////////////////////////////////////////////
	//// Weak learners (and input data) arguments
	for (it = learnersList.begin(); it != learnersList.end(); ++it)
	{
		args.setGroup(*it + " Options");
		// add weaklearner-specific options
		BaseLearner::RegisteredLearners().getLearner(*it)->declareArguments(args);
	}

	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////

	switch (args.readArguments(argc, argv))
	{
		case nor_utils::AOT_NO_ARGUMENTS:
			showBase();
			break;

		case nor_utils::AOT_UNKOWN_ARGUMENT:
			exit(1);
			break;

		case nor_utils::AOT_INCORRECT_VALUES_NUMBER:
			exit(1);
			break;
	}

	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////

	if (args.hasArgument("help"))
		showHelp(args, learnersList);
	if (args.hasArgument("static"))
		showStaticConfig();

	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////

	if (args.hasArgument("h"))
		showOptionalHelp(args);

	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////

	if (args.hasArgument("verbose"))
		args.getValue("verbose", 0, verbose);

	// -test <dataFile> <shypFile>
	string testFileName = args.getValue < string > ("test", 0);
	string shypFileName = args.getValue < string > ("test", 1);

	_classifier = new Classifier(args, shypFileName, allLabels, verbose);
}

MultiBoostLive::~MultiBoostLive(void)
{
	//delete _classifier;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/**
 * Show the basic output. Called when no argument is provided.
 * @date 11/11/2005
 */
void MultiBoostLive::showBase()
{
	cout << "MultiBoost (v" << CURRENT_VERSION << "). An obvious name for a multi-class AdaBoost learner." << endl;
	cout << "---------------------------------------------------------------------------" << endl;
	cout << "Build: " << __DATE__ << " (" << __TIME__ << ") (C) Norman Casagrande 2005-2006" << endl << endl;
	cout << "===> Type -help for help or -static to show the static options" << endl;

	exit(0);
}

//---------------------------------------------------------------------------

/**
 * Show the help. Called when -h argument is provided.
 * @date 11/11/2005
 */
void MultiBoostLive::showHelp(nor_utils::Args& args, const vector<string>& learnersList)
{
	cout << "MultiBoost (v" << CURRENT_VERSION << "). An obvious name for a multi-class AdaBoost learner." << endl;
	cout << "------------------------ HELP SECTION --------------------------" << endl;

	args.printGroup("Parameters");

	cout << endl;
	cout << "For specific help options type:" << endl;
	cout << "   -h general: General options" << endl;
	cout << "   -h io: I/O options" << endl;
	cout << "   -h algo: Basic algorithm options" << endl;

	cout << endl;
	cout << "For weak learners specific options type:" << endl;

	vector<string>::const_iterator it;
	for (it = learnersList.begin(); it != learnersList.end(); ++it)
		cout << "   -h " << *it << endl;

	exit(0);
}

//---------------------------------------------------------------------------

/**
 * Show the help for the options.
 * @param args The arguments structure.
 * @date 28/11/2005
 */
void MultiBoostLive::showOptionalHelp(nor_utils::Args& args)
{
	string helpType = args.getValue < string > ("h", 0);

	cout << "MultiBoost (v" << CURRENT_VERSION << "). An obvious name for a multi-class AdaBoost learner." << endl;
	cout << "---------------------------------------------------------------------------" << endl;

	if (helpType == "general")
		args.printGroup("General Options");
	else if (helpType == "io")
		args.printGroup("I/O Options");
	else if (helpType == "algo")
		args.printGroup("Basic Algorithm Options");
	else if (BaseLearner::RegisteredLearners().hasLearner(helpType))
		args.printGroup(helpType + " Options");
	else
		cerr << "ERROR: Unknown help section <" << helpType << ">" << endl;
}

//---------------------------------------------------------------------------

/**
 * Show the default values.
 * @date 11/11/2005
 */
void MultiBoostLive::showStaticConfig()
{
	cout << "MultiBoost (v" << CURRENT_VERSION << "). An obvious name for a multi-class AdaBoost learner." << endl;
	cout << "------------------------ STATIC CONFIG -------------------------" << endl;

	cout << "- Sort type = ";
#if CONSERVATIVE_SORT
	cout << "CONSERVATIVE (slow)" << endl;
#else
	cout << "NON CONSERVATIVE (fast)" << endl;
#endif

	cout << "Comment: " << COMMENT << endl;
#ifndef NDEBUG
	cout << "Important: NDEBUG not active!!" << endl;
#endif

#if MB_DEBUG
	cout << "MultiBoost debug active (MB_DEBUG=1)!!" << endl;
#endif

	exit(0);
}

int MultiBoostLive::runLive(vector1Df &testData)
{
	int label = _classifier->runLive(testData);

	return label;
}

