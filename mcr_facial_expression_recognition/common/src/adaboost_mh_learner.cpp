/*
 * This file is part of MultiBoost, a multi-class
 * AdaBoost learner/classifier
 *
 * Copyright (C) 2005-2006 Norman Casagrande
 * For informations write to nova77@gmail.com
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <ctime> // for time
#include <cmath> // for exp
#include <fstream> // for ofstream of the step-by-step data
#include <limits>
#include <iomanip> // setprecision
#include "utils/utils.cpp" // for addAndCheckExtension
#include "defaults.h" // for defaultLearner
#include "io/output_info.cpp"
#include "io/input_data.cpp"
#include "io/serialization.cpp" // to save the found strong hypothesis
#include "weak_learners/base_learner.cpp"
#include "adaboost_mh_learner.h"

namespace MultiBoost
{

// -----------------------------------------------------------------------------------

	AdaBoostMHLearner::AdaBoostMHLearner(nor_utils::Args& args, int verbose)
			: _args(args),
			  _maxTime(-1),
			  _theta(0),
			  _verbose(verbose),
			  _outputInfoFile(""),
			  _resumeShypFileName(""),
			  _smallVal(1E-10)
	{
		// The file with the step-by-step information
		if (args.hasArgument("outputinfo"))
			args.getValue("outputinfo", 0, _outputInfoFile);

		///////////////////////////////////////////////////
		// get the learner type, if given
		_basicLearnerName = defaultLearner;  // default learner. Defined in Defaults.h

		if (args.hasArgument("learnertype"))
		{
			string learnerString = args.getValue < string > ("learnertype", 0);

			if (!BaseLearner::RegisteredLearners().hasLearner(learnerString))
			{
				// Not found in the registered!
				cerr << "ERROR: learner <" << learnerString << "> not found in the registered learners!" << endl;
				exit(1);
			}
			else
				_basicLearnerName = learnerString;
		}

		if (_verbose > 1)
			cout << "--> Using learner: " << _basicLearnerName << endl;

		///////////////////////////////////////////////////
		// get the output strong hypothesis file name, if given
		if (args.hasArgument("shypname"))
			args.getValue("shypname", 0, _shypFileName);
		else
			_shypFileName = string(SHYP_NAME);

		_shypFileName = nor_utils::addAndCheckExtension(_shypFileName, SHYP_EXTENSION);

		///////////////////////////////////////////////////
		// Set time limit
		if (args.hasArgument("timelimit"))
		{
			args.getValue("timelimit", 0, _maxTime);
			if (_verbose > 1)
				cout << "--> Overall Time Limit: " << _maxTime << " minutes" << endl;
		}

		// Set the value of theta
		if (args.hasArgument("edgeoffset"))
			args.getValue("edgeoffset", 0, _theta);

		// Set the filename of the strong hypothesis file in the case resume is
		// called
		if (args.hasArgument("resume"))
			args.getValue("resume", 0, _resumeShypFileName);
	}

// -----------------------------------------------------------------------------------

	void AdaBoostMHLearner::run(int numIterations, vector3Df &trainData, vector1Di &labels)
	{
		time_t startTime, currentTime;
		time(&startTime);

		// get the registered weak learner (type from name)
		BaseLearner* pWeakHypothesisSource = BaseLearner::RegisteredLearners().getLearner(_basicLearnerName);

		// get the training input data, and load it
		InputData* pTrainData = pWeakHypothesisSource->createInputData();
		pTrainData->initOptions(_args);
//	 pTrainData->loadTrain(trainData, labels);

		// get the testing input data, and load it
		InputData* pTestData = NULL;

		// The output information object
		OutputInfo* pOutInfo = NULL;

		if (!_outputInfoFile.empty())
			pOutInfo = new OutputInfo(_outputInfoFile);

		// reload the previously found weak learners if -resume is set.
		// otherwise just return 0
		int startingIteration = resumeWeakLearners();

		Serialization ss(_shypFileName);
		ss.writeHeader(_basicLearnerName);  // this must go after resumeProcess has been called

		// perform the resuming if necessary. If not it will just return
		resumeProcess(ss, pTrainData, pTestData, pOutInfo);

		if (_verbose == 1)
			cout << "Learning in progress..." << endl;

		///////////////////////////////////////////////////////////////////////
		// Starting the AdaBoost main loop
		///////////////////////////////////////////////////////////////////////
		for (int t = startingIteration; t < numIterations; ++t)
		{
			std::cout << "iteration: " << t << std::endl;

			if (_verbose > 1)
				cout << "------- WORKING ON ITERATION " << t << " -------" << endl;

			BaseLearner* pWeakHypothesis = pWeakHypothesisSource->create();
			pWeakHypothesis->initOptions(_args);
			pWeakHypothesis->run(pTrainData);

			// Output the step-by-step informations
			if (pOutInfo)
			{
				pOutInfo->outputIteration(t);
				pOutInfo->outputError(pTrainData, pWeakHypothesis);
				if (pTestData)
					pOutInfo->outputError(pTestData, pWeakHypothesis);
				pOutInfo->outputMargins(pTrainData, pWeakHypothesis);
				pOutInfo->outputEdge(pTrainData, pWeakHypothesis);
				pOutInfo->endLine();
			}

			// Updates the weights and returns the edge
			double gamma = updateWeights(pTrainData, pWeakHypothesis);

			if (_verbose > 1)
			{
				cout << "--> Alpha = " << pWeakHypothesis->getAlpha() << endl;
				cout << "--> Edge  = " << gamma << endl;
			}

			// If gamma <= theta the algorithm must stop.
			// If theta == 0 and gamma is 0, it means that the weak learner is no better than chance
			// and no further training is possible.
			if (gamma <= _theta)
			{
				if (_verbose > 0)
				{
					cout << "Can't train any further: edge = " << gamma << " (with and edge offset (theta)=" << _theta << ")" << endl;
				}
				break;
			}

			// append the current weak learner to strong hypothesis file,
			// that is, serialize it.
			ss.appendHypothesis(t, pWeakHypothesis);

			// Add it to the internal list of weak hypotheses
			_foundHypotheses.push_back(pWeakHypothesis);

			// check if the time limit has been reached
			if (_maxTime > 0)
			{
				time(&currentTime);
				double diff = difftime(currentTime, startTime);  // difftime is in seconds
				diff /= 60;  // = minutes

				if (diff > _maxTime)
				{
					if (_verbose > 0)
						cout << "Time limit of " << _maxTime << " minutes has been reached!" << endl;
					break;
				}
			}  // check for maxtime

		}  // loop on iterations
		   /////////////////////////////////////////////////////////

		   // write the footer of the strong hypothesis file
		ss.writeFooter();

		// Free the two input data objects
		if (pTrainData)
			delete pTrainData;
		if (pTestData)
			delete pTestData;

		if (pOutInfo)
			delete pOutInfo;

		if (_verbose > 0)
			cout << "Learning completed." << endl;
	}

	void AdaBoostMHLearner::run(int numIterations, const string& trainFileName, const string& testFileName)
	{
		time_t startTime, currentTime;
		time(&startTime);

		// get the registered weak learner (type from name)
		BaseLearner* pWeakHypothesisSource = BaseLearner::RegisteredLearners().getLearner(_basicLearnerName);

		// get the training input data, and load it
		InputData* pTrainData = pWeakHypothesisSource->createInputData();
		pTrainData->initOptions(_args);
//   pTrainData->load(trainFileName, IT_TRAIN, _verbose);

		// get the testing input data, and load it
		InputData* pTestData = NULL;
		if (!testFileName.empty())
		{
			pTestData = pWeakHypothesisSource->createInputData();
			pTestData->initOptions(_args);
//      pTestData->load(testFileName, IT_TEST, _verbose);
		}

		// The output information object
		OutputInfo* pOutInfo = NULL;

		if (!_outputInfoFile.empty())
			pOutInfo = new OutputInfo(_outputInfoFile);

		// reload the previously found weak learners if -resume is set.
		// otherwise just return 0
		int startingIteration = resumeWeakLearners();

		Serialization ss(_shypFileName);
		ss.writeHeader(_basicLearnerName);  // this must go after resumeProcess has been called

		// perform the resuming if necessary. If not it will just return
		resumeProcess(ss, pTrainData, pTestData, pOutInfo);

		if (_verbose == 1)
			cout << "Learning in progress..." << endl;

		///////////////////////////////////////////////////////////////////////
		// Starting the AdaBoost main loop
		///////////////////////////////////////////////////////////////////////
		for (int t = startingIteration; t < numIterations; ++t)
		{
			std::cout << "iteration: " << t << std::endl;

			if (_verbose > 1)
				cout << "------- WORKING ON ITERATION " << t << " -------" << endl;

			BaseLearner* pWeakHypothesis = pWeakHypothesisSource->create();
			pWeakHypothesis->initOptions(_args);
			pWeakHypothesis->run(pTrainData);

			// Output the step-by-step informations
			if (pOutInfo)
			{
				pOutInfo->outputIteration(t);
				pOutInfo->outputError(pTrainData, pWeakHypothesis);
				if (pTestData)
					pOutInfo->outputError(pTestData, pWeakHypothesis);
				pOutInfo->outputMargins(pTrainData, pWeakHypothesis);
				pOutInfo->outputEdge(pTrainData, pWeakHypothesis);
				pOutInfo->endLine();
			}

			// Updates the weights and returns the edge
			double gamma = updateWeights(pTrainData, pWeakHypothesis);

			if (_verbose > 1)
			{
				cout << "--> Alpha = " << pWeakHypothesis->getAlpha() << endl;
				cout << "--> Edge  = " << gamma << endl;
			}

			// If gamma <= theta the algorithm must stop.
			// If theta == 0 and gamma is 0, it means that the weak learner is no better than chance
			// and no further training is possible.
			if (gamma <= _theta)
			{
				if (_verbose > 0)
				{
					cout << "Can't train any further: edge = " << gamma << " (with and edge offset (theta)=" << _theta << ")" << endl;
				}
				break;
			}

			// append the current weak learner to strong hypothesis file,
			// that is, serialize it.
			ss.appendHypothesis(t, pWeakHypothesis);

			// Add it to the internal list of weak hypotheses
			_foundHypotheses.push_back(pWeakHypothesis);

			// check if the time limit has been reached
			if (_maxTime > 0)
			{
				time(&currentTime);
				double diff = difftime(currentTime, startTime);  // difftime is in seconds
				diff /= 60;  // = minutes

				if (diff > _maxTime)
				{
					if (_verbose > 0)
						cout << "Time limit of " << _maxTime << " minutes has been reached!" << endl;
					break;
				}
			}  // check for maxtime

		}  // loop on iterations
		   /////////////////////////////////////////////////////////

		   // write the footer of the strong hypothesis file
		ss.writeFooter();

		// Free the two input data objects
		if (pTrainData)
			delete pTrainData;
		if (pTestData)
			delete pTestData;

		if (pOutInfo)
			delete pOutInfo;

		if (_verbose > 0)
			cout << "Learning completed." << endl;
	}

// -------------------------------------------------------------------------

	double AdaBoostMHLearner::updateWeights(InputData* pData, BaseLearner* pWeakHypothesis)
	{
		const int numExamples = pData->getNumExamples();
		const int numClasses = ClassMappings::getNumClasses();

		const double alpha = pWeakHypothesis->getAlpha();

		double Z = 0;  // The normalization factor
		vector<vector<double> > hy(numExamples);
		for (int i = 0; i < numExamples; ++i)
			hy[i].resize(numClasses);

		// recompute weights
		// computing the normalization factor Z
		for (int i = 0; i < numExamples; ++i)
		{
			for (int l = 0; l < numClasses; ++l)
			{
				hy[i][l] = pWeakHypothesis->classify(pData, i, l) *  // h_l(x_i)
				        pData->getBinaryClass(i, l);  // y_i

				Z += pData->getWeight(i, l) *  // w
				        exp(-alpha * hy[i][l]  // -alpha * h_l(x_i) * y_i
				                );
			}  // numClasses
		}  // numExamples

		// The edge. It measures the
		// accuracy of the current weak hypothesis relative to random guessing
		double gamma = 0;

		// Now do the actual re-weight
		// (and compute the edge at the same time)
		for (int i = 0; i < numExamples; ++i)
		{
			for (int l = 0; l < numClasses; ++l)
			{
				//double hy = pWeakHypothesis->classify(pData, i, l) * // h_l(x_i)
				//            pData->getBinaryClass(i, l); // y_i

				double w = pData->getWeight(i, l);

				gamma += w * hy[i][l];

				// The new weight is  w * exp( -alpha * h(x_i) * y_i ) / Z
				pData->setWeight(i, l, w * exp(-alpha * hy[i][l]) / Z);
			}  // numClasses
		}  // numExamples

		return gamma;
	}

// -------------------------------------------------------------------------

	int AdaBoostMHLearner::resumeWeakLearners()
	{
		if (_resumeShypFileName.empty())
			return 0;

		if (_verbose > 0)
			cout << "Reloading strong hypothesis file <" << _resumeShypFileName << ">.." << flush;

		// The class that loads the weak hypotheses
		UnSerialization us;

		// loads them
		us.loadHypotheses(_resumeShypFileName, _foundHypotheses, _verbose);

		if (_verbose > 0)
			cout << "Done!" << endl;

		// return the number of iterations found
		return static_cast<int>(_foundHypotheses.size());
	}

// -------------------------------------------------------------------------

	void AdaBoostMHLearner::resumeProcess(Serialization& ss, InputData* pTrainData, InputData* pTestData, OutputInfo* pOutInfo)
	{

		if (_resumeShypFileName.empty())
			return;

		if (_verbose > 0)
			cout << "Resuming up to iteration " << _foundHypotheses.size() - 1 << "..0%." << flush;

		vector<BaseLearner*>::iterator it;
		int t;

		// simulate the AdaBoost algorithm for the weak learners already found
		for (it = _foundHypotheses.begin(), t = 0; it != _foundHypotheses.end(); ++it, ++t)
		{
			BaseLearner* pWeakHypothesis = *it;

			// append the current weak learner to strong hypothesis file,
			ss.appendHypothesis(t, pWeakHypothesis);
		}

		const int numIters = static_cast<int>(_foundHypotheses.size());
		const int step = numIters / 5;

		// simulate the AdaBoost algorithm for the weak learners already found
		for (it = _foundHypotheses.begin(), t = 0; it != _foundHypotheses.end(); ++it, ++t)
		{
			BaseLearner* pWeakHypothesis = *it;

			// Output the step-by-step informations
			if (pOutInfo)
			{
				pOutInfo->outputIteration(t);
				pOutInfo->outputError(pTrainData, pWeakHypothesis);
				if (pTestData)
					pOutInfo->outputError(pTestData, pWeakHypothesis);
				pOutInfo->outputMargins(pTrainData, pWeakHypothesis);
				pOutInfo->outputEdge(pTrainData, pWeakHypothesis);
				pOutInfo->endLine();
			}

			// Updates the weights and returns the edge
			double gamma = updateWeights(pTrainData, pWeakHypothesis);

			if (_verbose > 1 && (t + 1) % step == 0)
			{
				double progress = static_cast<double>(t) / static_cast<double>(numIters) * 100.0;
				cout << "." << setprecision(2) << progress << "%." << flush;
			}

			// If gamma <= theta there is something really wrong.
			if (gamma <= _theta)
			{
				cerr << "ERROR!" << endl << "Edge smaller than the edge offset (theta). Something must be wrong!" << endl
				     << "Is the data file the same one used during the original training?" << endl;
				exit(1);
			}

		}  // loop on iterations

		if (_verbose > 0)
			cout << "Done!" << endl;

	}

// -------------------------------------------------------------------------

}// end of namespace MultiBoost
