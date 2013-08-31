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

#include "io/serialization.cpp"
#include "io/output_info.cpp"
#include "classifier.h"

#include "weak_learners/single_stump_learner.cpp" // for saveSingleStumpFeatureData
#include <iomanip> // for setw
namespace MultiBoost
{

// -------------------------------------------------------------------------

	/**
	 * Holds the results per example.
	 * This class holds all the results obtained with computeResults(),
	 * which is equivalent to \f${\bf g}(x)\f$. It also offers two methods
	 * that allow the quick evaluation of the results.
	 * @date 16/11/2005
	 */
	class ExampleResults
	{
 	public:

		/**
		 * The constructor. Initialize the index and the votes vector.
		 * @param idx The index of the example.
		 * @param numClasses The number of classes.
		 * @date 16/11/2005
		 */
		ExampleResults(int idx, int numClasses)
				: idx(idx),
				  votesVector(numClasses, 0)
		{
		}

		int idx;  //!< The index of the example

		/**
		 * The vector with the results. Equivalent to what returned by \f${\bf g}(x)\f$.
		 * @remark It is public because the methods of Classifier will access it
		 * directly.
		 */
		vector<double> votesVector;

		/**
		 * Get the winner.
		 * @param rank The rank. 0 = winner. 1 = second, etc..
		 * @return A pair <\f$\ell\f$, \f$g_\ell(x)\f$>, where \f$\ell\f$ is the class index.
		 * @date 16/11/2005
		 */
		pair<int, double> getWinner(int rank = 0);

		/**
		 * Checks if the given class is the winner class.
		 * Example: if the ranking is 5 2 6 3 1 4 (in class indexes):
		 * \code
		 * isWinner(5,0); // -> true
		 * isWinner(2,0); // -> false
		 * isWinner(2,1); // -> true
		 * isWinner(3,3); // -> true
		 * \endcode
		 * @param idxRealClass The index of the actual class.
		 * @param atLeastRank The maximum rank in which the class must be
		 * to be considered a "winner".
		 * @date 16/11/2005
		 */
		bool isWinner(int idxRealClass, int atLeastRank = 0) const;

 	private:

		/**
		 * Create a sorted ranking list. It uses votesVector to build a vector
		 * of pairs that contains the index of the class and the value of the votes
		 * (that is a vector of <\f$\ell\f$, \f$g_\ell(x)\f$>), which is sorted
		 * by the second element, resulting in a ranking of the votes per class.
		 * @param rankedList the vector that will be filled with the rankings.
		 * @date 16/11/2005
		 */
		void getRankedList(vector<pair<int, double> >& rankedList) const;

	};
	// ExampleResults

// -------------------------------------------------------------------------
// -------------------------------------------------------------------------

	pair<int, double> ExampleResults::getWinner(int rank)
	{
		assert(rank >= 0);

		vector<pair<int, double> > rankedList;
		getRankedList(rankedList);  // get the sorted rankings
		return rankedList[rank];
	}

// -------------------------------------------------------------------------

	bool ExampleResults::isWinner(int idxRealClass, int atLeastRank) const
	{
		assert(atLeastRank >= 0);

		vector<pair<int, double> > rankedList;
		getRankedList(rankedList);  // get the sorted rankings

		for (int i = 0; i <= atLeastRank; ++i)
		{
			if (rankedList[i].first == idxRealClass)
				return true;
		}

		return false;
	}

// -------------------------------------------------------------------------

	void ExampleResults::getRankedList(vector<pair<int, double> >& rankedList) const
	{
		rankedList.resize(votesVector.size());

		vector<double>::const_iterator vIt;
		const vector<double>::const_iterator votesVectorEnd = votesVector.end();
		int i;
		for (vIt = votesVector.begin(), i = 0; vIt != votesVectorEnd; ++vIt, ++i)
			rankedList[i] = make_pair(i, *vIt);

		sort(rankedList.begin(), rankedList.end(), nor_utils::comparePairOnSecond<int, double, greater<double> >);
	}

// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------

	Classifier::Classifier(nor_utils::Args &args, const string& shypFileName, vector1Di &allLabels, int verbose)
			: _args(args),
			  _verbose(verbose)
	{
		// The file with the step-by-step information
		if (args.hasArgument("outputinfo"))
			args.getValue("outputinfo", 0, _outputInfoFile);

		// Load strong hypothesis
		if (_verbose > 0)
			cout << endl << endl << "Loading strong hypothesis..." << flush;

		// Create container for the input (test) data.
		_pData = this->loadInputData(shypFileName);

		// Add all the possible classes.
		_pData->addClassNames(allLabels);

		// The class that loads the weak hypotheses
		UnSerialization us;

		// loads them
		us.loadHypotheses(shypFileName, _weakHypotheses);
	}

// ----------------

	Classifier::~Classifier(void)
	{
		vector<BaseLearner*>::iterator it;
		for (it = _weakHypotheses.begin(); it != _weakHypotheses.end(); ++it)
			delete (*it);

		_weakHypotheses.~vector();

		// delete the input data file
		if (_pData)
			delete _pData;
	}

// -------------------------------------------------------------------------

	int Classifier::runLive(vector1Df &testData)
	{
		// Load the data
		_pData->loadTest(testData);

		// Where the results go
		vector<ExampleResults*> results;

		// get the results
		computeResults(_pData, _weakHypotheses, results);

		string sLabel = ClassMappings::getClassNameFromIdx(results[0]->getWinner().first);
		cout << "Label: " << sLabel << endl;

		restartInputDataContainer();

		vector<ExampleResults*>::iterator it;
		for (it = results.begin(); it != results.end(); ++it)
			delete (*it);

		//results.~vector();

		return atoi(sLabel.c_str());
	}

// -------------------------------------------------------------------------

	InputData* Classifier::loadInputData(const string& shypFileName)
	{
		// open file
		ifstream inFile(shypFileName.c_str());
		if (!inFile.is_open())
			throw "ERROR: Cannot open strong hypothesis file <" + shypFileName + ">!";

		// Declares the stream tokenizer
		nor_utils::StreamTokenizer st(inFile, "<>\n\r\t");

		// Move until it finds the multiboost tag
		if (!UnSerialization::seekSimpleTag(st, "multiboost"))
		{
			// no multiboost tag found: this is not the correct file!
			throw "ERROR: Not a valid MultiBoost Strong Hypothesis file!!";
		}

		// Move until it finds the algo tag
		string basicLearnerName = UnSerialization::seekAndParseEnclosedValue < string > (st, "algo");

		// Check if the weak learner exists
		if (!BaseLearner::RegisteredLearners().hasLearner(basicLearnerName))
			throw "ERROR: Weak learner <" + basicLearnerName + "> not registered!!";

		// get the training input data, and load it
		InputData* pData = BaseLearner::RegisteredLearners().getLearner(basicLearnerName)->createInputData();

		// set the non-default arguments of the input data
		pData->initOptions(_args);

		return pData;
	}

	void Classifier::restartInputDataContainer(void)
	{
		_pData->restartInputDataObject();
	}

// -------------------------------------------------------------------------

// Returns the results into ptRes
	void Classifier::computeResults(InputData* pData, vector<BaseLearner*>& weakHypotheses, vector<ExampleResults*>& results)
	{
		assert(!weakHypotheses.empty());

		const int numClasses = ClassMappings::getNumClasses();
		const int numExamples = pData->getNumExamples();

		// Initialize the output info
		OutputInfo* pOutInfo = NULL;

		if (!_outputInfoFile.empty())
			pOutInfo = new OutputInfo(_outputInfoFile);

		// Creating the results structures. See file Structures.h for the
		// PointResults structure
		results.clear();
		results.reserve(numExamples);
		for (int i = 0; i < numExamples; ++i)
			results.push_back(new ExampleResults(i, numClasses));

		// iterator over all the weak hypotheses
		vector<BaseLearner*>::const_iterator whyIt;
		int t;

		// for every feature: 1..T
		for (whyIt = weakHypotheses.begin(), t = 0; whyIt != weakHypotheses.end(); ++whyIt, ++t)
		{
			BaseLearner* currWeakHyp = *whyIt;
			double alpha = currWeakHyp->getAlpha();

			// for every point
			for (int i = 0; i < numExamples; ++i)
			{
				// a reference for clarity and speed
				vector<double>& currVotesVector = results[i]->votesVector;

				// for every class
				for (int l = 0; l < numClasses; ++l)
					currVotesVector[l] += alpha * currWeakHyp->classify(pData, i, l);
			}

			// if needed output the step-by-step information
			if (pOutInfo)
			{
				pOutInfo->outputIteration(t);
				pOutInfo->outputError(pData, currWeakHyp);

				// Margins and edge requires an update of the weight,
				// therefore I keep them out for the moment
				//outInfo.outputMargins(pData, currWeakHyp);
				//outInfo.outputEdge(pData, currWeakHyp);
				pOutInfo->endLine();
			}
		}

		if (pOutInfo)
			delete pOutInfo;

	}

// -------------------------------------------------------------------------

	double Classifier::getOverallError(InputData* pData, const vector<ExampleResults*>& results, int atLeastRank)
	{
		const int numExamples = pData->getNumExamples();
		int numErrors = 0;

		assert(atLeastRank >= 0);

		for (int i = 0; i < numExamples; ++i)
		{
			// if the actual class is not the one with the highest vote in the
			// vote vector, then it is an error!
			if (!results[i]->isWinner(pData->getClass(i), atLeastRank))
				++numErrors;
		}

		// makes the error between 0 and 1
		return (double) numErrors / (double) numExamples;
	}

// -------------------------------------------------------------------------

	void Classifier::getClassError(InputData* pData, const vector<ExampleResults*>& results, vector<double>& classError, int atLeastRank)
	{
		const int numExamples = pData->getNumExamples();
		const int numClasses = ClassMappings::getNumClasses();

		classError.resize(numClasses, 0);

		assert(atLeastRank >= 0);

		for (int i = 0; i < numExamples; ++i)
		{
			// if the actual class is not the one with the highest vote in the
			// vote vector, then it is an error!
			if (!results[i]->isWinner(pData->getClass(i), atLeastRank))
				++classError[pData->getClass(i)];
		}

		// makes the error between 0 and 1
		for (int l = 0; l < numClasses; ++l)
			classError[l] /= (double) pData->getNumExamplesPerClass(l);
	}

// -------------------------------------------------------------------------

}// end of namespace MultiBoost
