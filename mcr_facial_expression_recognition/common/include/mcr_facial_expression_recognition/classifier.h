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

/**
 * \file Classifier.h Performs the classification.
 */

#ifndef __CLASSIFIER_H
#define __CLASSIFIER_H

#include "utils/args.h"
#include "io/input_data.h"
#include "utils/utils.h"
#include "weak_learners/base_learner.h"

#include <string>
#include <cassert>

#include "general.h"

using namespace std;

namespace MultiBoost
{

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

// Forward declaration of the class that will hold the results for each example.
	class ExampleResults;

	/**
	 * Classify a dataset. Using the strong hypothesis file (shyp.xml by default) it builds the
	 * list of weak hypothesis (or weak learners), and use them to perform a classification over
	 * the given data set. The strong hypothesis is the linear combination of the weak
	 * hypotheses and their confidence alpha, and is defined as:
	 * \f[
	 * {\bf g}(x) = \sum_{t=1}^T \alpha^{(t)} {\bf h}^{(t)}(x),
	 * \f]
	 * where the bold defines a vector as returned value.
	 * To obtain a single class, we simply take the winning class that receives
	 * the "most vote", that is:
	 * \f[
	 * f(x) = \mathop{\rm arg\, max}_{\ell} g_\ell(x).
	 * \f]
	 * @date 15/11/2005
	 */
	class Classifier
	{
 	public:

		/**
		 * The constructor. It initializes the variable and set them using the
		 * information provided by the arguments passed. They are parsed
		 * using the helpers provided by class Args.
		 * @param args The arguments defined by the user in the command line.
		 * @param verbose The level of verbosity
		 * @see _verbose
		 * @date 16/11/2005
		 */
		Classifier(nor_utils::Args& args, const string& shypFileName, vector1Di &allLabels, int verbose = 1);

		~Classifier(void);

		/**
		 *	NEW method @ggiorgana 25-01-11
		 */
		int runLive(vector1Df &testData);

 	protected:

		/**
		 *	NEW method @ggiorgana 25-01-11
		 */
		InputData* loadInputData(const string& shypFileName);

		/**
		 *	NEW method @ggiorgana 25-01-11
		 */
		void restartInputDataContainer(void);

		/**
		 * Compute the results using the weak hypotheses.
		 * This method is the one that effectively computes \f${\bf g}(x)\f$.
		 * @param pData A pointer to the data to be classified.
		 * @param weakHypotheses The list of weak hypotheses.
		 * @param results The vector where the results will be stored.
		 * @see ExampleResults
		 * @date 16/11/2005
		 */
		void computeResults(InputData* pData, vector<BaseLearner*>& weakHypotheses, vector<ExampleResults*>& results);

		/**
		 * Compute the overall error on the data.
		 * @param pData A pointer to the data. Needed to get the actual class of
		 * the example.
		 * @param results The vector where the results are hold.
		 * @param atLeastRank The maximum rank in which the classification will not be considered
		 * an error. If \a atLeastRank = 0, no errors are allowed. If it is 1, the second "guess"
		 * will be taken into consideration, among the first, and so on.
		 * @return The error.
		 * @see ExampleResults
		 * @date 16/11/2005
		 */
		double getOverallError(InputData* pData, const vector<ExampleResults*>& results, int atLeastRank = 0);

		/**
		 * Compute the error per class.
		 * @param pData A pointer to the data. Needed to get the actual class of
		 * the example.
		 * @param results The vector where the results are hold.
		 * @param classError The returned per class errors.
		 * @param atLeastRank The maximum rank in which the classification will not be considered
		 * an error. If \a atLeastRank = 0, no errors are allowed. If it is 1, the second "guess"
		 * will be taken into consideration, among the first, and so on.
		 * @see ExampleResults
		 * @date 16/11/2005
		 */
		void getClassError(InputData* pData, const vector<ExampleResults*>& results, vector<double>& classError, int atLeastRank = 0);

		/**
		 * Defines the level of verbosity:
		 * - 0 = no messages
		 * - 1 = basic messages
		 * - 2 = show all messages
		 */
		int _verbose;

		nor_utils::Args& _args;  //!< The arguments defined by the user.
		string _outputInfoFile;  //!< The filename of the step-by-step information file that will be updated

 	private:

		/**
		 * Fake assignment operator to avoid warning.
		 * @date 6/12/2005
		 */
		Classifier& operator=(const Classifier&)
		{
		}

		/**
		 *	Where to put the weak hypotheses
		 */
		vector<BaseLearner*> _weakHypotheses;

		/**
		 *	Where to put the input (test) data.
		 */
		InputData* _pData;
	};

}  // end of namespace MultiBoost

#endif // __CLASSIFIER_H
