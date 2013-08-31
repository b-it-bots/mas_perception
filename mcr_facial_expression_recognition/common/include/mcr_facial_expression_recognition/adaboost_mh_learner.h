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
 * \file AdaBoostMHLearner.h The meta-learner AdaBoostLearner.
 */

#ifndef __ADA_BOOST_H
#define __ADA_BOOST_H

#include "utils/args.h"
#include "general.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost
{

	class OutputInfo;
	class BaseLearner;
	class InputData;
	class Serialization;

	/**
	 * The AdaBoost learner. This class performs the meta-learning
	 * by calling the weak learners and updating the weights.
	 * @date 12/11/2005
	 */
	class AdaBoostMHLearner
	{
 	public:

		/**
		 * The constructor. It initializes the variables and sets them using the
		 * information provided by the arguments passed. They are parsed
		 * using the helpers provided by class Params.
		 * @param args The arguments defined by the user in the command line.
		 * @param verbose The level of verbosity.
		 * @date 13/11/2005
		 */
		AdaBoostMHLearner(nor_utils::Args& args, int verbose = 0);

		/**
		 * Start the learning process.
		 * @param numIterations The number of iterations.
		 * @param trainFileName The name of the file with the training data.
		 * @param testFileName Optional: The name of the file with the testing
		 * data. Used for step-by-step evaluation.
		 * @see OutputInfo
		 * @date 10/11/2005
		 */
		void run(int numIterations, const string& trainFileName, const string& testFileName = "");

		/**
		 *	NEW method @ggiorgana
		 *
		 *
		 */
		void run(int numIterations, vector3Df &trainData, vector1Di &labels);

 	protected:

		/**
		 * Updates the weights of the examples.
		 * The re-weighting of \f$w\f$ (the weight vector over all the examples and classes)
		 * is done using the following formula
		 * \f[
		 *  w^{(t+1)}_{i, \ell}=
		 *        \frac{ w^{(t)}_{i, \ell} \exp \left( -\alpha^{(t)}
		 *        h_\ell^{(t)}(x_i) y_{i, \ell} \right) }{ Z^{(t)} }
		 * \f]
		 * where \a Z is a normalization factor, and it is defined as
		 * \f[
		 *  Z^{(t)} =
		 *     \sum_{j=1}^n \sum_{\ell=1}^k w^{(t)}_{j, \ell} \exp \left( -\alpha^{(t)}
		 *        h_\ell^{(t)}(x_j) y_{j, \ell} \right)
		 * \f]
		 * where \f$n\f$ is the number of examples, \f$k\f$ the number of classes,
		 * \f$\alpha\f$ is the confidence in the weak classifier,
		 * \f$h_\ell(x_i)\f$ is the classification of example \f$x_i\f$ for class \f$\ell\f$
		 * with the classifier found at the current iteration (see BaseLearner::classify()),
		 * and \f$y_i\f$ is the binary label of that
		 * example, defined in InputData::getBinaryClass().
		 * @param pTrainData The pointer to the training data.
		 * @param pWeakHypothesis The current weak hypothesis.
		 * @return The value of the edge. It will be used to see if the algorithm can continue
		 * with learning.
		 * @date 16/11/2005
		 */
		double updateWeights(InputData* pTrainData, BaseLearner* pWeakHypothesis);

		/**
		 * Resume the weak learner list.
		 * @return The current iteration number. 0 if not -resume option has been called
		 * @date 21/12/2005
		 * @see resumeProcess
		 * @remark resumeProcess must be called too!
		 */
		int resumeWeakLearners();

		/**
		 * Resume the training using the features in _resumeShypFileName if the
		 * option -resume has been specified.
		 * @return The current iteration number.
		 * @date 21/12/2005
		 */
		void resumeProcess(Serialization& ss, InputData* pTrainData, InputData* pTestData, OutputInfo* pOutInfo);

		vector<BaseLearner*> _foundHypotheses;  //!< The list of the hypotheses found.

		InputData* _pTrainData;  //!< A pointer to the training data.
		InputData* _pTestData;  //!< A pointer to the test data (if exists).

		string _basicLearnerName;  //!< The name of the basic learner used by AdaBoost.
		string _shypFileName;  //!< File name of the strong hypothesis.

		int _maxTime;  //!< Time limit for the whole processing. Default: no time limit (-1).
		double _theta;  //!< the value of theta. Default = 0.

		/**
		 * If resume is set, this will hold the strong hypothesis file to load in order to
		 * continue with the training process.
		 */
		string _resumeShypFileName;

		nor_utils::Args& _args;  //!< The arguments defined by the user.

		/**
		 * Verbose level.
		 * There are three levels of verbosity:
		 * - 0 = no messages
		 * - 1 = basic messages
		 * - 2 = show all messages
		 */
		int _verbose;
		string _outputInfoFile;  //!< The filename of the step-by-step information file that will be updated

		const double _smallVal;  //!< A small value, to solve numeric issues

		////////////////////////////////////////////////////////////////
 	private:
		/**
		 * Fake assignment operator to avoid warning.
		 * @date 6/12/2005
		 */
		AdaBoostMHLearner& operator=(const AdaBoostMHLearner&)
		{
		}

	};

}  // end of namespace MultiBoost

#endif // __ADA_BOOST_H
