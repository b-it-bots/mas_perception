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
 * \file StumpLearner.h Stump based weak learners.
 */

#ifndef __STUMP_LEARNER_H
#define __STUMP_LEARNER_H

#include "weak_learners/base_learner.cpp"
#include "utils/args.cpp"
#include "io/input_data.cpp"

#include <vector>
#include <fstream>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost
{

	/**
	 * A generic decision stump learner.
	 *
	 * @date 05/11/05
	 */
	class StumpLearner : public BaseLearner
	{
 	public:

		/**
		 * The constructor. It initializes theta to zero and the abstention to false
		 * (that is, their default values).
		 * @date 11/11/2005
		 */
		StumpLearner()
				: _theta(0),
				  _abstention(ABST_NO_ABSTENTION),
				  _selectedColumn(-1),
				  _verbose(1)
		{
		}

		/**
		 * The destructor. Must be declared (virtual) for the proper destruction of
		 * the object.
		 */
		virtual ~StumpLearner()
		{
		}

		/**
		 * Declare weak-learner-specific arguments.
		 * These arguments will be added to the list of arguments under
		 * the group specific of the weak learner. It is called
		 * automatically in main, when the list of arguments is built up.
		 * Use this method to declare the arguments that belongs to
		 * the weak learner only.
		 *
		 * This class declares the argument "abstention" only.
		 * @param args The Args class reference which can be used to declare
		 * additional arguments.
		 * @date 28/11/2005
		 */
		virtual void declareArguments(nor_utils::Args& args);

		/**
		 * Set the arguments of the algorithm using the standard interface
		 * of the arguments. Call this to set the arguments asked by the user.
		 * @param args The arguments defined by the user in the command line.
		 * @date 14/11/2005
		 */
		virtual void initOptions(nor_utils::Args& args);

		/**
		 * Creates an InputData object that it is good for the
		 * weak learner. Overridden to return SortedData.
		 * @see InputData
		 * @see BaseLearner::createInputData()
		 * @see SortedData
		 * @warning The object \b must be destroyed by the caller.
		 * @date 21/11/2005
		 */
		virtual InputData* createInputData();

		/**
		 * Return {+1, -1} for the given class and value using the learned classifier.
		 * @param pData The pointer to the data
		 * @param idx The index of the example to classify
		 * @param classIdx The index of the class
		 * @remark Passing the data and the index to the example is not nice at all.
		 * This will soon be replace with the passing of the example itself in some
		 * form (probably a structure to the example).
		 * @return +1 if the classifier thinks that \a val belongs to class
		 * \a classIdx, -1 if it does not and 0 if it abstain. If ABST_REAL is selected
		 * the value returned is a range between -1 and +1 which holds the confidence
		 * in the classification.
		 * @date 13/11/2005
		 */
		virtual double classify(InputData* pData, int idx, int classIdx);

		/**
		 * Save the current object information needed for classification,
		 * that is: \a _v, The alignment vector and \a _selectedColumn, the column of the
		 * data with that yielded the lowest error
		 * @param outputStream The stream where the data will be saved
		 * @param numTabs The number of tabs before the tag. Useful for indentation
		 * @remark To fully save the object it is \b very \b important to call
		 * also the super-class method.
		 * @see BaseLearner::save()
		 * @date 13/11/2005
		 */
		virtual void save(ofstream& outputStream, int numTabs = 0);

		/**
		 * Load the xml file that contains the serialized information
		 * needed for the classification and that belongs to this class
		 * @param st The stream tokenizer that returns tags and values as tokens
		 * @see save()
		 * @date 13/11/2005
		 */
		virtual void load(nor_utils::StreamTokenizer& st);

 	protected:

		/**
		 * A discriminative function.
		 * @remarks Positive or negative do NOT refer to positive or negative classification.
		 * This function is equivalent to the phi function in
		 * <a href="http://www.iro.umontreal.ca/~casagran/docs/thesis_casagrande.pdf">my thesis</a>.
		 * @param val The value to discriminate
		 * @param classIdx The index of the class
		 * @return +1 if \a val is on one side of the border for \a classIdx and -1 otherwise
		 * @date 11/11/2005
		 */
		virtual char phi(double val, int classIdx) = 0;

		/**
		 * The per class rates.
		 * It is defined (using \f$\mu\f$ as variable) as
		 * \f{eqnarray*}
		 *   \mu_{\ell+} & = & \sum_{correct} w_{i,\ell}^{(t)},\\
   *   \mu_{\ell-} &=& \sum_{incorrect} w_{i,\ell}^{(t)}, \\ 
		 *   \mu_{\ell0} & = & \mu_{\ell-} + \mu_{\ell+}
		 * \f}
		 * @remark The implemented decision stump algorithms do not abstain themselves.
		 * The abstention value, that is for \f$v=0\f$ is computed in getEnergy()
		 * @date 11/11/2005
		 */
		struct sRates
		{
			sRates()
					: classIdx(-1),
					  rPls(0),
					  rMin(0),
					  rZero(0)
			{
			}  //!< The constructor.

			int classIdx;  //!< the index of the class. Needed because we will sort the vector that contains this object

			double rPls;  //!< positive rate, or \f$\mu_{\ell+}\f$
			double rMin;  //!<  negative rate, or \f$\mu_{\ell-}\f$
			double rZero;  //!< abstention rate, or \f$\mu_{\ell0}\f$

			/**
			 * Overloading of the operator to allow sorting.
			 * @param el The other element to be compared with
			 * @date 11/11/2005
			 */
			bool operator<(const sRates& el) const
			{
				return el.rPls * el.rMin < this->rPls * this->rMin;
			}
		};

		/**
		 * Return the energy of the current learner. The energy is defined as
		 * \f[
		 * Z = 2 \sqrt{\epsilon_+ \epsilon_-} + \epsilon_0
		 * \f]
		 * and it is the value to minimize.
		 * @param alpha The value of alpha that will be updated by this function minimizing the
		 * energy and using the helper function provided by BaseLearner.
		 * @param mu The class rates.
		 * @param v The alignment vector that will be updated in the case of abstention.
		 * @return The energy value that we want minimize.
		 * @date 12/11/2005
		 */
		virtual double getEnergy(vector<sRates>& mu, double& alpha, vector<double>& v);

		/**
		 * Updates the \a v vector (alignment vector) using a greedy abstention algorithm.
		 * We do not leave the decision to the weak learner as usual, but we add 0 to the
		 * decisions of the alignment vector \a v. This is done by optimizing the energy value
		 * with a greedy algorithm (that, for the time being, is not proved to be optimal.
		 * We first get \a v from one of the stump algorithms (see for instance
		 * SingleStumpLearner::findThreshold()).
		 * Then, in an iteration over the classes, we select the "best" element of v to set
		 * to 0, that is, the one that decreases the energy the most.
		 * @param mu The class rates. It is not \a const because sort() is called.
		 * @param currEnergy The current energy value, obtained in getEnergy().
		 * @param eps The current epsilons, that the overall error, correct and abstention rates.
		 * @param alpha The value of alpha that will be updated by this function minimizing the
		 * energy and using the helper function provided by BaseLearner.
		 * @param v The alignment vector that will be updated in the case of abstention.
		 * @remark The complexity of this algorithm is O(k^2).
		 * @date 28/11/2005
		 */
		virtual double doGreedyAbstention(vector<sRates>& mu, double currEnergy, sRates& eps, double& alpha, vector<double>& v);

		/**
		 * Updates the \a v vector (alignment vector) evaluating all the possible combinations.
		 * Again we do not leave the abstention to the weak learner but we add 0 to the
		 * alignment vector \a v, which decreases most the energy function.
		 * @param mu The class rates.
		 * @param currEnergy The current energy value, obtained in getEnergy().
		 * @param eps The current epsilons, that is the overall error, correct and abstention rates.
		 * @param alpha The value of alpha that will be updated by this function minimizing the
		 * energy and using the helper function provided by BaseLearner.
		 * @param v The alignment vector that will be updated in the case of abstention.
		 * @remark The complexity of this algorithm is O(2^k).
		 * @date 28/11/2005
		 */
		virtual double doFullAbstention(const vector<sRates>& mu, double currEnergy, sRates& eps, double& alpha, vector<double>& v);

		/**
		 * Update the \a v vector (alignment vector) with \b real values, setting alpha to 1.
		 * This corresponds to "AdaBoost.MH with real valued predictions" (see "BoosTexter:
		 * A Boosting-based System for Text Categorization" by Schapire & Singer), which must
		 * \b not be confused with Real AdaBoost, where the weak learner returns a confidence
		 * on its prediction.
		 * @param mu The class rates.
		 * @param eps The current epsilons, that is the overall error, correct and abstention rates.
		 * @param alpha The value of alpha which will be turned into 1.
		 * @param v The alignment vector that will be updated with the real valued values.
		 * @remark Because here we have an alpha which is not uniform among all the classes, (\a v in
		 * fact becomes the alpha) the energy function to minimize becomes:
		 * \f[
		 * Z = \sum_{i=1}^n \sum_{\ell=1}^k w_{i,\ell} \exp{ \left( -\alpha_\ell h_\ell(x_i) y_{i,\ell} \right) },
		 * \f]
		 * which simplifies to the new energy function that we have to minimize:
		 * \f[
		 * Z = 2 \sum_{\ell=1}^k \sqrt{ \mu_{-,\ell} \mu_{+,\ell} }
		 * \f]
		 * @remark If a margin has been selected (\f$\theta > 0\f$), then the energy function
		 * is computed as usual.
		 * @see sRates
		 * @see doGreedyAbstention
		 * @see doFullAbstention
		 * @date 10/2/2006
		 * @bug The actual formula should be
		 * \f[
		 Z = \mu_0 + 2 \sum_{\ell=1}^k \sqrt{ \mu_{-,\ell} \mu_{+,\ell} },
		 * \f]
		 * but we do not consider the abstention at weak learner level yet.
		 */
		virtual double doRealAbstention(const vector<sRates>& mu, const sRates& eps, double& alpha, vector<double>& v);

		/**
		 * The class-wise abstention/alignment vector.
		 * If the abstention is \b not set to ABST_REAL, it is obtained simply with
		 * \f[
		 *   v_\ell = \begin{cases}
		 *   +1 & \mbox{ if } \mu_{\ell+} > \mu_{\ell-}\\
   *   -1 & \mbox{ otherwise.}
		 *   \end{cases}
		 * \f]
		 * where  \f$\mu\f$ are defined in sRates.
		 *
		 * In the case of ABST_REAL, this vector becomes the class-wise alpha value,
		 * and alpha is set to 1.
		 * @see sRates
		 * @see doRealAbstention
		 * @see eAbstType
		 * @date 11/11/2005
		 */
		vector<double> _v;
		int _selectedColumn;  //!< The column of the training data with the lowest error.

		double _theta;  //!< the value of theta. Default = 0;

		/**
		 * The type of abstention.
		 * @date 28/11/2005
		 * @see doGreedyAbstention
		 * @see doFullAbstention
		 * @see doRealAbstention
		 */
		enum eAbstType
		{
			ABST_NO_ABSTENTION,  //!< No abstention is performed.
			ABST_GREEDY,  //!< The abstention is type greedy, which complexity is O(k^2), where k is the number of classes.
			ABST_REAL,  //!< The value of the v vector is float instead of {-1,0,+1} as described in Boostexter's paper
			ABST_FULL  //!< The abstention is full, which complexity is O(2^k).
		};
		eAbstType _abstention;  //!< Activate abstention. Default = 0 (no abstention);

		vector<double> _rightErrors;  //!< the class-wise errors on the right side of the threshold.
		vector<double> _leftErrors;  //!< the class-wise errors on the left side of the threshold.
		vector<double> _bestErrors;  //!< the errors of the best found threshold.

		vector<double> _weightsPerClass;  //!< The total weight per class.
		vector<double> _halfWeightsPerClass;  //!< The half of the total weights per class.

		int _verbose;  //!< The level of verbosity.
	};

// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------

}// end of namespace MultiBoost

#endif // __STUMP_LEARNER_H
