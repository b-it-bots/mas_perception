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
 * \file MultiStumpLearner.h A multi threshold decision stump learner.
 */

#ifndef __MULTI_STUMP_LEARNER_H
#define __MULTI_STUMP_LEARNER_H

#include "weak_learners/stump_learner.h"
#include "utils/args.h"
#include "io/input_dData.h"
#include "io/class_mappings.h"

#include <vector>
#include <fstream>
#include <cassert>
#include <limits>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost
{

	/**
	 * A \b multi threshold decision stump learner.
	 * There is a threshold for every class.
	 */
	class MultiStumpLearner : public StumpLearner
	{
 	public:

		/**
		 * The destructor. Must be declared (virtual) for the proper destruction of
		 * the object.
		 */
		virtual ~MultiStumpLearner()
		{
		}

		/**
		 * Returns itself as object.
		 * @remark It uses the trick described in http://www.parashift.com/c++-faq-lite/serialization.html#faq-36.8
		 * for the auto-registering classes.
		 * @date 14/11/2005
		 */
		virtual BaseLearner* create()
		{
			return new MultiStumpLearner();
		}

		/**
		 * Run the learner to build the classifier on the given data.
		 * @param pData The pointer to the data
		 * @see BaseLearner::run
		 * @date 11/11/2005
		 */
		virtual void run(InputData* pData);

		/**
		 * Save the current object information needed for classification,
		 * that is the threshold list.
		 * @param outputStream The stream where the data will be saved
		 * @param numTabs The number of tabs before the tag. Useful for indentation
		 * @remark To fully save the object it is \b very \b important to call
		 * also the super-class method.
		 * @see StumpLearner::save()
		 * @date 13/11/2005
		 */
		virtual void save(ofstream& outputStream, int numTabs = 0);

		/**
		 * Load the xml file that contains the serialized information
		 * needed for the classification and that belongs to this class.
		 * @param st The stream tokenizer that returns tags and values as tokens
		 * @see save()
		 * @date 13/11/2005
		 */
		virtual void load(nor_utils::StreamTokenizer& st);

 	protected:

		/**
		 * A discriminative function.
		 * @remarks Positive or negative do NOT refer to positive or negative classification.
		 * This function is equivalent to the phi function in my thesis.
		 * @param val The value to discriminate
		 * @param classIdx The index of the class
		 * @return +1 if \a val is on one side of the border for \a classIdx and -1 otherwise
		 * @date 11/11/2005
		 * @see classify
		 */
		virtual char phi(double val, int classIdx);

		/**
		 * Find the thresholds (one for each class) for column \a columnIndex.
		 * @param dataBegin The iterator to the beginning of the data.
		 * @param dataEnd The iterator to the end of the data.
		 * @param pData The pointer to the data
		 * @param thresholds The thresholds to update
		 * @param mu The The class-wise rates to update
		 * @param v The alignment vector to update
		 * @see StumpLearner::sRates
		 * @see run
		 * @see _thresholds
		 * @date 11/11/2005
		 */
		template<typename T>
		void findThreshold(const typename vector<pair<int, T> >::iterator& dataBegin, const typename vector<pair<int, T> >::iterator& dataEnd, InputData* pData,
		                   vector<double>& thresholds, vector<sRates>& mu, vector<double>& v);

		vector<double> _thresholds;  //!< The thresholds (one for each class) of the decision stump.
	};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// The implementation of the template function findThreshold of class
// MultiStumpLearner
	template<typename T>
	void MultiStumpLearner::findThreshold(const typename vector<pair<int, T> >::iterator& dataBegin, const typename vector<pair<int, T> >::iterator& dataEnd,
	                                      InputData* pData, vector<double>& thresholds, vector<sRates>& mu, vector<double>& v)
	{
		const int numClasses = ClassMappings::getNumClasses();

		// resize and set to 0
		fill(_leftErrors.begin(), _leftErrors.end(), 0);
		fill(_weightsPerClass.begin(), _weightsPerClass.end(), 0);

		typename vector<pair<int, T> >::iterator currentSplitPos;  // the iterator of the currently examined example
		typename vector<pair<int, T> >::iterator previousSplitPos;  // the iterator of the example before the current example
		typename vector<pair<int, T> >::const_iterator endArray;  // the iterator on the last example (just before dataEnd)

		//////////////////////////////////////////////////
		// Initialization of the class-wise error

		// The class-wise error on the right side of the threshold
		double tmpRightError;

		for (int l = 0; l < numClasses; ++l)
		{
			tmpRightError = 0;

			for (currentSplitPos = dataBegin; currentSplitPos != dataEnd; ++currentSplitPos)
			{
				double weight = pData->getWeight(currentSplitPos->first, l);

				// We assume that class "currClass" is always on the right side;
				// therefore, all points l that are not currClass (x) on right side,
				// are considered error.
				// <l x l x x x l x x> = 3 (if each example has weight 1)
				// ^-- tmpError: error if we set the cut at the extreme left side
				if (pData->getClass(currentSplitPos->first) != l)
					tmpRightError += weight;

				_weightsPerClass[l] += weight;
			}

			_halfWeightsPerClass[l] = _weightsPerClass[l] / 2;

			assert(tmpRightError < 1);
			_rightErrors[l] = tmpRightError;  // store the class-wise error
			_bestErrors[l] = numeric_limits<double>::max();
		}

		////////////////////////////////////////////////////

		currentSplitPos = dataBegin;  // reset position
		endArray = dataEnd;
		--endArray;

		double tmpError = 0;
		bool flipIt;

		// find the best threshold (cutting point)
		while (currentSplitPos != endArray)
		{
			// at the first split we have
			// first split: x | x x x x x x x x ..
			//    previous -^   ^- current
			previousSplitPos = currentSplitPos;
			++currentSplitPos;

			// point at the same position: to skip because we cannot find a cutting point here!
			while (previousSplitPos->second == currentSplitPos->second && currentSplitPos != endArray)
			{
				for (int l = 0; l < numClasses; ++l)
				{
					if (pData->getClass(previousSplitPos->first) == l)
						_leftErrors[l] += pData->getWeight(previousSplitPos->first, l);
					else
						_rightErrors[l] -= pData->getWeight(previousSplitPos->first, l);
				}

				previousSplitPos = currentSplitPos;
				++currentSplitPos;
			}

			for (int l = 0; l < numClasses; ++l)
			{
				if (pData->getClass(previousSplitPos->first) == l)
				{
					// c=current class, x=other class
					// .. c | x x c x c x ..
					_leftErrors[l] += pData->getWeight(previousSplitPos->first, l);
				}
				else
				{
					// c=current class, x=other class
					// .. x | x x c x c x ..
					_rightErrors[l] -= pData->getWeight(previousSplitPos->first, l);
				}

				tmpError = _rightErrors[l] + _leftErrors[l];

				// switch the class-wise error if it is bigger than chance
				if (tmpError > _halfWeightsPerClass[l] + _smallVal)
				{
					tmpError = _weightsPerClass[l] - tmpError;
					flipIt = true;
				}
				else
					flipIt = false;

				// The summed error MUST be smaller than chance
				assert(tmpError <= _halfWeightsPerClass[l] + _smallVal);

				// the overall error is smaller!
				if (tmpError < _bestErrors[l] + _smallVal)
				{
					_bestErrors[l] = tmpError;
					// compute the thresholds
					thresholds[l] = static_cast<double>(previousSplitPos->second + currentSplitPos->second) / 2;

					// If we assume that class [l] is always on the right side,
					// here we must flip, as the lowest error is on the left side.
					// example:
					// c=current class, x=other class
					// .. c c c x | c x x x .. = 2 errors (if we flip!)
					if (flipIt)
						v[l] = -1;
					else
						v[l] = +1;

				}
			}  // for l
		}  // while (currentSplitPos != endArray)

		////////////////////////////////////////////////////

		// Fill the mus. This could have been done in the threshold loop,
		// but here is done just once
		for (int l = 0; l < numClasses; ++l)
		{
			mu[l].classIdx = l;

			mu[l].rPls = _weightsPerClass[l] - _bestErrors[l];
			mu[l].rMin = _bestErrors[l];
			mu[l].rZero = mu[l].rPls + mu[l].rMin;
		}

	}

}  // end of namespace MultiBoost

#endif // __MULTI_STUMP_LEARNER_H
