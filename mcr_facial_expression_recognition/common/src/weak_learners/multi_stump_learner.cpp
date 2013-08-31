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

#include "weak_learners/multi_stump_learner.h"

#include "io/serialization.h"
#include "io/sorted_data.h"

#include <limits> // for numeric_limits<>
namespace MultiBoost
{

	REGISTER_LEARNER (MultiStumpLearner)

// ------------------------------------------------------------------------------

	void MultiStumpLearner::run(InputData* pData)
	{
		const int numClasses = ClassMappings::getNumClasses();
		const int numColumns = pData->getNumColumns();

		// set the smoothing value to avoid numerical problem
		// when theta=0.
		setSmoothingVal(1.0 / (double) pData->getNumExamples() * 0.01);

		// resize
		_leftErrors.resize(numClasses);
		_rightErrors.resize(numClasses);
		_bestErrors.resize(numClasses);
		_weightsPerClass.resize(numClasses);
		_halfWeightsPerClass.resize(numClasses);

		vector<sRates> mu(numClasses);  // The class-wise rates. See BaseLearner::sRates for more info.

		vector<double> tmpV(numClasses);  // The class-wise votes/abstentions
		vector<double> tmpThresholds(numClasses);
		double tmpAlpha;

		double bestE = numeric_limits<double>::max();
		double tmpE;

		for (int j = 0; j < numColumns; ++j)
		{
			const vpIterator dataBegin = static_cast<SortedData*>(pData)->getSortedBegin(j);
			const vpIterator dataEnd = static_cast<SortedData*>(pData)->getSortedEnd(j);

			//findThresholds(pData, j, tmpThresholds, mu, tmpV);
			findThreshold<double>(dataBegin, dataEnd, pData, tmpThresholds, mu, tmpV);

			tmpE = getEnergy(mu, tmpAlpha, tmpV);
			if (tmpE < bestE)
			{
				// Store it in the current algorithm
				// note: I don't really like having so many temp variables
				// but the alternative would be a structure, which would need
				// to be inheritable to make things more consistent. But this would
				// make it less flexible. Therefore, I am still undecided. This
				// might change!

				_alpha = tmpAlpha;
				_v = tmpV;
				_selectedColumn = j;
				_thresholds = tmpThresholds;

				bestE = tmpE;
			}

		}

	}

// ------------------------------------------------------------------------------

	char MultiStumpLearner::phi(double val, int classIdx)
	{
		if (val > _thresholds[classIdx])
			return +1;
		else
			return -1;
	}

// -----------------------------------------------------------------------

	void MultiStumpLearner::save(ofstream& outputStream, int numTabs)
	{
		// Calling the super-class method
		StumpLearner::save(outputStream, numTabs);

		// save all the thresholds
		outputStream << Serialization::vectorTag("thArray", _thresholds, numTabs) << endl;
	}

// -----------------------------------------------------------------------

	void MultiStumpLearner::load(nor_utils::StreamTokenizer& st)
	{
		// Calling the super-class method
		StumpLearner::load(st);

		// load vArray data
		UnSerialization::seekAndParseVectorTag(st, "thArray", _thresholds);
	}

// -----------------------------------------------------------------------

}// end of namespace MultiBoost
