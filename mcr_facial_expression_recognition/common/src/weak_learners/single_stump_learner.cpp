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

#include "weak_learners/single_stump_learner.h"

#include "io/serialization.cpp"
#include "io/sorted_data.cpp"

#include <limits> // for numeric_limits<>
namespace MultiBoost
{

//REGISTER_LEARNER_NAME(SingleStump, SingleStumpLearner)
	REGISTER_LEARNER (SingleStumpLearner)

// ------------------------------------------------------------------------------

	void SingleStumpLearner::run(InputData* pData)
	{
		const int numClasses = ClassMappings::getNumClasses();
		const int numColumns = pData->getNumColumns();

		// set the smoothing value to avoid numerical problem
		// when theta=0.
		setSmoothingVal(1.0 / (double) pData->getNumExamples() * 0.01);

		// resize: it's done here to avoid a reallocation
		// for each dimension.
		_leftErrors.resize(numClasses);
		_rightErrors.resize(numClasses);
		_bestErrors.resize(numClasses);
		_weightsPerClass.resize(numClasses);
		_halfWeightsPerClass.resize(numClasses);

		vector<sRates> mu(numClasses);  // The class-wise rates. See BaseLearner::sRates for more info.
		vector<double> tmpV(numClasses);  // The class-wise votes/abstentions

		double tmpThreshold;
		double tmpAlpha;

		double bestE = numeric_limits<double>::max();
		double tmpE;

		for (int j = 0; j < numColumns; ++j)
		{
			const vpIterator dataBegin = static_cast<SortedData*>(pData)->getSortedBegin(j);
			const vpIterator dataEnd = static_cast<SortedData*>(pData)->getSortedEnd(j);

			//findThreshold(pData, j, tmpThreshold, mu, tmpV);
			findThreshold<double>(dataBegin, dataEnd, pData, tmpThreshold, mu, tmpV);

			tmpE = getEnergy(mu, tmpAlpha, tmpV);
			if (tmpE < bestE)
			{
				// Store it in the current weak hypothesis.
				// note: I don't really like having so many temp variables
				// but the alternative would be a structure, which would need
				// to be inheritable to make things more consistent. But this would
				// make it less flexible. Therefore, I am still undecided. This
				// might change!

				_alpha = tmpAlpha;
				_v = tmpV;
				_selectedColumn = j;
				_threshold = tmpThreshold;

				bestE = tmpE;
			}

		}

	}

// ------------------------------------------------------------------------------

	char SingleStumpLearner::phi(double val, int /*classIdx*/)
	{
		if (val > _threshold)
			return +1;
		else
			return -1;
	}

// -----------------------------------------------------------------------

	void SingleStumpLearner::save(ofstream& outputStream, int numTabs)
	{
		// Calling the super-class method
		StumpLearner::save(outputStream, numTabs);

		// save selectedCoulumn
		outputStream << Serialization::standardTag("threshold", _threshold, numTabs) << endl;
	}

// -----------------------------------------------------------------------

	void SingleStumpLearner::load(nor_utils::StreamTokenizer& st)
	{
		// Calling the super-class method
		StumpLearner::load(st);

		_threshold = UnSerialization::seekAndParseEnclosedValue<double>(st, "threshold");

	}

// -----------------------------------------------------------------------

	void SingleStumpLearner::getStateData(vector<double>& data, const string& /*reason*/, InputData* pData)
	{
		const int numClasses = ClassMappings::getNumClasses();
		const int numExamples = pData->getNumExamples();

		// reason ignored for the moment as it is used for a single task
		data.resize(numClasses + numExamples);

		int pos = 0;

		for (int l = 0; l < numClasses; ++l)
			data[pos++] = _v[l];

		for (int i = 0; i < numExamples; ++i)
			data[pos++] = SingleStumpLearner::phi(pData->getValue(i, _selectedColumn), 0);
	}

// -----------------------------------------------------------------------

}// end of namespace MultiBoost
