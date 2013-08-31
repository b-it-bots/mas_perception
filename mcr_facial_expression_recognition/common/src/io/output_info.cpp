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

#ifndef __OUTPUT_INFO_CPP
#define __OUTPUT_INFO_CPP

#include <limits>

#include "io/output_info.h"
#include "io/class_mappings.cpp"
#include "weak_learners/base_learner.cpp"

namespace MultiBoost
{

// -------------------------------------------------------------------------

	OutputInfo::OutputInfo(const string& outputInfoFile)
	{
		// open the stream
		_outStream.open(outputInfoFile.c_str());

		// is it really open?
		if (!_outStream.is_open())
		{
			cerr << "ERROR: cannot open the output steam (<" << outputInfoFile << ">) for the step-by-step info!" << endl;
			exit(1);
		}
	}

// -------------------------------------------------------------------------

	void OutputInfo::outputIteration(int t)
	{
		_outStream << t;  // just output t
	}

// -------------------------------------------------------------------------

	void OutputInfo::outputError(InputData* pData, BaseLearner* pWeakHypothesis)
	{
		const int numClasses = ClassMappings::getNumClasses();
		const int numExamples = pData->getNumExamples();

		if (_gTableMap.find(pData) == _gTableMap.end())
		{
			// if it's the first time it sees this data
			// it creates and initializes a new table

			table& g = _gTableMap[pData];
			g.resize(numExamples);

			for (int i = 0; i < numExamples; ++i)
				g[i].resize(numClasses, 0);
		}

		table& g = _gTableMap[pData];

		int numWrongs = 0;

		// Compute the training error
		for (int i = 0; i < numExamples; ++i)
		{
			// the class with the highest vote
			int maxClassIdx = -1;

			// the vote of the winning class
			double maxClass = -numeric_limits<double>::max();

			for (int l = 0; l < numClasses; ++l)
			{
				// building the strong learner
				g[i][l] += pWeakHypothesis->getAlpha() *  // alpha
				        pWeakHypothesis->classify(pData, i, l);  // h_l(x)

				        // get the winner class
				if (g[i][l] > maxClass)
				{
					maxClass = g[i][l];
					maxClassIdx = l;
				}

			}

			// if the winner class is not the actual class, then it is
			// an error
			if (maxClassIdx != pData->getClass(i))
				++numWrongs;
		}

		// The error must be bounded between 0 and 1
		_outStream << '\t' << (double) (numWrongs) / (double) (numExamples);
	}

// -------------------------------------------------------------------------

	void OutputInfo::outputMargins(InputData* pData, BaseLearner* pWeakHypothesis)
	{
		const int numClasses = ClassMappings::getNumClasses();
		const int numExamples = pData->getNumExamples();

		if (_margins.find(pData) == _margins.end())
		{
			// if it's the first time it sees this data
			// it creates and initializes a new table

			table& margins = _margins[pData];
			margins.resize(numExamples);

			for (int i = 0; i < numExamples; ++i)
				margins[i].resize(numClasses, 0);
		}

		// Same for the sums of alpha. If it is the first time it
		// sees this data, it initialize the sums of the alpha for it
		if (_alphaSums.find(pData) == _alphaSums.end())
			_alphaSums[pData] = 0;

		table& margins = _margins[pData];

		double minMargin = numeric_limits<double>::max();
		double belowZeroMargin = 0;

		for (int i = 0; i < numExamples; ++i)
		{
			for (int l = 0; l < numClasses; ++l)
			{
				// hy = +1 if the classification it is correct, -1 otherwise
				double hy = pWeakHypothesis->classify(pData, i, l) *  // h_l(x_i)
				        pData->getBinaryClass(i, l);  // y_i

				        // compute the margin
				margins[i][l] += pWeakHypothesis->getAlpha() * hy;

				// gets the margin below zero
				if (margins[i][l] < 0)
				{
					if (l == pData->getClass(i))
						belowZeroMargin += (1.0 / static_cast<double>(2 * numExamples));
					else
						belowZeroMargin += (1.0 / static_cast<double>(2 * numExamples * (numClasses - 1)));
				}

				// get the minimum margin among classes and examples
				if (margins[i][l] < minMargin)
					minMargin = margins[i][l];
			}
		}

		// compute the sums of the alphas for normalization
		_alphaSums[pData] += pWeakHypothesis->getAlpha();

		_outStream << '\t' << minMargin / _alphaSums[pData] << "\t"  // minimum margin
		           << belowZeroMargin;  // margins that are below zero
	}

// -------------------------------------------------------------------------

	void OutputInfo::outputEdge(InputData* pData, BaseLearner* pWeakHypothesis)
	{
		const int numClasses = ClassMappings::getNumClasses();
		const int numExamples = pData->getNumExamples();

		double gamma = 0;  // the edge

		for (int i = 0; i < numExamples; ++i)
		{
			for (int l = 0; l < numClasses; ++l)
			{
				// hy = +1 if the classification it is correct, -1 otherwise
				double hy = pWeakHypothesis->classify(pData, i, l) *  // h_l(x_i)
				        pData->getBinaryClass(i, l);  // y_i

				double w = pData->getWeight(i, l);

				gamma += w * hy;
			}
		}

		_outStream << '\t' << gamma;  // edge

	}

// -------------------------------------------------------------------------

}// end of namespace MultiBoost

#endif
