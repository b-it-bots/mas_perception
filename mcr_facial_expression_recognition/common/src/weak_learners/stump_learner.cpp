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

#include <cassert>
#include <limits> // for numeric_limits<>
#include <cmath>

#include "weak_learners/stump_learner.h"

#include "utils/utils.cpp"
#include "io/serialization.cpp"
#include "io/sorted_data.cpp"

namespace MultiBoost
{

// ------------------------------------------------------------------------------

	void StumpLearner::declareArguments(nor_utils::Args& args)
	{
		args.declareArgument("abstention", "Activate the abstention. Available types are:\n"
		                     "  greedy: sorting and checking in O(k^2)\n"
		                     "  full: the O(2^k) full search\n"
		                     "  real: use the AdaBoost.MH with real valued predictions",
		                     1, "<type>");
	}

// ------------------------------------------------------------------------------

	void StumpLearner::initOptions(nor_utils::Args& args)
	{
		if (args.hasArgument("verbose"))
			args.getValue("verbose", 0, _verbose);

		// Set the value of theta
		if (args.hasArgument("edgeoffset"))
			args.getValue("edgeoffset", 0, _theta);

		// set abstention
		if (args.hasArgument("abstention"))
		{
			string abstType = args.getValue < string > ("abstention", 0);

			if (abstType == "greedy")
				_abstention = ABST_GREEDY;
			else if (abstType == "full")
				_abstention = ABST_FULL;
			else if (abstType == "real")
				_abstention = ABST_REAL;
			else
			{
				cerr << "ERROR: Invalid type of abstention <" << abstType << ">!!" << endl;
				exit(1);
			}
		}
	}

// ------------------------------------------------------------------------------

	InputData* StumpLearner::createInputData()
	{
		return new SortedData();
	}

// ------------------------------------------------------------------------------

	double StumpLearner::classify(InputData* pData, int idx, int classIdx)
	{
		return _v[classIdx] * phi(pData->getValue(idx, _selectedColumn), classIdx);
	}

// ------------------------------------------------------------------------------

	double StumpLearner::getEnergy(vector<sRates>& mu, double& alpha, vector<double>& v)
	{
		const int numClasses = ClassMappings::getNumClasses();

		sRates eps;

		// Get the overall error and correct rates
		for (int l = 0; l < numClasses; ++l)
		{
			eps.rMin += mu[l].rMin;
			eps.rPls += mu[l].rPls;
		}

		// assert: eps- + eps+ + eps0 = 1
		assert(eps.rMin + eps.rPls <= 1 + _smallVal && eps.rMin + eps.rPls >= 1 - _smallVal);

		double currEnergy = 0;
		if (_abstention != ABST_REAL)
		{
			if (nor_utils::is_zero(_theta))
			{
				alpha = getAlpha(eps.rMin, eps.rPls);
				currEnergy = 2 * sqrt(eps.rMin * eps.rPls);

				//for (int l = 0; l < numClasses; ++l)
				//   currEnergy += sqrt( mu[l].rMin * mu[l].rPls );
				//currEnergy *= 2;
			}
			else
			{
				alpha = getAlpha(eps.rMin, eps.rPls, _theta);
				currEnergy = exp(_theta * alpha) * (eps.rMin * exp(alpha) + eps.rPls * exp(alpha));
			}
		}

		// perform abstention
		switch (_abstention)
		{
			case ABST_GREEDY:
				// alpha and v are updated!
				currEnergy = doGreedyAbstention(mu, currEnergy, eps, alpha, v);
				break;
			case ABST_FULL:
				// alpha and v are updated!
				currEnergy = doFullAbstention(mu, currEnergy, eps, alpha, v);
				break;
			case ABST_REAL:
				// alpha and v are updated!
				currEnergy = doRealAbstention(mu, eps, alpha, v);
				break;
			case ABST_NO_ABSTENTION:
				break;
		}

		// Condition: eps_pls > eps_min!!
		if (eps.rMin >= eps.rPls)
			currEnergy = numeric_limits<double>::max();

		return currEnergy;  // this is what we are trying to minimize: 2*sqrt(eps+*eps-)+eps0
	}

// -----------------------------------------------------------------------

	double StumpLearner::doGreedyAbstention(vector<sRates>& mu, double currEnergy, sRates& eps, double& alpha, vector<double>& v)
	{
		const int numClasses = ClassMappings::getNumClasses();

		// Abstention is performed by evaluating the class-wise error
		// and the case in which one element (the one with the highest mu_pls * mu_min value)
		// is ignored, that is has v[el] = 0

		// Sorting the energies for each vote
		sort(mu.begin(), mu.end());

		bool changed;
		sRates newEps;
		double newAlpha;
		double newEnergy;

		do
		{
			changed = false;

			for (int l = 0; l < numClasses; ++l)
			{
				if (v[mu[l].classIdx] != 0)
				{
					newEps.rMin = eps.rMin - mu[l].rMin;
					newEps.rPls = eps.rPls - mu[l].rPls;
					newEps.rZero = eps.rZero + mu[l].rZero;

					if (nor_utils::is_zero(_theta))
					{
						newEnergy = 2 * sqrt(newEps.rMin * newEps.rPls) + newEps.rZero;
						newAlpha = getAlpha(newEps.rMin, newEps.rPls);
					}
					else
					{
						newAlpha = getAlpha(newEps.rMin, newEps.rPls, _theta);
						newEnergy = exp(_theta * newAlpha) * (newEps.rPls * exp(-newAlpha) + newEps.rMin * exp(newAlpha) + newEps.rZero);
					}

					if (newEnergy + _smallVal < currEnergy)
					{
						// ok, this is v = 0!!
						changed = true;

						currEnergy = newEnergy;
						eps = newEps;

						v[mu[l].classIdx] = 0;
						alpha = newAlpha;

						// assert: eps- + eps+ + eps0 = 1
						assert(eps.rMin + eps.rPls + eps.rZero <= 1 + _smallVal && eps.rMin + eps.rPls + eps.rZero >= 1 - _smallVal);
					}
				}  // if
			}  //for

		}
		while (changed);

		return currEnergy;
	}

// -----------------------------------------------------------------------

	double StumpLearner::doFullAbstention(const vector<sRates>& mu, double currEnergy, sRates& eps, double& alpha, vector<double>& v)
	{
		const int numClasses = ClassMappings::getNumClasses();

		vector<char> best(numClasses, 1);
		vector<char> candidate(numClasses);
		sRates newEps;  // candidate
		double newAlpha;
		double newEnergy;

		sRates bestEps = eps;

		for (int l = 1; l < numClasses; ++l)
		{
			// starts with an array with just one 0 (and the rest 1),
			// then two 0, then three 0, etc..
			fill(candidate.begin(), candidate.begin() + l, 0);
			fill(candidate.begin() + l, candidate.end(), 1);

			// checks all the possible permutations of such array
			do
			{

				newEps = eps;

				for (int j = 0; j < numClasses; ++j)
				{
					if (candidate[j] == 0)
					{
						newEps.rMin -= mu[j].rMin;
						newEps.rPls -= mu[j].rPls;
						newEps.rZero += mu[j].rZero;
					}
				}

				if (nor_utils::is_zero(_theta))
				{
					newEnergy = 2 * sqrt(newEps.rMin * newEps.rPls) + newEps.rZero;
					newAlpha = getAlpha(newEps.rMin, newEps.rPls);
				}
				else
				{
					newAlpha = getAlpha(newEps.rMin, newEps.rPls, _theta);
					newEnergy = exp(_theta * newAlpha) * (newEps.rPls * exp(-newAlpha) + newEps.rMin * exp(newAlpha) + newEps.rZero);
				}

				if (newEnergy + _smallVal < currEnergy)
				{
					currEnergy = newEnergy;

					best = candidate;
					alpha = newAlpha;
					bestEps = newEps;

					// assert: eps- + eps+ + eps0 = 1
					assert(newEps.rMin + newEps.rPls + newEps.rZero <= 1 + _smallVal && newEps.rMin + newEps.rPls + newEps.rZero >= 1 - _smallVal);
				}

			}
			while (next_permutation(candidate.begin(), candidate.end()));

		}

		for (int l = 0; l < numClasses; ++l)
			v[l] = v[l] * best[l];  // avoiding v[l] *= best[l] because of a (weird) warning

		eps = bestEps;

		return currEnergy;  // this is what we are trying to minimize: 2*sqrt(eps+*eps-)+eps0
	}

// -----------------------------------------------------------------------

	double StumpLearner::doRealAbstention(const vector<sRates>& mu, const sRates& eps, double& alpha, vector<double>& v)
	{
		const int numClasses = ClassMappings::getNumClasses();

		double currEnergy = 0;
		alpha = 1;  // setting alpha to 1

		if (nor_utils::is_zero(_theta))
		{
			for (int l = 0; l < numClasses; ++l)
			{
				v[l] *= getAlpha(mu[l].rMin, mu[l].rPls);
				currEnergy += sqrt(mu[l].rMin * mu[l].rPls);
			}

			currEnergy *= 2;
		}
		else
		{
			for (int l = 0; l < numClasses; ++l)
				_v[l] = getAlpha(eps.rMin, eps.rPls, _theta);

			currEnergy = exp(_theta * alpha) * (eps.rMin * exp(alpha) + eps.rPls * exp(alpha));
		}

		return currEnergy;
	}

// -----------------------------------------------------------------------

	void StumpLearner::save(ofstream& outputStream, int numTabs)
	{
		// Calling the super-class method
		BaseLearner::save(outputStream, numTabs);

		// save selectedCoulumn
		outputStream << Serialization::standardTag("column", _selectedColumn, numTabs) << endl;
		outputStream << Serialization::vectorTag("vArray", _v, numTabs) << endl;
	}

// -----------------------------------------------------------------------

	void StumpLearner::load(nor_utils::StreamTokenizer& st)
	{
		// Calling the super-class method
		BaseLearner::load(st);

		_selectedColumn = UnSerialization::seekAndParseEnclosedValue<int>(st, "column");

		// move until vArray tag
		string rawTag;
		string tag, tagParam, tagValue;

		// load vArray data
		UnSerialization::seekAndParseVectorTag(st, "vArray", _v);
	}

// -----------------------------------------------------------------------

}// end of namespace MultiBoost
