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

#ifndef __BASE_LEARNER_CPP
#define __BASE_LEARNER_CPP

#include <cmath> // for log
#include "weak_learners/base_learner.h"
#include "utils/utils.h" // for is_zero
#include "io/serialization.h" // for the helper function "standardTag"
namespace MultiBoost
{

// -----------------------------------------------------------------------

	void BaseLearner::declareBaseArguments(nor_utils::Args& args)
	{
		args.setGroup("I/O Options");

		args.declareArgument("d",
		                     "The separation characters between the fields (default: whitespaces).\nExample: -d \"\\t,.-\"\nNote: new-line is always included!",
		                     1, "<separators>");
		args.declareArgument("classend", "The class is the last column instead of the first (or second if -examplelabel is active).");
		args.declareArgument("classmap", "Specify a file with the list of classes. Use it when the test set has less classes than the training set.", 1,
		                     "<filename>");
		args.declareArgument("examplelabel", "The data file has an additional column (the very first) which contains the label of the example.");
		args.declareArgument("shypname", "The name of output strong hypothesis (default: " + string(SHYP_NAME) + "." + string(SHYP_EXTENSION) + ").", 1,
		                     "<filename>");

		args.setGroup("Basic Algorithm Options");

		args.declareArgument("edgeoffset", "Defines the value of the edge offset (theta) (default: no edge offset).", 1, "<val>");
		args.declareArgument("resume", "Resumes a training process using the strong hypothesis file.", 1, "<shypFile>");
	}

// -----------------------------------------------------------------------

	InputData* BaseLearner::createInputData()
	{
		return new InputData();
	}

// -----------------------------------------------------------------------

	double BaseLearner::getAlpha(double error)
	{
		return 0.5 * log((1 - error) / error);
	}

// -----------------------------------------------------------------------

	double BaseLearner::getAlpha(double eps_min, double eps_pls)
	{
		return 0.5 * log((eps_pls + _smoothingVal) / (eps_min + _smoothingVal));
	}

// -----------------------------------------------------------------------

	double BaseLearner::getAlpha(double eps_min, double eps_pls, double theta)
	{
		// if theta == 0
		if (nor_utils::is_zero(theta))
			return getAlpha(eps_min, eps_pls);

		const double eps_zero = 1 - eps_min - eps_pls;

		if (eps_min < _smallVal)
		{
			// if eps_min == 0
			return log(((1 - theta) * eps_pls) / (theta * eps_zero));
		}
		else
		{
			// ln( -b + sqrt( b^2 + c) );
			const double denom = (1 + theta) * eps_min;
			const double b = ((theta) * eps_zero) / (2 * denom);
			const double c = ((1 - theta) * eps_pls) / denom;

			return log(-b + sqrt(b * b + c));
		}

	}

// -----------------------------------------------------------------------

	void BaseLearner::save(ofstream& outputStream, int numTabs)
	{
		// save alpha
		outputStream << Serialization::standardTag("alpha", _alpha, numTabs) << endl;
	}

// -----------------------------------------------------------------------

	void BaseLearner::load(nor_utils::StreamTokenizer& st)
	{
		_alpha = UnSerialization::seekAndParseEnclosedValue<double>(st, "alpha");
	}

// -----------------------------------------------------------------------

}// end of namespace MultiBoost

#endif
