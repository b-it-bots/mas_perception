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

#ifndef __ADTREE_LEARNER_CPP
#define __ADTREE_LEARNER_CPP

#include "weak_learners/ad_tree_learner.h"

#include "io/serialization.h" // used to serialize the data
#include "io/ad_tree_data.h"

namespace MultiBoost
{

	REGISTER_LEARNER (ADTreeLearner)

// ------------------------------------------------------------------------------

	void ADTreeLearner::declareArguments(nor_utils::Args& args)
	{
		// Call the super-class method
		SingleStumpLearner::declareArguments(args);

	}

// ------------------------------------------------------------------------------

	void ADTreeLearner::initOptions(nor_utils::Args& args)
	{
		// Call the super-class method
		SingleStumpLearner::initOptions(args);
	}

// ------------------------------------------------------------------------------

	void ADTreeLearner::run(InputData* pData)
	{
		// Perform the learning here! But don't forget to update _alpha too, which is declared
		// in BaseLearner!
	}

// -----------------------------------------------------------------------

	double ADTreeLearner::classify(InputData* pData, int idx, int classIdx)
	{
		// Guess what? You have to classify here!
		// Generally you return +1 if you think that the example if index idx
		// should belong to classIdx, and -1 otherwise. Other classifiers
		// returns a range between the two.

		return 0;
	}

// -----------------------------------------------------------------------

	void ADTreeLearner::save(ofstream& outputStream, int numTabs)
	{
		// Call the super-class method
		BaseLearner::save(outputStream, numTabs);

	}

// -----------------------------------------------------------------------

	void ADTreeLearner::load(nor_utils::StreamTokenizer& st)
	{
		// Call the super-class method
		BaseLearner::load(st);
	}

// -----------------------------------------------------------------------

}// end of namespace MultiBoost

#endif
