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
 * \file WeakLearnerTemplate.h A brief description of the file
 */

#ifndef __ADTREE_LEARNER_H
#define __ADTREE_LEARNER_H

#include "weak_learners/single_stump_learner.cpp"
#include "utils/args.cpp"
#include "io/input_data.cpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost
{

	/**
	 * Template weak learner class. Use it to write your own weak learner!
	 */
	class ADTreeLearner : public SingleStumpLearner
	{

 	public:

		ADTreeLearner()
				: _parent(NULL)
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
		 * @param args The Args class reference which can be used to declare
		 * additional arguments.
		 * @date 01/2/2006
		 */
		virtual void declareArguments(nor_utils::Args& args);

		/**
		 * Set the arguments of the algorithm using the standard interface
		 * of the arguments. Call this to set the arguments asked by the user.
		 * @param args The arguments defined by the user in the command line.
		 * @date 01/2/2006
		 */
		virtual void initOptions(nor_utils::Args& args);

		/**
		 * Returns itself as object.
		 * @remark It uses the trick described in http://www.parashift.com/c++-faq-lite/serialization.html#faq-36.8
		 * for the auto-registering classes.
		 * @date 01/2/2006
		 */
		virtual BaseLearner* create()
		{
			return new ADTreeLearner();
		}

		/**
		 * Creates an InputData object that it is good for the
		 * weak learner. Overridden to return SortedData.
		 * @see InputData
		 * @see BaseLearner::createInputData()
		 * @see SortedData
		 * @warning The object \b must be destroyed by the caller.
		 * @date 01/2/2006
		 */
		//virtual InputData* createInputData() { return new YourInputData(); }
		/**
		 * Run the learner to build the classifier on the given data.
		 * @param pData The pointer to the data
		 * @see BaseLearner::run
		 * @date 11/11/2005
		 */
		virtual void run(InputData* pData);

		/**
		 * Return {+1, -1} for the given class and value using the learned classifier.
		 * @param pData The pointer to the data
		 * @param idx The index of the example to classify
		 * @param classIdx The index of the class
		 * @return +1 if the classifier thinks that \a val belongs to class
		 * \a classIdx, -1 if it does not and 0 if it abstain. If ABST_REAL is selected
		 * the value returned is a range between -1 and +1 which holds the confidence
		 * in the classification.
		 * @date 11/11/2005
		 */
		virtual double classify(InputData* pData, int idx, int classIdx);

		/**
		 * Save the current object information needed for classification,
		 * that is the single threshold.
		 * @param outputStream The stream where the data will be saved
		 * @param numTabs The number of tabs before the tag. Useful for indentation
		 * @remark To fully save the object it is \b very \b important to call
		 * also the super-class method.
		 * @see StumpLearner::save()
		 * @date 11/11/2005
		 */
		virtual void save(ofstream& outputStream, int numTabs = 0);

		/**
		 * Load the xml file that contains the serialized information
		 * needed for the classification and that belongs to this class.
		 * @param st The stream tokenizer that returns tags and values as tokens
		 * @see save()
		 * @date 11/11/2005
		 */
		virtual void load(nor_utils::StreamTokenizer& st);

 	protected:

		//void  partitionData(); //!< Partition the data

		ADTreeLearner* _parent;
		vector<ADTreeLearner*> _children;

		// The base rule is composed by the decision over
		// _threshold
		// and the real values are stored in the vector
		// _v

		/**
		 * A direct access array with the size of the whole dataset, and
		 * with 1 if it belongs to this partition, and 0 otherwise.
		 */
		vector<char> _partitionedData;

	};

// ------------------------------------------------------------------------------

}// end of namespace MultiBoost

#endif // __ADTREE_LEARNER_H
