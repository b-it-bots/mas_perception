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
 * \file SortedData.h Input data which has the column sorted.
 */

#ifndef __SORTED_DATA_H
#define __SORTED_DATA_H

#include "io/input_data.cpp"

#include <vector>
#include <utility> // for pair
using namespace std;

namespace MultiBoost
{

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

	/**
	 * Overloading of the InputData class to support sorting of the column.
	 * This is particularly useful for stump-based learner, because
	 * they work column-by-column (dimension-by-dimension), looking for a threshold
	 * that minimizes the error, and sorting the data it's mandatory.
	 * The connection between this class and the weak learner that implements
	 * decision stump, is done with the overriding of method BaseLearner::createInputData()
	 * which will return the desired InputData type (and which might depend on
	 * the arguments of the command line too).
	 * @see BaseLearner::createInputData()
	 * @see StumpLearner::createInputData()
	 * @date 21/11/2005
	 */
	class SortedData : public InputData
	{
 	public:

		/**
		 * The destructor. Must be declared (virtual) for the proper destruction of
		 * the object.
		 */
		virtual ~SortedData()
		{
		}

		/**
		 * Overloading of the load function to support sorting.
		 * @param fileName The name of the file to be loaded.
		 * @param inputType The type of input.
		 * @param verboseLevel The level of verbosity.
		 * @see InputData::load()
		 * @date 21/11/2005
		 * @todo Erase the original memory once sorted?
		 */
		virtual void load(const string& fileName, eInputType inputType = IT_TRAIN, int verboseLevel = 1);

		virtual void loadTrain(vector3Df &trainData, vector1Di &labels);

		/**
		 * Get the first element of the (sorted) column of the data.
		 * @param colIdx The column index
		 * @return The iterator to the first element of the column
		 * @date 10/11/2005
		 */
		virtual vpIterator getSortedBegin(int colIdx)
		{
			return _sortedData[colIdx].begin();
		}

		/**
		 * Get the last element of the (sorted) column of the data.
		 * @param colIdx The column index
		 * @return The iterator to the last element (end()) of the column
		 * @remark It is the end() iterator, so it does not point to anything!
		 * @date 10/11/2005
		 */
		virtual vpIterator getSortedEnd(int colIdx)
		{
			return _sortedData[colIdx].end();
		}

 	protected:

		/**
		 * A column of the data.
		 * The pair represents the index of the example and the value of the column.
		 * The index of the column is the index of the vector itself.
		 * @remark I am storing both the index and the value because it is a trade off between.
		 * speed in a key part of the code (finding the threshold) and the memory consumption.
		 * In case of very large databases, this could be turned into a index only vector.
		 * @date 11/11/2005
		 */
		typedef vector<pair<int, double> > column;

		vector<column> _sortedData;  //!< the sorted data.

	};

}  // end of namespace MultiBoost

#endif // __SORTED_DATA_H
