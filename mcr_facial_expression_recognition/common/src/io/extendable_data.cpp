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

#ifndef __EXTENDABLE_DATA_CPP
#define __EXTENDABLE_DATA_CPP

#include "io/extendable_data.h"

#include "defaults.h" // for STABLE_SORT declaration
#include "utils/utils.h" // for comparePairOnSecond
#include <algorithm> // for sort
// ------------------------------------------------------------------------
namespace MultiBoost
{

	void ExtendableData::addColumn(const vector<double>& newColumnVector)
	{
		// increment the number of columns for the stored data
		++_numColumns;
		_sortedData.resize(_numColumns);

		//////////////////////////////////////////////////////////////////////////
		// Fill the new column of the sorted data vector.
		// The data is stored column-wise. The pair represents the index
		// of the example with the value.
		column& newColumn = _sortedData[_numColumns - 1];
		newColumn.reserve(_numExamples);
		for (int i = 0; i < _numExamples; ++i)
			newColumn.push_back(make_pair(i, newColumnVector[i]));

		//////////////////////////////////////////////////////////////////////////
		// Now sort the new column.  todo: for certain kind of features (2-valued or
		// 3-valued) sorting could be much more efficient with bucket sort, I might
		// want to signal it in a parameter and change the sorting action
		// accordingly.

#if STABLE_SORT
		stable_sort( newColumn.begin(), newColumn.end(),
				nor_utils::comparePairOnSecond< int, double, less<double> > );
#else
		sort(newColumn.begin(), newColumn.end(), nor_utils::comparePairOnSecond<int, double, less<double> >);
#endif
	}

// ------------------------------------------------------------------------

}// end of namespace MultiBoost

#endif
