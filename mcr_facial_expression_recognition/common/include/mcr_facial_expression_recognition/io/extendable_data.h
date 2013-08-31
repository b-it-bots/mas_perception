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
 * \file ExtendableData.h Input data with the possibility of adding
 * new features.
 */

#ifndef __EXTENDABLE_DATA_H
#define __EXTENDABLE_DATA_H

#include "io/sorted_data.cpp"

#include <vector>

using namespace std;

namespace MultiBoost
{

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

	/**
	 * Overloading of the SortedData class to support adding (and perhaps deleting)
	 * features (columns in the data matrix). Experimental stage.
	 * @date 15/02/2006
	 */
	class ExtendableData : public SortedData
	{
 	public:

		/**
		 * The destructor. Must be declared (virtual) for the proper destruction of
		 * the object.
		 */
		virtual ~ExtendableData()
		{
		}

		/**
		 * Add new column (feature) to the data matrix.
		 * @param newColumn The vector of the new feature values.
		 * @date 15/02/2006
		 */
		virtual void addColumn(const vector<double>& newColumnVector);
	};

}  // end of namespace MultiBoost

#endif // __EXTENDABLE_DATA_H
