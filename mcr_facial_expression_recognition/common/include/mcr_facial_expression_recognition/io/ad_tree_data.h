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
 * \file ADTreeData.h Extension of SortedData to deal with ADTrees.
 */

#ifndef __ADTREE_DATA_H
#define __ADTREE_DATA_H

#include "io/sorted_data.cpp"
#include "weak_learners/ad_tree_learner.cpp"

using namespace std;

namespace MultiBoost
{

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

	class ADTreeData : public SortedData
	{
 	public:

		ADTreeData()
				: _pHead(NULL)
		{
		}

		/**
		 * The destructor.
		 * the object.
		 */
		virtual ~ADTreeData();

		/**
		 * Overloading of the load function that defines the top precondition.
		 * @param fileName The name of the file to be loaded.
		 * @param inputType The type of input.
		 * @param verboseLevel The level of verbosity.
		 * @see SortedData::load()
		 * @see InpuData::load()
		 * @date 21/11/2005
		 */
		virtual void load(const string& fileName, eInputType inputType = IT_TRAIN, int verboseLevel = 1);

 	protected:

		ADTreeLearner* _pHead;

	};

}  // end of namespace MultiBoost

#endif // __ADTREE_DATA_H
