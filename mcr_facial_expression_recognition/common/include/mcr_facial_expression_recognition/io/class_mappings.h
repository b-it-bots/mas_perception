/*
 * This file is part of MultiBoost, a multi-class
 * AdaBoost learner/classifier
 *
 * Copyright (C) 2005-2006 Norman Casagrande
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
 * \file ClassMappings.h The mappings of the class (labels) names of the data.
 */

#ifndef __CLASS_MAPPINGS_H
#define __CLASS_MAPPINGS_H

#include <string>
#include <vector>
#include <map>
#include <cstdlib>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost
{

	/**
	 * An static class with the class mappings.
	 * Each class (or label) is mapped into an index for efficiency reasons.
	 * @date 14/11/2005
	 */
	class ClassMappings
	{
 	public:

		/**
		 * Add a class name to the registered list.
		 * @param className The name of the class to add.
		 * @date 22/11/2005
		 */
		static int addClassName(const string& className);

		/**
		 * Load a class file. The class file contains the list of the classes
		 * that will be used during learning or classification.
		 * The format of the file is just a list of names separated by white spaces.
		 * Example:
		 * \verbatim country metal rock blues classical pop disco blues hiphop reggae \endverbatim
		 * @param classMapFileName The name of the file to load.
		 * @date 22/11/2005
		 */
		static void loadClassMapFile(const string& classMapFileName);

		/**
		 * Get the class name using the index.
		 * Example:
		 * \code
		 * getClassNameFromIdx(0); // -> "rock"
		 * \endcode
		 * @param idx The index of the class.
		 * @date 14/11/2005
		 */
		static string getClassNameFromIdx(int idx);

		/**
		 * Get the index using the class name
		 * Example:
		 * \code
		 * getIdxFromClassName("rock"); // -> 0
		 * \endcode
		 * @param className The name of the class.
		 * @date 14/11/2005
		 */
		static int getIdxFromClassName(const string& className);

		static int getNumClasses()
		{
			return _numRegClasses;
		}   //!< Returns the number of classes

 	private:
		/**
		 * Maps the internal index to the class: internal_index->class.
		 * @see getClassNameFromIdx
		 * @date 10/11/2005
		 */
		static vector<string> _mapIdxToClass;

		/**
		 * Maps the class to the internal index: class->internal_index.
		 * @see getIdxFromClassName
		 * @date 10/11/2005
		 */
		static map<string, int> _mapClassToIdx;

		static int _numRegClasses;  //!< The number of the classes registered.
	};

}  // end of namespace MultiBoost

#endif // CLASS_MAPPINGS_H
