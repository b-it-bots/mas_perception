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

// Indexes: i = loop on examples
//          j = loop on columns
//          l = loop on classes
#ifndef __INPUT_DATA_CPP
#define __INPUT_DATA_CPP

#include <iostream> // for cerr
#include <algorithm> // for sort
#include <functional> // for less
#include <fstream>

#include "utils/utils.h" // for white_tabs
#include "io/input_data.h"

#include "io/class_mappings.h"

#include "general.h"	//for typedefs of vectorXDY
namespace MultiBoost
{

// ------------------------------------------------------------------------

	InputData::~InputData()
	{
		vector<double*>::iterator it;
		for (it = _data.begin(); it != _data.end(); ++it)
			delete[] *it;
	}

// ------------------------------------------------------------------------

	void InputData::initOptions(nor_utils::Args& args)
	{
		///////////////////////////////////////////////////
		// check if the input file has a filename for each example
		if (args.hasArgument("examplelabel"))
			_hasExampleLabel = true;

		///////////////////////////////////////////////////
		// check if the class is at the last column of the data file
		if (args.hasArgument("classend"))
			_classInLastColumn = true;

		if (args.hasArgument("classmap"))
		{
			string classMapName = args.getValue < string > ("classmap", 0);
			ClassMappings::loadClassMapFile(classMapName);
		}

		_sepChars = "\t\r ";  // "standard" white spaces
		if (args.hasArgument("d"))
		{
			_sepChars = args.getValue < string > ("d", 0);
			_sepChars = nor_utils::getEscapeSequence(_sepChars);
		}
	}

// ------------------------------------------------------------------------

	void InputData::restartInputDataObject(void)
	{
		delete _data[0];

		_numExamples = 0;
		_nExamplesPerClass.clear();
		_numColumns = 0;

		_data.clear();
		_infoData.clear();
	}

// ------------------------------------------------------------------------

	void InputData::loadTest(vector1Df &testData)
	{
		// This array will be filled with the values from an example.
		double* pDataArray;

		_numExamples = 1;
		_nExamplesPerClass.push_back(1);

		_numColumns = (int) testData.size();
		pDataArray = new double[_numColumns];

		for (int k = 0; k < _numColumns; k++)
			pDataArray[k] = testData[k];

		/////////////////////////

		_data.push_back(pDataArray);
		_infoData.push_back(Example(0));

		/////////////////////////

		// Initialize weights
		initWeights();
	}

// ------------------------------------------------------------------------

	void InputData::addClassNames(vector1Di &allLabels)
	{
		for (int i = 0; i < allLabels.size(); i++)
		{
			string tmpClassName;
			stringstream ss;
			ss.clear();
			ss.str("");
			ss << allLabels[i];
			tmpClassName = ss.str();

			ClassMappings::addClassName(tmpClassName);
		}
	}

// ------------------------------------------------------------------------

// Initialize weights
	void InputData::initWeights()
	{
		const int numClasses = ClassMappings::getNumClasses();

		for (int i = 0; i < _numExamples; ++i)
		{
			_infoData[i].weights.resize(numClasses);  // resize vector to the number of classes

			for (int l = 0; l < numClasses; ++l)
			{
				// basic formula for weight initialization: if the example [i] belongs to class [l]
				// then it's weights is 1 / (2*numExamples),
				// otherwise it is 1 / (2 * numExamples * (numClasses-1) )
				if (l == _infoData[i].classIdx)
					_infoData[i].weights[l] = 1.0 / (2.0 * static_cast<double>(_numExamples));
				else
					_infoData[i].weights[l] = 1.0 / (2.0 * static_cast<double>(_numExamples * (numClasses - 1)));
			}
		}
	}

// ------------------------------------------------------------------------

	bool InputData::checkInput(const string& line, int numColumns)
	{
		istringstream ss(line);
		ss.imbue(locale(locale(), new nor_utils::white_spaces(_sepChars)));

		string tmp;
		bool inputValid = true;

		if (_hasExampleLabel)
			ss >> tmp;  // filename

		if (!_classInLastColumn)
			ss >> tmp;  // class at the beginning

		for (int j = 0; j < numColumns; ++j)
		{
			if (ss.eof())
			{
				inputValid = false;
				break;
			}

			ss >> tmp;

			if (!nor_utils::is_number(tmp))
			{
				inputValid = false;
				break;
			}
		}

		if (_classInLastColumn)
			ss >> tmp;  // class
		if (tmp.empty())
			inputValid = false;

		return inputValid;
	}

// ------------------------------------------------------------------------

#if MB_DEBUG
// Print a warning if there is no variance in a column.
void InputData::checkVariances()
{
	// for each column
	for (int j = 0; j < _numColumns; ++j)
	{
		double valueChk = getValue(0, j);  // get value of the first example..
		bool hasVariance = false;

		for (int i = 1; i < _numExamples; ++i)
		{
			if ( getValue(i, j) != valueChk)
			{
				hasVariance = true;  // the value has changed.. thus variance > 0
				break;
			}
		}

		if (!hasVariance)
		cerr << "WARNING!! Column " << j << " has no variance!" << endl;
	}
}
#endif // MB_DEBUG
}
  // end of namespace MultiBoost

#endif
