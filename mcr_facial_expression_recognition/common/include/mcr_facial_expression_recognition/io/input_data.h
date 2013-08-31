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
 * \file InputData.h The input of the training and testing data.
 */

#ifndef __INPUT_DATA_H
#define __INPUT_DATA_H

#include <vector>
#include <map> // for class mappings
#include <utility> // for pair
#include <iosfwd> // for I/O
#include "utils/args.cpp"
#include "defaults.h" // for MB_DEBUG
#include "general.h"

using namespace std;

namespace MultiBoost
{

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

// A couple of useful typedefs
	typedef vector<pair<int, double> >::iterator vpIterator;  //!< Iterator on pair
	typedef vector<pair<int, double> >::const_iterator cvpIterator;  //!< Const iterator on pair

	/**
	 * Defines the type of input. Used in case
	 * train and test differs in any way.
	 * @date 21/11/2005
	 */
	enum eInputType
	{
		IT_TRAIN,  //!< If the input is train-type.
		IT_TEST  //!< If the input is test-type.
	};

	/**
	 * Handles the data.
	 * This class not just holds the data information but also the weights on examples
	 * and labels.
	 * It also stores the sorted data (for decision stump algorithms) if necessary.
	 *
	 * Here is an example of valid data (note: in this case the argument \b -examplelabel has been provided!):
	 * \verbatim
	 /home/music/classical/classical.00078.au	classical	5.72939e+01	2.95128e+02	6.43395e+00
	 /home/music/disco/disco.00078.au	disco	1.98315e+02	1.31341e+03	-6.15398e+00
	 /home/music/reggae/reggae.00022.au	reggae	2.51418e+02	7.68241e+02	-5.66704e+00
	 /home/music/hiphop/hiphop.00080.au	hiphop	2.62773e+02	4.83971e+02	8.80924e-01
	 /home/music/rock/rock.00015.au	rock	2.03546e+02	9.31192e+02	-7.56387e+00	1.15847e+02
	 /home/music/hiphop/hiphop.00027.au	hiphop	2.37860e+02	1.03110e+03	2.50052e-01
	 /home/music/rock/rock.00094.au	rock	2.48359e+02	1.69432e+02	-1.66508e+01
	 \endverbatim
	 * @remark The column can be separated by any white-space character. If a particular
	 * separator needs to be specified (for instance when the class name contains spaces),
	 * use the -d option.
	 * @date 05/11/2005
	 */
	class InputData
	{
 	public:

		/**
		 * The constructor. It does noting but initializing some variables.
		 * @date 12/11/2005
		 */
		InputData()
				: _numColumns(0),
				  _numExamples(0),
				  _hasExampleLabel(false),
				  _classInLastColumn(false)
		{
		}

		/**
		 * Destructor of the class InputData. It clears the memory allocated
		 * with a pointer in the Example data structure. It cannot be done by
		 * the structure itself because when I build it inside the vector the
		 * destructor of the structure is called automatically.
		 * @see Example#pValues
		 * @date 12/11/2005
		 */
		virtual ~InputData();

		/**
		 * Set the arguments of the algorithm using the standard interface
		 * of the arguments. Call this to set the arguments asked by the user.
		 * @param args The arguments defined by the user in the command line.
		 * on the derived classes.
		 * @warning It does not have a declareArguments because it is
		 * dealt by the weak learner responsible for the input data
		 * (so that the option goes under its own group).
		 * @date 14/11/2005
		 */
		virtual void initOptions(nor_utils::Args& args);

		/**
		 *	NEW method @ggiorgana 21-01-11
		 */
		virtual void loadTest(vector1Df &testData);

		/**
		 * Gets the label of the given example.
		 * @param idx The index of the example
		 * @return The class of the example [idx].
		 * @date 10/11/2005
		 */
		const int getClass(int idx) const
		{
			return _infoData[idx].classIdx;
		}

		/**
		 * Get the value of the example \a idx and column \a columnIdx.
		 * @param idx The index of the example.
		 * @param columnIdx The index of the column.
		 * @date 11/11/2005
		 */
		double getValue(int idx, int columnIdx) const
		{
			return _data[idx][columnIdx];
		}

		/**
		 * Get the label of the example.
		 * @param idx The index of the example.
		 * @return A string with the label of the example, if this has been specified with
		 * -examplelabel argument.
		 * @date 14/2/2006
		 */
		string getLabel(int idx)
		{
			return _infoData[idx].label;
		}

		/**
		 * Gets the binary label of the given example.
		 * It is defined as
		 * \f[
		 * y_{i,\ell} = \begin{cases} +1 & \mbox{ if $x_i$ belongs to class $\ell$} \\
   * -1 &  \mbox{ otherwise}. \end{cases}
		 * \f]
		 * @param idx The index of the example.
		 * @param classIdx The index of the class.
		 * @return The class of the example \a idx.
		 * @date 10/11/2005
		 */
		int getBinaryClass(int idx, int classIdx) const
		{
			return _infoData[idx].classIdx == classIdx ? 1 : -1;
		}

		/**
		 * Return the weight of class \a classIdx and example \a idx.
		 * @param idx The index of the example.
		 * @param classIdx The index of the class.
		 * @date 10/11/2005
		 */
		double getWeight(int idx, int classIdx) const
		{
			return _infoData[idx].weights[classIdx];
		}

		/**
		 * Set the value of weight of class \a classIdx and example \a idx.
		 * @param idx The index of the example.
		 * @param classIdx The index of the class.
		 * @param value The new value for the weight.
		 * @date 13/11/2005
		 */
		void setWeight(int idx, int classIdx, const double value)
		{
			_infoData[idx].weights[classIdx] = value;
		}

		int getNumColumns() const
		{
			return _numColumns;
		}   //!< Returns the number of columns.
		int getNumExamples() const
		{
			return _numExamples;
		}  //!< Returns the number of examples.

		/**
		 * Get the number of examples per class.
		 * @param classIdx The index of the class.
		 * @date 11/11/2005
		 */
		int getNumExamplesPerClass(int classIdx) const
		{
			return _nExamplesPerClass[classIdx];
		}

		/**
		 *	NEW ggiorgana
		 * @date 25-01-11
		 */
		void restartInputDataObject(void);

		/**
		 *	NEW ggiorgana
		 * @date 25-01-11
		 */
		void addClassNames(vector1Di &allLabels);

 	protected:

		int _numColumns;   //!< The number of columns (dimensions).
		int _numExamples;  //!<  The number of examples.
		vector<int> _nExamplesPerClass;   //!< The number of examples per class.

		bool _hasExampleLabel;         //!< true if each example has a "label" as the very first column.
		bool _classInLastColumn;   //!< true if the class is in the last column instead of the first.

		string _sepChars;  //!< The separation characters for the input data.

		/**
		 * Initialize the weights.
		 * The weights initialization formula is defined as:
		 * \f[
		 * w_{i,\ell}^{(1)} =  \begin{cases}
		 *     \frac{1}{2n}  & \mbox{ if $\ell$ is the correct class (if $y_{i,\ell} = 1$),} \\
   *     \frac{1}{2n(k-1)} & \mbox{ otherwise (if $y_{i,\ell} = -1$).}
		 *  \end{cases}
		 * \f]
		 * where \f$n\f$ is the number of examples and \f$k\f$ the number of classes.
		 * @see Example
		 * @see _data
		 * @date 11/11/2005
		 */
		virtual void initWeights();

		/**
		 * Performs a simple check of the input (just a line of it!)
		 * to see if it is well formatted.
		 * @param line A string with a line of the input to be evaluated.
		 * @param numColumns The number of columns that the line is supposed to have.
		 * @return true if everything is fine, otherwise false.
		 * @remark The check here is fairly simple: it just see if the number of
		 * columns is correct and if in the ones that are not class description nor
		 * filename (if asked) there are well formatted numbers.
		 * @date 10/2/2006
		 */
		bool checkInput(const string& line, int numColumns);

#if MB_DEBUG
		void checkVariances();  //!< Print a warning if there is no variance in a column.
#endif

		// --------------------------------------------------------------------

		/**
		 * Holds the "additional" data of the single example.
		 * @date 11/11/2005
		 */
		struct Example
		{
			/**
			 * The constructor that create the object example.
			 * @remark I am using a pointer for pValues because I want to avoid the extra copying
			 * for a standard vector.
			 * @param classIdx The class of the example.
			 * @param label The label of the example, if it exists.
			 * @date 11/11/2005
			 */
			Example(int classIdx, const string& label = "")
					: classIdx(classIdx),
					  label(label)
			{
			}

			vector<double> weights;  //!< The weights of the example.
			int classIdx;  //!< The class (index) of the example.
			string label;  //!< The label of the example.
		};

		vector<double*> _data;  //!< The values of the example (1..numColumns).
		vector<Example> _infoData;  //!< The vector of additional data for the examples.

	};

}  // end of namespace MultiBoost

#endif // __INPUT_DATA_H
