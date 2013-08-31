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
 * \file Serialization.h Save and load the strong hypothesis.
 */

#ifndef __SERIALIZATION_H
#define __SERIALIZATION_H

#include "io/class_mappings.cpp"
#include "utils/stream_tokenizer.h"

#include "utils/utils.cpp"

#include <sstream>
#include <algorithm> // for fill
#include <fstream> // input/output on file
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost
{

// forward declaration to avoid an include
	class BaseLearner;

	/**
	 * The serialization (saving) of the weak learners found.
	 * @see UnSerialization
	 * @date 13/11/2005
	 */
	class Serialization
	{
 	public:

		/**
		 * The constructor. Create the serialization object
		 * @param shypFileName The name of the serialized strong hypothesis file.
		 * hypotheses.
		 * @date 16/11/2005
		 */
		Serialization(const string& shypFileName);

		/**
		 * Write the header.
		 * @param weakLearnerName The name of the weak learner used to find the weak
		 * hypotheses.
		 * @date 16/11/2005
		 */
		void writeHeader(const string& weakLearnerName);

		/**
		 * Write the footer. Important because it closes the xml file.
		 * @date 27/12/2005
		 */
		void writeFooter();

		/**
		 * Save all the weak hypothesis all at once.
		 * @param weakHypotheses The vector of weak hypotheses found during training.
		 * @date 16/11/2005
		 */
		void saveHypotheses(vector<BaseLearner*>& weakHypotheses);

		/**
		 * Append the passed weak hypothesis to the file.
		 * @param iteration The iteration index.
		 * @param pWeakHypothesis The current weak hypothesis.
		 * @date 16/11/2005
		 */
		void appendHypothesis(int iteration, BaseLearner* pWeakHypothesis);

		//////////////////////////////////////////////////////////////////////////
		// helper functions for creating xml tags

		/**
		 * Create a standard xml \<tag\>value\<\\tag\>.
		 * @param tagName The name of the tag.
		 * @param value The value enclosed in the tag opening and closing.
		 * @param numTab The number of tabs before the tag. Useful for indentation.
		 * @return The string with the formatted tag and value.
		 * @date 16/11/2005
		 */
		template<typename T>
		static string standardTag(const string& tagName, const T& value, int numTab = 0);

		//////////////////////////////////////////////////////////////////////////

		/**
		 * Create a set of xml tags that describes a vector.
		 * For instance, if the vector \a values is [-1, 1, 1],
		 * with \a tagName="vArray", and the classes are ["one", "two", "three"]
		 * will return:
		 *\verbatim
		 <vArray size="3">
		 <class id="one">-1</class>
		 <class id="two">1</class>
		 <class id="three">1</class>
		 </vArray> \endverbatim
		 * @param tagName The name of the tag.
		 * @param values The vector of the values.
		 * @param numTab The number of tabs before the tag. Useful for indentation.
		 * @date 16/11/2005
		 */
		template<typename T>
		static string vectorTag(const string& tagName, const vector<T>& values, int numTab = 0);

		/**
		 * Create a string which contains only tab characters
		 * @param numTabs the number of tabs.
		 * @return The string with only tabs.
		 * @date 13/11/2005
		 */
		inline static string getTabs(int numTabs)
		{
			return string(numTabs, '\t');
		}

 	private:

		ofstream _shypFile;  //!< The strong learner file

	};

// -----------------------------------------------------------------------
// -----------------------------------------------------------------------

	template<typename T>
	string Serialization::standardTag(const string& tagName, const T& value, int numTab)
	{
		ostringstream ss;
		ss << getTabs(numTab);
		ss << "<" << tagName << ">" << value << "</" << tagName << ">";
		return ss.str();
	}

// -----------------------------------------------------------------------

	template<typename T>
	string Serialization::vectorTag(const string& tagName, const vector<T>& values, int numTab)
	{
		ostringstream ss;
		ss << getTabs(numTab);

		// The header of the vector
		ss << "<" << tagName << " size=\"" << values.size() << "\">\n";

		// The vector values
		for (int i = 0; i < (int) values.size(); ++i)
		{
			ss << getTabs(numTab + 1);
			ss << "<class id=\"" << ClassMappings::getClassNameFromIdx(i) << "\">" << values[i] << "</class>\n";
		}

		// closing tag
		ss << getTabs(numTab);
		ss << "</" << tagName << ">";  // close tag

		return ss.str();
	}

// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------

	/**
	 * The un-serialization (loading) of the weak learners found.
	 * @see Serialization
	 * @date 13/11/2005
	 */
	class UnSerialization
	{
 	public:
		/**
		 * Load the strong hypothesis file.
		 * @param shypFileName The name of the serialized strong hypothesis file.
		 * @param weakHypotheses The vector of weak hypotheses that will be filled with the weak
		 * hypotheses stored in the xml file.
		 * @param verbose The level of verbosity. Default = 1.
		 * @date 16/11/2005
		 */
		void loadHypotheses(const string& shypFileName, vector<BaseLearner*>& weakHypotheses, int verbose = 1);

		/**
		 * Move in the stream until it finds the given tag.
		 * For instance if the stream is at the beginning of
		 * \verbatim <aTag>10<\aTag><anotherTag>15<\anotherTag><thirdTag>1<\thirdTag> \endverbatim
		 * and \a tag = thirdTag, the next token returned by \a st will be 1.
		 * @param st The stream tokenizer on the file stream.
		 * @param tag The tag which to look for.
		 * @return True if the tag has been found, otherwise false.
		 * @date 16/11/2005
		 */
		static bool seekSimpleTag(nor_utils::StreamTokenizer& st, const string& tag);

		/**
		 * Move in the stream until it finds the given parameter tag.
		 * This kind of tag, differs from the simple one because it is formatted this way:
		 * \<tag param="val"\>.
		 * @param st The stream tokenizer on the file stream.
		 * @param tag The tag which to look for.
		 * @return True if the tag has been found, otherwise false.
		 * @see seekSimpleTag
		 * @date 16/11/2005
		 */
		static bool seekParamTag(nor_utils::StreamTokenizer& st, const string& tag);

		/**
		 * Move in the stream until it finds the given parameter tag, then it returns its components.
		 * This kind of tag, differs from the simple one because it is formatted this way:
		 * \<tag param="val"\>.
		 * This method returns both param and value.
		 * @param st The stream tokenizer on the file stream.
		 * @param tag The tag which to look for.
		 * @param tagParam The tag parameter to be filled.
		 * @param paramValue The value of the parameter to be filled.
		 * @return True if the tag has been found, otherwise false.
		 * @see seekSimpleTag
		 * @see seekParamTag
		 * @date 16/11/2005
		 */
		static bool seekAndParseParamTag(nor_utils::StreamTokenizer& st, const string& tag, string& tagParam, string& paramValue);

		/**
		 * Parse the given token that contains a parameter tag.
		 * This kind of tag, differs from the simple one because it is formatted this way:
		 * \<tag param="val"\>. This method returns the tag, the parameter and the value.
		 * @param str The string containing the token.
		 * @param tag The tag to be filled.
		 * @param tagParam The tag parameter to be filled.
		 * @param paramValue The value of the parameter to be filled.
		 * @date 16/11/2005
		 */
		static void parseParamTag(const string& str, string& tag, string& tagParam, string& paramValue);

		/**
		 * Move in the stream until it finds the given tag, then it returns the value
		 * following the tag.
		 * @param st The stream tokenizer on the file stream.
		 * @param tag The tag which to look for.
		 * @remark The returned value depends on the template parameter.
		 * @date 16/11/2005
		 */
		template<typename T>
		static T seekAndParseEnclosedValue(nor_utils::StreamTokenizer& st, const string& tag);

		/**
		 * Move in the stream until it finds the given tag, then parse the vector
		 * which follows the tag.
		 * @param st The stream tokenizer on the file stream.
		 * @param arrayName The name of the tag that defines the beginning of a vector.
		 * @param vecToFill The vector that will receive the data.
		 * @remark The type of data which is stored in the vector depends on how the
		 * vector itself has been declared, because this method is templetized.
		 * @date 16/11/2005
		 */
		template<typename T>
		static void seekAndParseVectorTag(nor_utils::StreamTokenizer& st, const string& arrayName, vector<T>& vecToFill);

	};

// -----------------------------------------------------------------------
// -----------------------------------------------------------------------

	template<typename T>
	T UnSerialization::seekAndParseEnclosedValue(nor_utils::StreamTokenizer& st, const string& tag)
	{
		do
		{
			// keeps getting the next token until it finds the tag
			if (nor_utils::cmp_nocase(st.next_token(), tag))
			{
				T val;
				istringstream ss(st.next_token());
				ss >> val;
				return val;
			}
		}
		while (st.has_token());

		return T();
	}

// -----------------------------------------------------------------------

	template<typename T>
	void UnSerialization::seekAndParseVectorTag(nor_utils::StreamTokenizer& st, const string& arrayName, vector<T>& vecToFill)
	{

		string tagParam, paramVal;

		// move until the tag arrayName, then get the value of the param "size"
		seekAndParseParamTag(st, arrayName, tagParam, paramVal);

		// resize the vector to vector size
		// (FIXME: not very safe!)
		vecToFill.resize(atoi(paramVal.c_str()));

		T tmpVal;
		string className;
		string tag, param;

		// parses the xml vector definition
		const string closeTag = "/" + arrayName;
		while (!nor_utils::cmp_nocase(tag, closeTag) && st.has_token())
		{
			// seek the parameter "class"
			parseParamTag(st.next_token(), tag, param, paramVal);
			if (!nor_utils::cmp_nocase(tag, "class"))
				continue;

			// convert the string into a value
			istringstream ss(st.next_token());
			ss >> tmpVal;

			// stores the value, using the mapping from the class name
			// to the class index.
			vecToFill[ClassMappings::getIdxFromClassName(paramVal)] = tmpVal;
		}

	}

// -----------------------------------------------------------------------

}// end of namespace MultiBoost

#endif // __SERIALIZATION_H
