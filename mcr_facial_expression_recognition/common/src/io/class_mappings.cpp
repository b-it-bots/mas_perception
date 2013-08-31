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

#ifndef __CLASS_MAPPINGS_CPP
#define __CLASS_MAPPINGS_CPP

#include "io/class_mappings.h"

#include "defaults.h" // for MB_DEBUG
#include <fstream> // for ifstream
#include <iostream> // for cerr

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

namespace MultiBoost {

// static objects
vector<string> ClassMappings::_mapIdxToClass; 
map<string, int> ClassMappings::_mapClassToIdx;
int ClassMappings::_numRegClasses = 0;

// ------------------------------------------------------------------------

int ClassMappings::addClassName(const string& className)
{
   // if we haven't seen yet this class, add it to the mapping structures
   if ( _mapClassToIdx.find(className) == _mapClassToIdx.end() )
   {
      _mapClassToIdx[className] = _numRegClasses;
      _mapIdxToClass.push_back(className);
      ++_numRegClasses;
   }
   
   return _mapClassToIdx[className];
}

// ------------------------------------------------------------------------

void ClassMappings::loadClassMapFile(const string& classMapFileName)
{
   ifstream classMapFile(classMapFileName.c_str());
   if ( !classMapFile.is_open() )
   {
      cerr << "ERROR: Cannot find classmap file <" << classMapFileName << ">!" << endl;
      exit(1);
   }

   string className;
   do {
      classMapFile >> className;
      addClassName(className);
   } while ( !classMapFile.eof() && !className.empty() );

}


// ------------------------------------------------------------------------

string ClassMappings::getClassNameFromIdx(int idx)
{
#if MB_DEBUG
   if ( idx >= _mapIdxToClass.size() )
   {
      cerr << "ERROR: trying to map a class index that does not exists. The input file" << endl;
      cerr << "might not have all the classes used for training. Use -classmap to define" << endl;
      cerr << "the list of classes instead." << endl;
      exit(1);
   }
#endif
   return _mapIdxToClass[idx]; 
}

// ------------------------------------------------------------------------

int ClassMappings::getIdxFromClassName(const string& className)
{ 
#if MB_DEBUG
   if ( _mapClassToIdx.find(className) == _mapClassToIdx.end() )
   {
      cerr << "ERROR: trying to map a class (" << className << ") that does not exists." << endl;
      cerr << "The input file might not have all the classes used for training." << endl;
      cerr << "Use -classmap to define the list of classes instead." << endl;
      exit(1);
   }
#endif
   return _mapClassToIdx[className]; 
}

// ------------------------------------------------------------------------


} // end of namespace MultiBoost

#endif
