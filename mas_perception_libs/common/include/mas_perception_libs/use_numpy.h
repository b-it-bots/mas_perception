/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 * following suggestion from https://stackoverflow.com/a/47027598 to create a single numpy API reference.
 * All 'cpp' files that use NumPy API need to include this header. Directly including of NumPy header will cause
 * segmentation fault error as multiple instances of 'PyArray_API' are initialized.
 * Works in conjunction with 'common/src/init_numpy_api.cpp', where a single instance of 'MY_PyArray_API' is initialized
 * using the 'import_array' call
 *
 */
#ifndef MAS_PERCEPTION_LIBS_USE_NUMPY_H
#define MAS_PERCEPTION_LIBS_USE_NUMPY_H

// single symbol for PyArrayAPI
#define PY_ARRAY_UNIQUE_SYMBOL Unique_PyArray_API

// this macro must be defined for the translation unit
#ifndef INIT_NUMPY_ARRAY_CPP
#define NO_IMPORT_ARRAY     // for usual translation units
#endif

// include the numpy-arrays:
#include <numpy/arrayobject.h>

#endif  // MAS_PERCEPTION_LIBS_USE_NUMPY_H
