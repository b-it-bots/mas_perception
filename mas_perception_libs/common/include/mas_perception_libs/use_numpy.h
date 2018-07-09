/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 * following suggestion from https://stackoverflow.com/a/47027598
 * to create single numpy API reference
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
