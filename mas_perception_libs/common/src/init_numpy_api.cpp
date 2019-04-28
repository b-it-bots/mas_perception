/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */

// first make clear, here we initialize the Unique_PyArray_API
#define INIT_NUMPY_ARRAY_CPP

// now include the arrayobject.h, which defines void **Unique_PyArray_API
#include <mas_perception_libs/use_numpy.h>

// call import_array()
int init_numpy()
{
    import_array1(1);     // PyError if not successful
    return 0;
}
static const int numpy_initialized = init_numpy();
