#ifndef GENERAL_H
#define GENERAL_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>

using namespace ::std;

struct gaussDist
{
	float mean;
	float stdev;
};

typedef std::vector<int> vector1Di;
typedef std::vector<vector1Di> vector2Di;
typedef std::vector<vector2Di> vector3Di;
typedef std::vector<vector3Di> vector4Di;
typedef std::vector<vector4Di> vector5Di;
typedef std::vector<vector5Di> vector6Di;

typedef std::vector<float> vector1Df;
typedef std::vector<vector1Df> vector2Df;
typedef std::vector<vector2Df> vector3Df;
typedef std::vector<vector3Df> vector4Df;
typedef std::vector<vector4Df> vector5Df;
typedef std::vector<vector5Df> vector6Df;

typedef std::vector<double> vector1Dd;
typedef std::vector<vector1Dd> vector2Dd;
typedef std::vector<vector2Dd> vector3Dd;
typedef std::vector<vector3Dd> vector4Dd;
typedef std::vector<vector4Dd> vector5Dd;
typedef std::vector<vector5Dd> vector6Dd;

typedef std::vector<string> vector1Ds;
typedef std::vector<vector1Ds> vector2Ds;
typedef std::vector<vector2Ds> vector3Ds;
typedef std::vector<vector3Ds> vector4Ds;
typedef std::vector<vector4Ds> vector5Ds;
typedef std::vector<vector5Ds> vector6Ds;

#endif
