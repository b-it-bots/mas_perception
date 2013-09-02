/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

/* This file include a listing of all parameters used in the
 program. Having them in one place should make everything easier to
 visualize and modify */

#ifndef __PARAMETERS__H
#define __PARAMETERS__H
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

struct ConfigurableParameters
{
	int verbosity;
	float unaryThreshold;
	float binaryThreshold;
	float chenNiblackMaxRadius;
	float chenNiblackK;
	float chenNiblackTd;
	/* int binarizationMethod; */
	/* Most of the previous parameters will dissapear, here are some new
	 more useful ones */
	bool doBinaryFiltering;
	char* unarySVMModel;
	char* unarySVMNormalization;
	int unaryNFeatures;
	int zernikeMaxOrder;
	int pseudoZernikeMaxOrder;
	bool useHuMoments;
	bool initialized;
};

void initParameters();

/* Stick to C calling conventions for external access */
extern "C" struct ConfigurableParameters configurableParameters;

/* Pz-moments */
/* #define UNARY_CLASSIFICATION_N_FEATURES 73 */
/*Hu moments*/
#define UNARY_CLASSIFICATION_N_FEATURES 14

#define ZERNIKE_ORDER 0
#define PZERNIKE_ORDER 0
#define UNARY_THRESHOLD 0
#define BINARY_THRESHOLD 0.0

/* Binarization */

#define CHEN_NIBLACK_MAX_RADIUS 100
#define CHEN_NIBLACK_DEFAULT_K 0.5
#define CHEN_NIBLACK_DEFAULT_TD 100.0
#define BINARIZATION_METHOD CHEN_NIBLACK_SEQUENTIAL_SEARCH
/* #define DEBUG_CC_QUERY_USER 1 */

#endif

