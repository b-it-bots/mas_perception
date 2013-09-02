/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include "parameters.h"

#ifndef CHEN_NIBLACK_BINARIZATION_H
#define CHEN_NIBLACK_BINARIZATION_H

#define CHEN_NIBLACK_SEQUENTIAL_SEARCH 0
#define CHEN_NIBLACK_BINARY_SEARCH 1

void correctIllumination(IplImage* image, IplImage* result, IplImage* mask);
IplImage* chenNiblackBinarization(IplImage* image, IplImage* result, float Td = configurableParameters.chenNiblackTd,
                                  float k = configurableParameters.chenNiblackK, int R = configurableParameters.chenNiblackMaxRadius, int searchMethod =
                                          CHEN_NIBLACK_BINARY_SEARCH);
int estimateRSequential(IplImage* image, IplImage* sums, IplImage* ssums, CvPoint* center, bool* homogeneous, double Td, double* mean, double* std, int R =
                                configurableParameters.chenNiblackMaxRadius);

int estimateRBSearch(IplImage* image, IplImage* sums, IplImage* ssums, CvPoint* center, bool* homogeneous, double Td, double* mean, double* std, int R =
                             CHEN_NIBLACK_MAX_RADIUS);

void chenNiblackBinarizePixel(IplImage* image, IplImage* sums, IplImage* ssums, IplImage* result, CvPoint* center, double Td, float k, int R =
                                      configurableParameters.chenNiblackMaxRadius,
                              int searchMethod = CHEN_NIBLACK_BINARY_SEARCH);

#endif
