/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __UTIL_HPP__
#define __UTIL_HPP__

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>  
#include <opencv/cxcore.h>
#include <stdio.h>
#include <math.h>

/* These macros allow for efficient access to the image elements, instead of */
/* using the built in opencv functions. We still implemented them, in despite */
/* of the available c++ interface, because we need both, C and C++ */

#define GETMC2D(scalar, image, y, x, type) \
  scalar.val[0]= ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels]; \
  scalar.val[1]= ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels + 1]; \
  scalar.val[2]= ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels + 2];

#define SETMC2D(image, scalar,  y, x, type) \
  ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels]= scalar.val[0]; \
  ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels + 1]=  scalar.val[1]; \
  ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels + 2]=  scalar.val[2];

#define GETMC2DI(scalar, image, y, x, type) \
  scalar.val[0]= ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels]; 

#define SETMC2DI(image, scalar,  y, x, type) \
  ((type *)(image->imageData + (y)*image->widthStep))[(x)*image->nChannels]= scalar.val[0];

#define SOBEL_MAGNITUDE 0
#define SOBEL_ANGLE 1

IplImage* sobel2D(IplImage* img, IplImage* result, int d, int s, int mode);
IplImage* multiChannelSobel2D(IplImage* img, IplImage* result, int d, int ssize, int mode);

#endif
