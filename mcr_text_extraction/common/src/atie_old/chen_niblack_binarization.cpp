/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */


#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  
#include <cxcore.h>
#include <stdio.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <libgen.h>
#include <iostream>

#include "atie_old/util.h"
#include "atie_old/chen_niblack_binarization.h"

using namespace std;
#define TT double


#define STDDEV(sx, ssx, n, mean) sqrt(ssx / n -  (mean * mean))

int inline sumsInWindow(IplImage* image, IplImage* sums, IplImage* ssums, 
			CvPoint* center, int r, TT* sx, TT* ssx) {
  CvRect roi= cvGetImageROI(image);
  double value;
  CvScalar p1, p2, p3, p4;
  int halfSize= (int) r / 2;


  int x1= center->x - halfSize;
  int y1= center->y - halfSize;
  int x2= center->x + halfSize;
  int y2= center->y + halfSize;
      
  GETMC2DI(p1, sums, y2, x2, TT);
  GETMC2DI(p2, sums, y2, x1, TT);
  GETMC2DI(p3, sums, y1, x2, TT);
  GETMC2DI(p4, sums, y1, x1, TT);
  
  (*sx)= p1.val[0] - p2.val[0] - p3.val[0] + p4.val[0];
  
  GETMC2DI(p1, ssums, y2, x2, TT);
  GETMC2DI(p2, ssums, y2, x1, TT);
  GETMC2DI(p3, ssums, y1, x2, TT);
  GETMC2DI(p4, ssums, y1, x1, TT);
  
  (*ssx)= p1.val[0] - p2.val[0] - p3.val[0] + p4.val[0];
  double numberOfPixels= r * r;
  return numberOfPixels;
}



int inline estimateRSequential(IplImage* image, IplImage* sums, 
			       IplImage* ssums, CvPoint* center, bool* homogeneous,
			       double Td, double* mean, double* std, int R) {
  TT sx, ssx;
  int n, r;  
  for (r=3; r < R; r+=2) {
    n= sumsInWindow(image, sums, ssums, center, r, &sx, &ssx);
    (*mean)= sx/n;
    (*std)= STDDEV(sx, ssx, n, (*mean));
    if ((*std) > Td) { 
      return r;
    }
  }
  (*homogeneous)= true;
  return r;
}

int inline estimateRBSearch(IplImage* image, IplImage* sums, 
			    IplImage* ssums, CvPoint* center, bool* homogeneous, 
			    double Td, double* mean, double* std, int R) {

  TT sx, ssx;
  int n, r;  

  n=sumsInWindow(image, sums, ssums, center, R, &sx, &ssx);
  (*mean)= sx/n;
  (*std)= STDDEV(sx, ssx, n, (*mean));
  return R;
}


void inline chenNiblackBinarizePixel(IplImage* image, IplImage* sums, IplImage* ssums, 
			      IplImage* result, CvPoint* center, 
			       double Td, float k, int R, int searchMethod) {

  CvScalar black= cvScalar(0, 0, 0, 0);
  CvScalar white= cvScalar(255, 0, 0, 0);
  CvScalar gray= cvScalar(100, 0, 0, 0);

  double mean, std;
  int r;
  bool homogeneous= false;
  if (searchMethod == CHEN_NIBLACK_SEQUENTIAL_SEARCH) {
    r= estimateRSequential(image, sums, ssums, center, &homogeneous, Td, &mean, &std, R);
  }
  else {
    r= estimateRBSearch   (image, sums, ssums, center, &homogeneous, Td, &mean, &std, R);
  }
  CvScalar p;
  double value;

  GETMC2DI(p, image, center->y, center->x, uchar);
  value= p.val[0];
  if (homogeneous) {
    SETMC2DI(result, gray, center->y, center->x, uchar);
  }
  if (value > (mean + k * std)) {
    SETMC2DI(result, white, center->y, center->x, uchar);
  }
  else if (value < (mean - k * std)){ 
    SETMC2DI(result, black, center->y, center->x, uchar);
  } 
  else {    
    SETMC2DI(result, gray, center->y, center->x, uchar);
  }
}

IplImage* chenNiblackBinarization(IplImage* image, IplImage* result, 
				  float Td, float k, int R, int searchMethod) {
  int x, y;
  CvRect roi= cvGetImageROI(image);
  CvSize size= cvSize(roi.width, roi.height);
  cvCvtColor(image, result, CV_BGR2GRAY);
  CvSize extendedSize= cvSize(roi.width + R, roi.height + R);
  IplImage* extendedGray= cvCreateImage(extendedSize, 8, 1);
  cvCopyMakeBorder(result, extendedGray, cvPoint((R-1)/2,(R-1)/2),
		   IPL_BORDER_REPLICATE);
  IplImage* sums= cvCreateImage(cvSize(extendedSize.width + 1, extendedSize.height + 1), IPL_DEPTH_64F, 1);
  IplImage* ssums= cvCreateImage(cvSize(extendedSize.width + 1, extendedSize.height + 1), IPL_DEPTH_64F, 1);
  cvIntegral(extendedGray,sums, ssums, NULL);

  CvPoint center;
  IplImage* extendedResult= cvCreateImage(extendedSize, 8, 1);

  for (y=0; y < roi.height ; y++)
    for (x=0; x < roi.width; x++){
      center.x= x + R / 2; center.y= y + R / 2;
      chenNiblackBinarizePixel(extendedGray, sums, ssums, 
			       extendedResult, &center, Td, k, R, searchMethod);
    }
/* Copy the ROI of the extended image into the gray level image (we do
   not need so we can return it, releasing it and allocating an extra
   image. */
  CvRect resultROI;
  resultROI.x= R / 2;
  resultROI.y= R / 2;
  resultROI.width= roi.width;
  resultROI.height= roi.height;
  cvSetImageROI(extendedResult, resultROI);
  cvCopy(extendedResult, result);

  cvReleaseImage(&extendedResult);
  
  cvReleaseImage(&extendedGray);
  cvReleaseImage(&sums);
  cvReleaseImage(&ssums);
  return result;
}


#ifdef USE_MAIN
int main(int argc, char** argv){
  char* imagePath= argv[1];
  IplImage* image= cvLoadImage(imagePath, -1);  

  if (argc >= 4) {
    if (!strncmp(argv[2], "binary", 6) || !strncmp(argv[2], "sequen", 6)) {
      /* We are requested to use particular parameters for the search method
	 and the maximum window size */
      int mode= (!strncmp(argv[2], "binary", 6)) ? CHEN_NIBLACK_BINARY_SEARCH :
	CHEN_NIBLACK_SEQUENTIAL_SEARCH;
      int R= atoi(argv[3]);
      printf("Binarize tune mode \n");
      IplImage* binary= cvCreateImage(cvGetSize(image), 8, 1);
      if (mode == CHEN_NIBLACK_BINARY_SEARCH) printf("Optimized search \n\n");
      
      chenNiblackBinarization(image, binary,
			      CHEN_NIBLACK_DEFAULT_TD,
			      CHEN_NIBLACK_DEFAULT_K,
			      R, mode);
      cvReleaseImage(&binary);
    }
    else  {
      printf("Evaluation mode \n");
      int R= atoi(argv[3]);
      char* outputPath= argv[4];
      // First, binarize the image with bisection
      IplImage* bisectionImage= cvCreateImage(cvGetSize(image), 8, 1);
      IplImage* sequentialImage= cvCreateImage(cvGetSize(image), 8, 1);
      IplImage* diffImage= cvCreateImage(cvGetSize(image), 8, 1);
      clock_t start, finish;
      start = clock();
      cout << "Start clock " <<  start << endl;
      chenNiblackBinarization(image, bisectionImage,
			      /* CHEN_NIBLACK_DEFAULT_TD, */
			      0.0,
			      0.8,
			      /* CHEN_NIBLACK_DEFAULT_K, */
			      R, CHEN_NIBLACK_BINARY_SEARCH);
      
      finish = clock();
      cout << "End clock " << finish << endl;
      long bisectionTime= (finish - start);
      cout << "Binary search took " << bisectionTime << endl;

      start = clock();
      cout << "start clock " << start << endl;
      chenNiblackBinarization(image, sequentialImage,
			      /* CHEN_NIBLACK_DEFAULT_TD, */
			      70.0,
			      0.5,
			      /* CHEN_NIBLACK_DEFAULT_K, */
			      R, CHEN_NIBLACK_SEQUENTIAL_SEARCH);
      finish = clock();
      cout << "End clock " <<  finish << endl;
      long sequentialTime= (finish - start);///CLOCKS_PER_SEC;
      cout << "Sequential search took  " << sequentialTime << endl;
      // Compute the difference image
      cvCmp(sequentialImage, bisectionImage, diffImage, CV_CMP_NE);
      float diff= cvCountNonZero(diffImage) * 100.0f / (image->width * image->height);
      /* Create file names for the ouput images using the following rules syntax:
	 <SSB-search> ::= <output-path>"/"<image-name>"_["<time>","<R>"ssb]"
	 <S-search> ::= <output-path>"/"<image-name>"_["<time>","<R>",s]"
	 <diff-image> ::= <output-path>"/"<image-name>"_["<percentage>",diff]"
      */
      /* char* ssbPath= (char*)calloc(strlen(outputPath) + 9, sizeof(char)); */
      /* char* sPath= (char*)calloc(strlen(outputPath) + 9, sizeof(char)); */
      /* char* diffPath= (char*)calloc(strlen(outputPath) + 9, sizeof(char)); */
      
      char* allPaths[3];
      char* marks[]= {"ssb", "s", "diff"};
      char* extraInfo[3];
      IplImage* allImages[3]= {bisectionImage, sequentialImage, diffImage};
      extraInfo[0]= (char*) calloc(20, sizeof(char));
      extraInfo[1]= (char*) calloc(20, sizeof(char));
      extraInfo[2]= (char*) calloc(20, sizeof(char));
      sprintf(extraInfo[0], "%u", bisectionTime);
      sprintf(extraInfo[1], "%u", sequentialTime);
      sprintf(extraInfo[2], "%0.4f", diff);
      for (int i=0; i < 3; i++) {
	allPaths[i]= (char*) calloc(strlen(outputPath) + 40, sizeof(char));
	sprintf(allPaths[i], "%s/%s_[%s,%i,%s].png",
		outputPath, basename(imagePath),
		extraInfo[i], R, marks[i]);
	cvSaveImage(allPaths[i], allImages[i]);	
	/* free (&extraInfo[i]); */
	/* free (&allPaths[i]); */
      }
      cvReleaseImage(&bisectionImage);
      cvReleaseImage(&sequentialImage);
      cvReleaseImage(&diffImage);
    }
  }
  else {
    /* Binarize with default parameters */
    printf("Using default parameters \n");
    IplImage* binary= cvCreateImage(cvGetSize(image), 8, 1);
    chenNiblackBinarization(image, binary);
    cvNamedWindow("input", 0);
    cvShowImage("input", image);
    cvNamedWindow("result", 0);
    cvShowImage("result", binary);
    while(cvWaitKey(0) != '\n') {}
    cvReleaseImage(&binary);      
  }
  cvReleaseImage(&image);
}

void homogeneizeIllumination(IplImage* image, IplImage* result, IplImage* plane, IplImage* mask= NULL) {
  CvMemStorage* storage= cvCreateMemStorage(0);
  CvSeq* firstCC;
  IplImage* planeTmp= cvCloneImage(plane);
  cvFindContours(planeTmp, storage, &firstCC, sizeof(CvContour),
		 CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
  /* Get the mean value of the RGB value f each contour */
  /*Iterate contours */
  CvSeq* ccc;
  CvScalar white= cvScalar(255,255,255, 0);
  for (ccc= firstCC; ccc; ccc= ccc->h_next) {
    CvRect br= cvBoundingRect(ccc);
    if (br.height < 15) {
      if(mask){
	cvDrawContours(mask, ccc, white, white, 0, CV_FILLED);
	cvSetImageROI(mask,br);
	cvSetImageROI(plane,br);
	cvOr(mask, plane, mask);
	cvResetImageROI(mask);
	cvResetImageROI(plane);
      }
      continue;
    }
    cvSetImageROI(planeTmp, br);
    cvZero(planeTmp);
    cvResetImageROI(planeTmp);
    cvDrawContours(planeTmp, ccc, white, white, 0, CV_FILLED);
    cvSetImageROI(planeTmp, br);
    cvSetImageROI(plane, br);
    cvSetImageROI(image, br);
    
    cvAnd(planeTmp, plane, planeTmp);
    int i;
    CvPoint* p;
    CvScalar rgb;
    CvScalar sum= cvScalar(0,0,0,0);
    CvScalar isOn;
    int n=0;
    for(int y=br.y; y < br.y + br.height; y++) 
      for(int x=br.x; x < br.x + br.width; x++) {	
	GETMC2DI(isOn, planeTmp, y, x, uchar);	
	if (isOn.val[0] != 255) continue;
	n++;
	GETMC2DI(rgb, image, y, x, uchar);
	sum.val[0]+= rgb.val[0];
	sum.val[1]+= rgb.val[1];
	sum.val[2]+= rgb.val[2];
      }
    sum.val[0] /= (float) n;
    sum.val[1] /= (float) n;
    sum.val[2] /= (float) n;
    /* Iterate each pixel in the contour and replace the value with
       the mean */
    for(int y=br.y; y < br.y + br.height; y++) 
      for(int x=br.x; x < br.x + br.width; x++) {
	GETMC2DI(isOn, planeTmp, y, x, uchar);
	if (isOn.val[0] != 255) continue;
	SETMC2DI(result, sum, y, x, uchar);
      }
  }  
  cvReleaseImage(&planeTmp);
  cvResetImageROI(plane);
  if (mask) cvResetImageROI(mask);
  cvResetImageROI(image);
}
void correctIllumination(IplImage* image, IplImage* result, IplImage* mask=NULL) {
  CvSize size= cvGetSize(image);
  IplImage* whitePlane= cvCreateImage(size, 8, 1);
  IplImage* blackPlane= cvCreateImage(size, 8, 1);
  IplImage* grayPlane= cvCreateImage(size, 8, 1);
  IplImage* binary= cvCreateImage(size, 8, 1);
  chenNiblackBinarization(image, binary);
  cvCmpS(binary, 100, grayPlane, CV_CMP_EQ);
  cvCmpS(binary, 255, whitePlane, CV_CMP_EQ);
  cvCmpS(binary, 0, blackPlane, CV_CMP_EQ);
  homogeneizeIllumination(image, result, blackPlane, NULL);
  homogeneizeIllumination(image, result, whitePlane, NULL);
  homogeneizeIllumination(image, result, grayPlane, NULL);
  cvReleaseImage(&binary);
  cvReleaseImage(&blackPlane);
  cvReleaseImage(&grayPlane);
  cvReleaseImage(&whitePlane);
}

#endif

