/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __FEATURE_EXTRACTORS__
#define __FEATURE_EXTRACTORS__

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <list>
#include "util.h"
#include <libsvm/svm.h>
#include "parameters.h"

using namespace std;

class ccRelatedData
{
 public:
	CvSeq* cc;
	CvSeq* firstCC;
	IplImage* original;
	IplImage* plane;
	IplImage* rendered;
	IplImage* renderedFilled;
	IplImage* natural;
	IplImage* gray;
	IplImage* gradient;
};

/* Feature extraction functions prototypes */
double* getNormalizedCentralMoments(ccRelatedData* ccd, int* n);
double* getOcuppyRatio(ccRelatedData* ccd, int* n);
double* getCompactness(ccRelatedData* ccd, int* n);
double* howManyHolesInCC(ccRelatedData* ccd, int* n);
double* howManyCCContains(ccRelatedData* ccd, int* n);
double* aspectRatio(ccRelatedData* ccd, int* n);
double* getHomogeneity(ccRelatedData* ccd, int* n);
double* getEdgeContent(ccRelatedData* ccd, int* n);
double* getEdgeContrast(ccRelatedData* ccd, int* n);
double* getContourRoughness(ccRelatedData* ccd, int* n, int operation = CV_MOP_OPEN);
double* getContourRoughnessOpen(ccRelatedData* ccd, int* n);
double* getContourRoughnessClose(ccRelatedData* ccd, int* n);
double* getHuMoments(ccRelatedData* ccd, int* n);
double* getHorizontalStrokeStatistics(ccRelatedData* ccd, int* n);
double* getVerticalStrokeStatistics(ccRelatedData* ccd, int* n);
int countWhitePixels(IplImage* binaryImage);

void inline strokeStatisticsMainHelper(bool& inStrokeRun, int& inStrokeRunWidth, int& nStrokes, list<int>& strokeWidths, int& sumStrokeWidths, CvScalar& pixel);
void inline strokeStatisticsScanLineEndHelper(bool& inStrokeRun, int& inStrokeRunWidth, int& sumStrokeWidths, list<int>& strokeWidths);

void inline strokeStatisticsLastHelper(ccRelatedData* ccd, int& nStrokes, int& sumStrokeWidths, list<int>& strokeWidths, double* result);

/* ==================== Binary classification ==================== */

class BinaryCCRelatedData
{
 public:
	CvSeq* cc1, *cc2;
	CvRect br1, br2;
	IplImage* image;
};

/* Feature extraction function prototypes */
void getCCCenterCoordinates(CvRect* bb, CvPoint* p);
double* getCCCentroidDistance(BinaryCCRelatedData* b, int* n);
double* getNormalizedCCCentroidDistance(BinaryCCRelatedData* b, int* n);
double* getScaleDifference(BinaryCCRelatedData* b, int* n);
double* getOverlapDegree(BinaryCCRelatedData* b, int* n);
double* getShapeDifference(BinaryCCRelatedData* b, int* n);
double* getCCCentroidAngle(BinaryCCRelatedData* b, int* n);

#endif
