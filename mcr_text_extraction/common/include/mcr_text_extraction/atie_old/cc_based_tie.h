/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __K_MEANS_QUANTIZATION__
#define __K_MEANS_QUANTIZATION__

#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <stdio.h>
#include <map>
#include <string>
#include <libgen.h>

#include "atie_old/util.h"
#include "atie_old/chen_niblack_binarization.h"
#include "atie_old/parameters.h"
#include "atie_old/cc_classification.h"

using namespace std;
/* using namespace Torch; */

class DetectionResults
{
 public:
	DetectionResults(int nPlanes);
	~DetectionResults();
	int *nInstancesInPlane, *nPositivesInPlane, *nFalsesInPlane;
	bool deallocated;
	void callDestructor();
	IplImage** planes;
	IplImage* segmentedImage;
	int nPlanes;
	CvRect* rectangles;
	int nRectangles;
	int currentPlane;
	int currentRectangle;
	int* nRectanglesInPlaneSum;
	int* nRectanglesInPlane;
	IplImage* getPlane(int i);
	int getNPlanes();
	int getNRectangles();
	int getNRectanglesInPlane(int p);
	CvRect* getRectangle(int p, int i);
	void addResultsNextPlane(CvSeq* cc, IplImage* plane);
};

class Segmentator
{
 protected:
	IplImage* currentPlane;
	IplImage* segmentedImage;
 public:
	Segmentator(IplImage* plane, IplImage* segmented);
	virtual int getNPlanes()= 0;
	virtual bool deliverNextPlane()= 0;
};

class ChenNiblackSegmentator : public Segmentator
{
 private:
	int planesDelivered;
	IplImage* binaryImage;
 public:
	ChenNiblackSegmentator(IplImage* img, IplImage* plane, IplImage* segmented, int t = 100, float k = 0.4);
	bool deliverNextPlane();
	int getNPlanes();
};

class ChenNiblackKMeansSegmentator : Segmentator
{
};

extern "C" DetectionResults* processImage(IplImage* imageOrig);

extern CCBasicFeatureExtractor* fe;
extern CCClassifier* classifier;

extern "C" void init();

#endif
