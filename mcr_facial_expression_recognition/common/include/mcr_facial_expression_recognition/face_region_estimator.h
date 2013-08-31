#ifndef __FACEREGIONESTIMATOR__
#define __FACEREGIONESTIMATOR__

#include <highgui.h>
#include <cxcore.h>
#include <cv.h>

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

//using namespace visg::idt;
using namespace std;

typedef CvRect FaceRegion;

class FaceRegionEstimator
{
 public:
	FaceRegionEstimator(bool verbose);

	~FaceRegionEstimator(void);

	FaceRegion estimateFaceBoundaries(CvPoint &LEyePos, CvPoint &REyePos);

 private:
	bool verbose;

};

#endif
