#include "face_region_estimator.h"

FaceRegionEstimator::FaceRegionEstimator(bool verbose)
{
	this->verbose = verbose;
}

FaceRegionEstimator::~FaceRegionEstimator(void)
{
}

FaceRegion FaceRegionEstimator::estimateFaceBoundaries(CvPoint &LEyePos, CvPoint &REyePos)
{
	FaceRegion faceRegion;
	CvPoint upperLeftCorner;
	int faceWidth;
	int faceHeight;

	float vert_left = 0.4;	//0.4     vert_left means the left vertical edge in the rectangle for cropping.
	float vert_right = 1.8;  //1.8     vert_right means the right vertical edge in the rectangle for cropping.
	float hrz_up = 0.5;		  //1.0     hrz_up means the upper horizontal edge in the rectangle for cropping.
	float hrz_down = 1.5;		//1.8     hrz_down means the lower horizontal edge in the rectangle for cropping.

	//Computes the horizontal distance between the eyes
	int d = abs(LEyePos.x - REyePos.x);

	//compute values of face rectangle
	upperLeftCorner.x = REyePos.x - (vert_left * d);
	upperLeftCorner.y = REyePos.y - (hrz_up * d);

	faceWidth = vert_right * d;
	faceHeight = (hrz_up + hrz_down) * d;

	//Create face rectangle
	faceRegion = cvRect(upperLeftCorner.x, upperLeftCorner.y, faceWidth, faceHeight);

	return faceRegion;
}
