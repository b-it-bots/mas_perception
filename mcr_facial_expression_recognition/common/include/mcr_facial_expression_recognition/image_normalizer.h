#pragma once

#define PI 3.14159

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <math.h>
#include <highgui.h>
#include <cxcore.h>
#include <cv.h>

using namespace ::std;

class ImageNormalizer
{
 public:
	ImageNormalizer(int scld_W,												//in: width of scaled image
	        int scld_H, 											//in: height of scaled image
	        bool verbose);

	~ImageNormalizer(void);

	void normalizeImage(IplImage* src,								//in:	original image
	        IplImage* dest,							  //out: normalized image
	        CvPoint LEyePos,	            //in: 2D coord of Left eye (with respect to photographed person)
	        CvPoint REyePos,	            //in: 2D coord of Right eye (with respect to photographed person)
	        CvRect &faceRegion,           //in: parameters delimiting the face rectangle.
	        bool doRotation = false,	  	//in: flag to de/activate rotation
	        bool doHistEq = false);				//in: flag to (de)activate Histogram Equalization

 private:
	CvPoint LEyePos;
	CvPoint REyePos;
	bool showImages;
	CvPoint upper_left_corner;      //Position of the upper left corner of the rectangle for cropping
	CvPoint lower_right_corner;     //Position of the lower right corner of the rectangle for cropping
	int scldImg_W;
	int scldImg_H;
	bool verbose;

	//Private functions
	float computeLineOfEyesAngle();

	void rotateImage(IplImage* src, IplImage* &dest, float angle, CvPoint2D32f img_center);

	void computePositionOfEyesAfterRotating(float angle, CvPoint2D32f *img_center);

	void cropImage(IplImage* src, IplImage* &dest, CvRect &faceRegion);

	void computePositionOfEyesAfterCropping();

	void prepareFaceRectangle(CvRect &face_rectg);			//out: face rectangle														

	void scaleImage(IplImage* src, IplImage* &dest);

	void computePositionOfEyesAfterScaling();

	void histEqualization(IplImage* src, IplImage* &dest);

	void convertFrom_BGR2GRAY(IplImage* src, IplImage* &dest);

};
