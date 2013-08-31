#include "image_normalizer.h"

// Constructor of the class
// 

ImageNormalizer::ImageNormalizer(int scld_W,											//in: width of scaled image
        int scld_H,                   	//in: height of scaled image
        bool verbose)                   //in: define if verbose text is displayed or not.
{
	this->LEyePos.x = 0;
	this->LEyePos.y = 0;
	this->REyePos.x = 0;
	this->REyePos.y = 0;
	this->scldImg_W = scld_W;
	this->scldImg_H = scld_H;

	this->showImages = false;
	this->verbose = verbose;
}

// Destructor of the class

ImageNormalizer::~ImageNormalizer(void)
{
}

/**
 /*	This function returns the normalized version (dest) of the src image.
 /*	It will first rotate, then translate and finally scale the src image
 /*	in order to obtain the normalized image.
 **/
void ImageNormalizer::normalizeImage(IplImage* src,				//in:	original image
        IplImage* dest,			//out: normalized image
        CvPoint LEyePosin,	    //in: 2D coord of Left eye (with respect to photographed person)
        CvPoint REyePosin,	    //in: 2D coord of Right eye (with respect to photographed person)
        CvRect &faceRegion,   //in: parameters delimiting the face rectangle.
        bool doRotation,			//in: flag to (de)activate rotation
        bool doHistEq)				//in: flag to (de)activate Histogram Equalization
{
	cout << endl << "normalizeImage2" << endl;
	this->LEyePos.x = LEyePosin.x;
	this->LEyePos.y = LEyePosin.y;
	this->REyePos.x = REyePosin.x;
	this->REyePos.y = REyePosin.y;
	cout << endl << "normalizeImage" << endl;
	IplImage* rotated_img = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);
	cout << endl << "normalizeImage s2" << endl;

	if (doRotation == true)
	{
		//rotate		
		if (verbose)
			cout << endl << "Rotating" << endl;

		CvPoint2D32f img_center;
		img_center.x = src->width / 2;
		img_center.y = src->height / 2;
		float angle = computeLineOfEyesAngle();

		this->rotateImage(src, rotated_img, angle, img_center);
		this->computePositionOfEyesAfterRotating(angle, &img_center);

		if (showImages == true)
		{
			cvNamedWindow("rotated", 1);
			cvShowImage("rotated", rotated_img);
			cvSaveImage("rotated_+3.jpg", rotated_img);
			cvWaitKey(0);
			cvDestroyWindow("rotated");
		}
	}
	else
	{
		cvCopyImage(src, rotated_img);
	}

	//Crop
	cout << endl << "Cropping" << endl;
	if (verbose)
		cout << endl << "Cropping" << endl;

	IplImage* cropped_img;
	this->cropImage(rotated_img, cropped_img, faceRegion);
	this->computePositionOfEyesAfterCropping();

	if (showImages)
	{
		cvNamedWindow("cropped", 1);
		cvShowImage("cropped", cropped_img);
		cvSaveImage("cropped.jpg", cropped_img);
		cvWaitKey(0);
		cvDestroyWindow("cropped");
	}

	/*cout<<"upper_left_corner.x: " <<upper_left_corner.x <<endl;
	 cout<<"upper_left_corner.y: " <<upper_left_corner.y <<endl <<endl;
	 cout<<"lower_right_corner.x: " <<lower_right_corner.x <<endl;
	 cout<<"lower_right_corner.y: " <<lower_right_corner.y <<endl <<endl;*/

	//scale
	if (verbose)
		cout << endl << "Scaling" << endl;

	this->scaleImage(cropped_img, dest);
//	this->computePositionOfEyesAfterScaling();

	if (showImages)
	{
		cvNamedWindow("scaled", 1);
		cvShowImage("scaled", dest);
		cvSaveImage("scaled.jpg", dest);
		cvWaitKey(0);
		cvDestroyWindow("scaled");
	}

	/*cout<<"new lower_right_corner.x: " <<lower_right_corner.x <<endl;
	 cout<<"new lower_right_corner.y: " <<lower_right_corner.y <<endl <<endl;*/

	//do Histogram Equalization
	if (verbose)
		cout << endl << "Equalizing image histogram" << endl;

	//this->histEqualization(dest, dest);

	if (showImages == true)
	{
		cvNamedWindow("histEqzld", 1);
		cvShowImage("histEqzld", dest);
		cvSaveImage("histEqzld.jpg", dest);
		cvWaitKey(0);
		cvDestroyWindow("histEqzld");
	}

	cvReleaseImage(&rotated_img);
	cvReleaseImage(&cropped_img);
}

///PRIVATE

/**
 /*	This functions will compute the rotation angle of the image.
 /*
 /*	Positive values of slope mean counter-clockwise rotation (the coordiate
 /*	origin is assumed at top-left corner).
 /*
 /*	angle: is the rotation angle in degress.
 **/
float ImageNormalizer::computeLineOfEyesAngle()
{
	float slope = (float) (this->LEyePos.y - this->REyePos.y) / (this->LEyePos.x - this->REyePos.x);
	float angle = atan(slope) * 180 / PI;

	/*cout<<endl <<"slope: " <<slope <<endl;
	 cout<<"angle: " <<angle <<endl;*/

	return angle;
}

void ImageNormalizer::rotateImage(IplImage* src, IplImage* &dest, float angle, CvPoint2D32f img_center)
{
	float isoScale = 1;																		//Isotropic scale factor.

	CvMat *rotMat = cvCreateMat(2, 3, CV_32FC1);					//2�3 map matrix.

	cv2DRotationMatrix(img_center, angle, isoScale, rotMat);
	cvWarpAffine(src, dest, rotMat);

	cvReleaseMat(&rotMat);
}

void ImageNormalizer::computePositionOfEyesAfterRotating(float angle, CvPoint2D32f *img_center)
{
	float angle_rad = angle * 3.14159 / 180;
	float alpha = cos(angle_rad);
	float beta = sin(angle_rad);
	float factor1 = (1 - alpha) * img_center->x - beta * img_center->y;
	float factor2 = (beta * img_center->x) + ((1 - alpha) * img_center->y);

	float temp1 = 0, temp2 = 0;

	//Left Eye
	temp1 = (-beta * this->LEyePos.x);
	temp2 = (alpha * this->LEyePos.y);

	this->LEyePos.x = (alpha * this->LEyePos.x) + (beta * this->LEyePos.y) + factor1;
	this->LEyePos.y = temp1 + temp2 + factor2;

	//Right Eye	
	temp1 = (-beta * this->REyePos.x);
	temp2 = (alpha * this->REyePos.y);

	this->REyePos.x = (alpha * this->REyePos.x) + (beta * this->REyePos.y) + factor1;
	this->REyePos.y = temp1 + temp2 + factor2;
}

void ImageNormalizer::prepareFaceRectangle(CvRect &face_rectg)
{
	int faceWidth = 0;
	int faceHeight = 0;

	float vert_left = 0.4;	//0.4     vert_left means the left vertical edge in the rectangle for cropping.
	float vert_right = 1.8;	//1.8     vert_right means the right vertical edge in the rectangle for cropping.
	float hrz_up = 0.5;	    //1.0     hrz_up means the upper horizontal edge in the rectangle for cropping.
	float hrz_down = 1.5;		//1.8     hrz_down means the lower horizontal edge in the rectangle for cropping.

	//Computes the horizontal distance between the eyes
	int d = abs(this->LEyePos.x - this->REyePos.x);

	//compute values of face rectangle
	this->upper_left_corner.x = this->REyePos.x - (vert_left * d);
	this->upper_left_corner.y = this->REyePos.y - (hrz_up * d);
	faceWidth = vert_right * d;
	faceHeight = (hrz_up + hrz_down) * d;

	//create face rectangle
	face_rectg = cvRect(this->upper_left_corner.x, this->upper_left_corner.y, faceWidth, faceHeight);

	//fill lower_right_corner structure
	this->lower_right_corner.x = upper_left_corner.x + faceWidth;
	this->lower_right_corner.y = this->upper_left_corner.y + faceHeight;
}

void ImageNormalizer::cropImage(IplImage* src, IplImage* &dest, CvRect &faceRegion)
{
	dest = cvCloneImage(src);							//dest = ROI.
	cvSetImageROI(dest, faceRegion);

	/*if(showCroppedImages==true)
	 {
	 cvNamedWindow( "cropped", CV_WINDOW_AUTOSIZE );
	 cvShowImage( "cropped", ROI);
	 if(saveCroppedImages==true)
	 cvSaveImage("cropped.jpg",ROI);
	 }*/
}

void ImageNormalizer::scaleImage(IplImage* src, IplImage* &dest)
{
	cvResize(src, dest, CV_INTER_AREA);
}

void ImageNormalizer::histEqualization(IplImage* src, IplImage* &dest)
{
	cvEqualizeHist(src, dest);
}

void ImageNormalizer::convertFrom_BGR2GRAY(IplImage* src, IplImage* &dest)
{
	cvCvtColor(src, dest, CV_BGR2GRAY);
}

void ImageNormalizer::computePositionOfEyesAfterCropping()
{
	//Left Eye
	this->LEyePos.x = this->LEyePos.x - this->upper_left_corner.x;
	this->LEyePos.y = this->LEyePos.y - this->upper_left_corner.y;

	//Right Eye	
	this->REyePos.x = this->REyePos.x - this->upper_left_corner.x;
	this->REyePos.y = this->REyePos.y - this->upper_left_corner.y;

	lower_right_corner.x = lower_right_corner.x - this->upper_left_corner.x;
	lower_right_corner.y = lower_right_corner.y - this->upper_left_corner.y;
}

void ImageNormalizer::computePositionOfEyesAfterScaling()
{
	//Left Eye

	this->LEyePos.x = (this->LEyePos.x * this->scldImg_W) / lower_right_corner.x;
	this->LEyePos.y = (this->LEyePos.y * this->scldImg_H) / lower_right_corner.y;

	//Right Eye

	this->REyePos.x = (this->REyePos.x * this->scldImg_W) / lower_right_corner.x;
	this->REyePos.y = (this->REyePos.y * this->scldImg_H) / lower_right_corner.y;
}
