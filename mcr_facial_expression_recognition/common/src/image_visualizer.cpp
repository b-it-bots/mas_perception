#include "image_visualizer.h"

ImageVisualizer::ImageVisualizer(bool verbose)
{
	this->verbose;
}

ImageVisualizer::~ImageVisualizer(void)
{
}

void ImageVisualizer::displayImage(IplImage* imageToDisplay, const char* windowName)
{
	cvNamedWindow(windowName, 1);
	cvShowImage(windowName, imageToDisplay);
	cvWaitKey(10);
	//cvDestroyWindow("Image");
}
