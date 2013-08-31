#pragma once

#include <highgui.h>
#include <cxcore.h>
#include <cv.h>

class ImageVisualizer
{
 public:
	ImageVisualizer(bool verbose);
	~ImageVisualizer(void);

	void displayImage(IplImage* imageToDisplay, const char* windowName);

 private:
	bool verbose;
};
