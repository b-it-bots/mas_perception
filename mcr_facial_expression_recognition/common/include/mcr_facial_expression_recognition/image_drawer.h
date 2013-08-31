#pragma once

#include <highgui.h>
#include <cxcore.h>
#include <cv.h>

#define RED cvScalar(0, 0, 255)
#define GREEN cvScalar(0, 255, 0)
#define BLUE cvScalar(255, 0, 0)

class ImageDrawer
{
 public:
	ImageDrawer(bool verbose);
	~ImageDrawer(void);

	void drawRectangle(IplImage* imageToDraw, CvRect &rectToDraw, int lineThickness, CvScalar &color);

	void drawCross(IplImage* imageToDraw, CvPoint &crossCenter, int lineThickness, CvScalar &color);

	void drawText(IplImage* imageToDraw, const char* text, CvPoint &textPosition, CvFont *font, CvScalar& color);

 private:
	bool verbose;
};
