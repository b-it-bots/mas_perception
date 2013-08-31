#include "image_drawer.h"

ImageDrawer::ImageDrawer(bool verbose)
{
	this->verbose = verbose;
}

ImageDrawer::~ImageDrawer(void)
{
}

//Draw rectangle
void ImageDrawer::drawRectangle(IplImage* imageToDraw, CvRect &rectToDraw, int lineThickness, CvScalar &color)
{
	CvPoint upperLeftCorner, lowerRightCorner;
	upperLeftCorner.x = rectToDraw.x;
	upperLeftCorner.y = rectToDraw.y;

	lowerRightCorner.x = rectToDraw.x + rectToDraw.width;
	lowerRightCorner.y = rectToDraw.y + rectToDraw.height;

	cvDrawRect(imageToDraw, upperLeftCorner, lowerRightCorner, color, lineThickness);
}

//Draw a cross
void ImageDrawer::drawCross(IplImage* imageToDraw, CvPoint &crossCenter, int lineThickness, CvScalar &color)
{
	double a = 5;
	int x = crossCenter.x;
	int y = crossCenter.y;

	//Draw horizontal line of the cross.
	cvLine(imageToDraw, cvPoint(x - a, y), cvPoint(x + a, y), color, lineThickness);

	//Draw vertical line of the cross.
	cvLine(imageToDraw, cvPoint(x, y - a), cvPoint(x, y + a), color, lineThickness);
}

//Draw text on image.
void ImageDrawer::drawText(IplImage* imageToDraw, const char* text, CvPoint &textPosition, CvFont *font, CvScalar &color)
{
	cvPutText(imageToDraw, text, textPosition, font, color);
}
