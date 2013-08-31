#include "gf_extractor_on_image.h"

GFExtractorOnImage::GFExtractorOnImage(void)
{
}

GFExtractorOnImage::~GFExtractorOnImage(void)
{
}

void GFExtractorOnImage::extractFeaturesOnImage(IplImage *image, vector1Df &gFV)
{
	const int numberOfLambdas = 5;
	const int numberOfOrientations = 8;

	float lambda[numberOfLambdas] =
	{ 3.5, 4.5, 7.5, 10.5, 15.5 };
	float orientation[numberOfOrientations] =
	{ 0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5 };

	float bandwidth = 1;
	float psi = 0;
	float gamma = 1;

	int pixelValue = 0;
	int numFilter = 0;

	bool showMagnitudeResponseImage = false;
	bool showRealKernel = false;
	bool showImagKernel = false;

	IplImage *magresp = cvCreateImage(cvSize(image->width, image->height), image->depth, image->nChannels);

	if (!gFV.empty())
		gFV.clear();

	for (int i = 0; i < numberOfLambdas; i++)
	{
		for (int j = 0; j < numberOfOrientations; j++)
		{
			CvGaborWiki gabor1(lambda[i], orientation[j], psi, bandwidth, gamma);
			gabor1.conv_img(image, magresp, CV_GABOR_MAG);

			if (showMagnitudeResponseImage)
			{
				cout << "frequency: " << i << "  orientation: " << j << endl;

				cvNamedWindow("magresp", CV_WINDOW_AUTOSIZE);
				cvShowImage("magresp", magresp);
				cvWaitKey(10);
			}

			if (showRealKernel)
			{
				IplImage* real = gabor1.get_image(1);
				cvNamedWindow("real", CV_WINDOW_AUTOSIZE);
				cvShowImage("real", real);
				cvWaitKey(0);

				cvReleaseImage(&real);
			}

			if (showImagKernel)
			{
				IplImage* imag = gabor1.get_image(2);
				cvNamedWindow("magresp", CV_WINDOW_AUTOSIZE);
				cvShowImage("magresp", magresp);
				cvWaitKey(0);

				cvReleaseImage(&imag);
			}

			fillGaborFeatureVector(magresp, gFV);
		}
	}
	cvReleaseImage(&magresp);
}

void GFExtractorOnImage::fillGaborFeatureVector(IplImage *magresp, vector1Df &gFV)
{
	int height = magresp->height;
	int width = magresp->width;
	int pixelValue = 0;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			pixelValue = ((uchar*) (magresp->imageData + magresp->widthStep * y))[x];
			gFV.push_back(pixelValue);
		}
	}
}
