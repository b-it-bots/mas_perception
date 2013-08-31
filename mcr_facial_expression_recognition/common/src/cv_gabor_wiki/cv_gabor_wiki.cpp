/***************************************************************************
 *   Copyright (C) 2006 by Mian Zhou   *
 *   M.Zhou@reading.ac.uk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "cv_gabor_wiki/cv_gabor_wiki.h"

CvGaborWiki::CvGaborWiki()
{
}

CvGaborWiki::~CvGaborWiki()
{
	cvReleaseMat(&Real);
	cvReleaseMat(&Imag);
}

/**
 *
 *	@brief:
 *		Construct a Gabor filter.
 *
 *	@params
 *			lambda Wavelength of the complex sinusoidal wave (in pixels/cycle).
 *			phi Orientation of the sinusoid carrier (in degrees)		[0 pi)
 *			psi Phase offset (in degrees)
 *			sigma Standard deviation of the Gaussian envelope in x and y direction (in pixels)
 *			gamma Spatial aspect ratio
 */
CvGaborWiki::CvGaborWiki(float lambda, float phi, float psi, float bandwidth, float gamma)
{
	if (lambda < 2)
		throw "ERROR: Lambda value is lower than 2 pixels per cycle.";

	this->lambda = lambda;
	this->phi = phi;
	this->psi = psi;
	this->bandwidth = 1;  //bandwidth;
	this->gamma = gamma;

	// Convert degrees to radians.
	this->phi *= (PI / 180);
	this->psi *= (PI / 180);

	//Initilise the parameters    
	this->sigma = computeSigma();
	Width = mask_width();
	Real = cvCreateMat(Width, Width, CV_32FC1);
	Imag = cvCreateMat(Width, Width, CV_32FC1);
	bKernel = false;
	creat_kernel();
}

float CvGaborWiki::computeSigma(void)
{
	return 0.56 * lambda;		// just in case bandwidth = 1;
}

/*!
 \fn CvGaborWiki::mask_width()
 Give out the width of the mask

 Parameters:
 None

 Returns:
 The long type show the width.

 Return the width of mask (should be NxN) by the value of Sigma and iNu.
 */
long CvGaborWiki::mask_width()
{
	long lWidth;

	//determine the width of Mask
	double dWidth = sigma * 6;

	// round dWidth.
	dWidth = (int) (dWidth + 0.5);

	//test whether dWidth is an odd.
	if (fmod(dWidth, 2.0) == 0.0)
		dWidth++;

	lWidth = (long) dWidth;

	return lWidth;
}

/*!
 \fn CvGaborWiki::creat_kernel()
 Create gabor kernel

 Parameters:
 None

 Returns:
 None

 Create 2 gabor kernels - REAL and IMAG, with an orientation and a scale
 */
void CvGaborWiki::creat_kernel()
{
	CvMat *mReal, *mImag;
	mReal = cvCreateMat(Width, Width, CV_32FC1);
	mImag = cvCreateMat(Width, Width, CV_32FC1);

	/**************************** Gabor Function ****************************/
	double dReal = 0;
	double dImag = 0;
	double dTemp1 = 0, dTemp2 = 0, dTemp3 = 0;

	int x = 0, y = 0;
	int x0 = (Width - 1) / 2;			// Center of the image
	int y0 = (Width - 1) / 2;			// Center of the image
	float x_prime = 0;
	float y_prime = 0;

	for (int i = 0; i < Width; i++)
	{
		for (int j = 0; j < Width; j++)
		{
			x = i - x0;
			y = j - y0;

			// compute rotated x and y.
			x_prime = x * cos(phi) + y * sin(phi);
			y_prime = -x * sin(phi) + y * cos(phi);

			// compute common part
			dTemp1 = exp(-(pow(x_prime, 2) + (pow(gamma, 2) * pow(y_prime, 2))) / (2 * pow(sigma, 2)));

			// compute non-common factors.
			dTemp2 = cos((2 * PI * x_prime / lambda) + psi);
			dTemp3 = sin((2 * PI * x_prime / lambda) + psi);

			// compute the real and the imaginary part.
			dReal = dTemp1 * dTemp2;
			dImag = dTemp1 * dTemp3;

			// Save value in kernel image.
			cvSetReal2D((CvMat*) mReal, i, j, dReal);							//improve: use direct access.
			cvSetReal2D((CvMat*) mImag, i, j, dImag);							//improve: use direct access.
		}
	}
	/**************************** Gabor Function ****************************/

	cvCopy(mReal, Real, NULL);																	//improve: Fill Real directly, not using mReal.
	cvCopy(mImag, Imag, NULL);																	//improve: Fill Imag directly, not using mImag.

	cvReleaseMat(&mReal);
	cvReleaseMat(&mImag);

	bKernel = true;
}

/*!
 \fn CvGaborWiki::get_image(int Type)
 Get the speific type of image of Gabor

 Parameters:
 Type            The Type of gabor kernel, e.g. REAL, IMAG, MAG, PHASE

 Returns:
 Pointer to image structure, or NULL on failure

 Return an Image (gandalf image class) with a specific Type   "REAL"     "IMAG" "MAG" "PHASE"
 */
IplImage* CvGaborWiki::get_image(int Type)
{
	if (IsKernelCreate() == false)
		throw "ERROR: the Gabor kernel has not been created in get_image()!";

	IplImage* pImage;
	IplImage *newimage;
	newimage = cvCreateImage(cvSize(Width, Width), IPL_DEPTH_8U, 1);
	//printf("Width is %d.\n",(int)Width);
	//printf("Sigma is %f.\n", Sigma);
	//printf("F is %f.\n", F);
	//printf("Phi is %f.\n", Phi);

	//pImage = gan_image_alloc_gl_d(Width, Width);
	pImage = cvCreateImage(cvSize(Width, Width), IPL_DEPTH_32F, 1);

	CvMat* kernel = cvCreateMat(Width, Width, CV_32FC1);
	double ve;
	CvScalar S;
	CvSize size = cvGetSize(kernel);
	int rows = size.height;
	int cols = size.width;
	switch (Type)
	{
		case 1:  //Real

			cvCopy((CvMat*) Real, (CvMat*) kernel, NULL);
			//pImage = cvGetImage( (CvMat*)kernel, pImageGL );
			for (int i = 0; i < rows; i++)
			{
				for (int j = 0; j < cols; j++)
				{
					ve = cvGetReal2D((CvMat*) kernel, i, j);
					cvSetReal2D((IplImage*) pImage, j, i, ve);
				}
			}
			break;
		case 2:  //Imag
			cvCopy((CvMat*) Imag, (CvMat*) kernel, NULL);
			//pImage = cvGetImage( (CvMat*)kernel, pImageGL );
			for (int i = 0; i < rows; i++)
			{
				for (int j = 0; j < cols; j++)
				{
					ve = cvGetReal2D((CvMat*) kernel, i, j);
					cvSetReal2D((IplImage*) pImage, j, i, ve);
				}
			}
			break;
		case 3:  //Magnitude
			///@todo
			break;
		case 4:  //Phase
			///@todo
			break;
	}

	cvNormalize((IplImage*) pImage, (IplImage*) pImage, 0, 255, CV_MINMAX, NULL);

	cvConvertScaleAbs((IplImage*) pImage, (IplImage*) newimage, 1, 0);

	cvReleaseMat(&kernel);

	cvReleaseImage(&pImage);

	return newimage;
}

/*!
 \fn CvGaborWiki::IsKernelCreate()
 Determine the gabor kernel is created or not

 Parameters:
 None

 Returns:
 a boolean value, TRUE is created or FALSE is non-created.

 Determine whether a gabor kernel is created.
 */
bool CvGaborWiki::IsKernelCreate()
{

	return bKernel;
}

/*!
 \fn CvGaborWiki::get_mask_width()
 Reads the width of Mask

 Parameters:
 None

 Returns:
 Pointer to long type width of mask.
 */
long CvGaborWiki::get_mask_width()
{
	return Width;
}

/*!
 \fn CvGaborWiki::get_matrix(int Type)
 Get a matrix by the type of kernel

 Parameters:
 Type            The type of kernel, e.g. REAL, IMAG, MAG, PHASE

 Returns:
 Pointer to matrix structure, or NULL on failure.

 Return the gabor kernel.
 */
CvMat* CvGaborWiki::get_matrix(int Type)
{
	if (!IsKernelCreate())
	{
		perror("Error: the gabor kernel has not been created!\n");
		return NULL;
	}
	switch (Type)
	{
		case CV_GABOR_REAL:
			return Real;
			break;
		case CV_GABOR_IMAG:
			return Imag;
			break;
		case CV_GABOR_MAG:
			return NULL;
			break;
		case CV_GABOR_PHASE:
			return NULL;
			break;
	}
}

/*!
 \fn CvGaborWiki::output_file(const char *filename, Gan_ImageFileFormat file_format, int Type)
 Writes a gabor kernel as an image file.

 Parameters:
 filename        The name of the image file
 file_format     The format of the file, e.g. GAN_PNG_FORMAT
 Type            The Type of gabor kernel, e.g. REAL, IMAG, MAG, PHASE
 Returns:
 None

 Writes an image from the provided image structure into the given file and the type of gabor kernel.
 */
void CvGaborWiki::output_file(const char *filename, int Type)
{
	IplImage *pImage;
	pImage = get_image(Type);
	if (pImage != NULL)
	{
		if (cvSaveImage(filename, pImage))
			printf("%s has been written successfully!\n", filename);
		else
			printf("Error: writting %s has failed!\n", filename);
	}
	else
		perror("Error: the image is empty in output_file()!\n");

	cvReleaseImage(&pImage);
}

/*!
 \fn CvGaborWiki::show(int Type)
 */
void CvGaborWiki::show(int Type)
{
	//    IplImage *pImage;
	//pImage = get_image(Type);
	//cvNamedWindow("Testing",1);
	//cvShowImage("Testing",pImage);
	//cvWaitKey(0);
	//cvDestroyWindow("Testing");
	//cvReleaseImage(&pImage);
}

/*!
 \fn CvGaborWiki::normalize( const CvArr* src, CvArr* dst, double a, double b, int norm_type, const CvArr* mask )
 */
void CvGaborWiki::normalize(const CvArr* src, CvArr* dst, double a, double b, int norm_type, const CvArr* mask)
{
	CvMat* tmp = 0;
	__BEGIN__;

	double scale, shift;

	if (norm_type == CV_MINMAX)
	{
		double smin = 0, smax = 0;
		double dmin = MIN(a, b), dmax = MAX(a, b);
		cvMinMaxLoc(src, &smin, &smax, 0, 0, mask);
		scale = (dmax - dmin) * (smax - smin > DBL_EPSILON ? 1. / (smax - smin) : 0);
		shift = dmin - smin * scale;
	}
	else if (norm_type == CV_L2 || norm_type == CV_L1 || norm_type == CV_C)
	{
		CvMat *s = (CvMat*) src, *d = (CvMat*) dst;

		scale = cvNorm(src, 0, norm_type, mask);
		scale = scale > DBL_EPSILON ? 1. / scale : 0.;
		shift = 0;
	}
	else
	{
	}

	if (!mask)
		cvConvertScale(src, dst, scale, shift);
	else
	{
		CvMat stub, *dmat;

		cvConvertScale(src, tmp, scale, shift);
		cvCopy(tmp, dst, mask);
	}

	__END__;

	if (tmp)
		cvReleaseMat(&tmp);
}

/*!
 \fn CvGaborWiki::conv_img(IplImage *src, IplImage *dst, int Type)
 */
void CvGaborWiki::conv_img(IplImage *src, IplImage *dst, int Type)
{
	double ve, re, im;

	CvMat *mat = cvCreateMat(src->width, src->height, CV_32FC1);
	for (int i = 0; i < src->width; i++)
	{
		for (int j = 0; j < src->height; j++)
		{
			ve = CV_IMAGE_ELEM(src, uchar, j, i);
		CV_MAT_ELEM(*mat, float, i, j) = (float)ve;
	}
}

CvMat *rmat = cvCreateMat(src->width, src->height, CV_32FC1);
CvMat *imat = cvCreateMat(src->width, src->height, CV_32FC1);

switch (Type)
{
	case CV_GABOR_REAL:
		cvFilter2D((CvMat*) mat, (CvMat*) mat, (CvMat*) Real, cvPoint((Width - 1) / 2, (Width - 1) / 2));
		break;
	case CV_GABOR_IMAG:
		cvFilter2D((CvMat*) mat, (CvMat*) mat, (CvMat*) Imag, cvPoint((Width - 1) / 2, (Width - 1) / 2));
		break;
	case CV_GABOR_MAG:
		cvFilter2D((CvMat*) mat, (CvMat*) rmat, (CvMat*) Real, cvPoint((Width - 1) / 2, (Width - 1) / 2));
		cvFilter2D((CvMat*) mat, (CvMat*) imat, (CvMat*) Imag, cvPoint((Width - 1) / 2, (Width - 1) / 2));

		cvPow(rmat, rmat, 2);
		cvPow(imat, imat, 2);
		cvAdd(imat, rmat, mat);
		cvPow(mat, mat, 0.5);
		break;
	case CV_GABOR_PHASE:
		break;
}

if (dst->depth == IPL_DEPTH_8U)
{
	cvNormalize((CvMat*) mat, (CvMat*) mat, 0, 255, CV_MINMAX);
	for (int i = 0; i < mat->rows; i++)
	{
		for (int j = 0; j < mat->cols; j++)
		{
			ve = CV_MAT_ELEM(*mat, float, i, j);
			CV_IMAGE_ELEM(dst, uchar, j, i) = (uchar) cvRound(ve);
		}
	}
}

if (dst->depth == IPL_DEPTH_32F)
{
	for (int i = 0; i < mat->rows; i++)
	{
		for (int j = 0; j < mat->cols; j++)
		{
			ve = cvGetReal2D((CvMat*) mat, i, j);
			cvSetReal2D((IplImage*) dst, j, i, ve);
		}
	}
}
cvReleaseMat(&imat);
cvReleaseMat(&rmat);
cvReleaseMat(&mat);
}
