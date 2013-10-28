/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __FACTORIAL_H__
#define __FACTORIAL_H__
#define NDEBUG
#include <exception>
#include <complex>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <limits>

using namespace std;

namespace ComplexMoments
{
	/* Forward declaration */
	class BaseImage;

	class PixelData
	{
 	public:
		int x, y;
		int realX, realY;
		double r, theta;
		double value;
	};

	class ImageIterator
	{
 	protected:
		int x, y;
		int beginX, beginY;
		bool isAtEnd;
		double value;
		BaseImage* image;
		virtual void updatePixelData();
 	public:
		PixelData pixel;
		ImageIterator(BaseImage& i);
		virtual bool atBegin();
		virtual bool atEnd();
		virtual void rewind();
		virtual inline void increment();
	};

	class BaseImage
	{
 	public:
		virtual int getHeight() =0;
		virtual int getWidth() =0;
		virtual int setHeight(int h) =0;
		virtual int setWidth(int w) =0;
		virtual bool setIsBinary(bool b) =0;
		virtual bool getIsBinary() =0;
		virtual double pixelAt(int y, int x) =0;
		virtual void pixelAt(int y, int x, double value)= 0;
		virtual int getCenterX()= 0;
		virtual int getCenterY()= 0;
		virtual int setCenterX(int x)= 0;
		virtual int setCenterY(int y)= 0;
		/* Iterators related methods */
		ImageIterator* getIterator();
		virtual bool isValidPixel(PixelData& p) =0;
		virtual void offsetCoordinates(PixelData& p)= 0;
		virtual void scaleR(PixelData& p)= 0;
		virtual void shiftTheta(PixelData& p)= 0;
		virtual void initializeCenter()= 0;

	};

	class Image : public virtual BaseImage
	{
		/* This class can be specialized for Ad-hoc images, e.g. OpenCV or
		 your homebrew implementation of an image. This class is only
		 useful for images whose coordinates are in cartesian space. */
 	protected:
		int width, height;
		int centerX, centerY;
		bool isBinary;
 	public:
		virtual int getHeight();
		virtual int getWidth();
		virtual int setHeight(int h);
		virtual int setWidth(int w);
		virtual bool setIsBinary(bool b);
		virtual bool getIsBinary();
		virtual inline double pixelAt(int y, int x) =0;
		virtual inline void pixelAt(int y, int x, double value) =0;
		virtual bool isValidPixel(PixelData& p);
		/* Iterators related method */
		virtual inline void offsetCoordinates(PixelData& p);
		/* Polar iterators (only) related methods */
		virtual inline void scaleR(PixelData& p);
		virtual inline void shiftTheta(PixelData& p);
		virtual void initializeCenter();
		virtual int getCenterX();
		virtual int getCenterY();
		virtual int setCenterX(int x);
		virtual int setCenterY(int y);
	};

	class WrappedImage : public virtual BaseImage
	{
		/* Needed for creating nested images */
 	protected:
		BaseImage* image;

 	public:
		WrappedImage(BaseImage* img);
		virtual inline int getHeight();
		virtual inline int getWidth();
		virtual inline int setHeight(int h);
		virtual inline int setWidth(int w);
		virtual inline bool setIsBinary(bool b);
		virtual inline bool getIsBinary();
		virtual inline double pixelAt(int y, int x);
		virtual inline void pixelAt(int y, int x, double value);
		virtual inline bool isValidPixel(PixelData& p);
		virtual inline void offsetCoordinates(PixelData& p);
		virtual inline void scaleR(PixelData& p);
		virtual inline void shiftTheta(PixelData& p);
		virtual inline void initializeCenter();
		virtual inline int getCenterX();
		virtual inline int getCenterY();
		virtual inline int setCenterX(int x);
		virtual inline int setCenterY(int y);
	};

	class GeometricalMoments
	{
 	public:
		static double getMoment(BaseImage* image, int yOrder, int xOrder);
	};

	class NormalizedImage : public virtual WrappedImage
	{
 	private:
		double m00, m01, m10;
		double maxDistance;
		int centerX, centerY;
		bool escaleToUnitCircle;
 	public:
		NormalizedImage(BaseImage* img, bool escaleToUnitCircle = true);
		double getMaxDistance();
		virtual double pixelAt(int y, int x);
		virtual void pixelAt(int y, int x, double f);
		virtual inline void offsetCoordinates(PixelData& p);
		virtual inline void scaleR(PixelData& p);
		virtual inline void shiftTheta(PixelData& p);
		void initializeCenter();
		double getMass();
		double getM01();
		double getM10();
		virtual inline int getCenterX();
		virtual inline int getCenterY();
		virtual inline int setCenterX(int x);
		virtual inline int setCenterY(int y);
		virtual inline void setMaxDistance(double distance);
		virtual bool isValidPixel(PixelData& p);
	};

	class OpenCVImage : public Image
	{
 	private:
		IplImage* image;
		CvRect roc;
 	public:
		OpenCVImage(IplImage* i, bool binary = false);
		inline double pixelAt(int y, int x);
		inline void pixelAt(int y, int x, double value);
	};

	class Factorial
	{
		/* A factorial lookup table. This is important to speed common
		 operations needed for calculating some kinds of moments */
 	private:
		static vector<double> factorials;
		static double fillUpTable(int n);
 	public:
		static inline double value(int n);
	};

	struct _ComplexMoment
	{
		double rel, img;
		double invariant;
		int n, m;
	};

	typedef struct _ComplexMoment ComplexMoment;

#define  CRPG_MODE_SINGLE 0
#define  CRPG_MODE_GIVEN 1
#define  CRPG_MODE_UPTO 2

	class ComplexRadialPolynomialGenerator
	{
 	protected:
		int mode;
		/* maxOrder, singleN and singleM are needed for initialization */
		int maxOrder;
		int singleN, singleM;
		int* givenOrders;
		bool forPatternRecognition;

		ComplexMoment* features;
		int nFeatures;
		complex<double>* integrals;
		virtual double commonTerm(int n, int m)= 0;
		virtual void fillFeatureVectorOrdersUpTo()= 0;

 	public:
		virtual int nMomentsForMaxOrder(int order)= 0;
		ComplexRadialPolynomialGenerator(int nOrders, bool forPatternRecognition = false);
		ComplexRadialPolynomialGenerator(int* orders, int nOrders, bool forPatternRecognition = false);
		ComplexRadialPolynomialGenerator(int n, int m, bool forPatternRecognition = false);
		virtual bool validNM(int n, int m)= 0;
		virtual double radialPolynomial(int n, int m, float r)= 0;
		virtual void initialize();
		void momentSet(NormalizedImage* image);
		int getNFeatures();
		ComplexMoment* getFeatures();
		virtual void reconstruct(NormalizedImage* result, double mass, double scaling = 1);
	};

	class PZM : public virtual ComplexRadialPolynomialGenerator
	{
 	private:
		double radialPolynomial(int n, int m, float r);
		double commonTerm(int n, int m);
		void fillFeatureVectorOrdersUpTo();
 	public:
		int nMomentsForMaxOrder(int order);
		PZM(int order, bool forPatternRecognition = false);
		PZM(int* orders, int nOrders, bool forPatternRecognition = false);
		PZM(int n, int m, bool forPatternRecognition = false);
		bool validNM(int n, int m);

	};

	class Zernike : public virtual ComplexRadialPolynomialGenerator
	{
 	private:
		inline double radialPolynomial(int n, int m, float r);
		inline double commonTerm(int n, int m);
		void fillFeatureVectorOrdersUpTo();

 	public:
		virtual int nMomentsForMaxOrder(int order);
		Zernike(int order, bool forPatternRecognition = false);
		Zernike(int* orders, int nOrders, bool forPatternRecognition = false);
		Zernike(int n, int m, bool forPatternRecognition = false);
		inline virtual bool validNM(int n, int m);
	};

}  // End of package scope
#endif
