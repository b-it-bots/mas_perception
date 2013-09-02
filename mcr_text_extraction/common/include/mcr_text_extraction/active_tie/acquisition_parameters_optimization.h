/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

/* Definition of sharpness metrics as quality criterions for image
 acquisition optimization and Fibonacci search for global maximum
 search */

#ifndef __ACQUISITION_PARAMETERS_SEARCH__
#define __ACQUISITION_PARAMETERS_SEARCH__

#include "active_tie/sharpness_measures.h"
#include "active_tie/active_camera.h"
#include "active_tie/image_preprocessing.h"

namespace atie
{
	class SearchCriterion
	{
		/* Any estimator for image quality (regardless of the quantity to
		 optimize should implement this interface. */
 	public:
		/* Get value, should return the image quality measure by the
		 criterion when a configuration parameter is set to x_  */
		virtual double getValue(int x_)= 0;
		/* Returns the lower-bound of the parameter of interest */
		virtual int getMin()= 0;
		/* Returns the upper-bound of the parameter of interest */
		virtual int getMax()= 0;
	};

	class CameraParamSC : public SearchCriterion
	{
		/* Allows to set a parameter of the camera and meassure a quality
		 criterion once the parameter has been set */
 	protected:
		ActiveCamera* camera;
		char* param;
		int averageNFrames;
		int averageCriterionValueP;
		TemporalAverage temporalAverage;

 	public:
		CameraParamSC(ActiveCamera* camera_, char* param_);
		int getMin();
		int getMax();
	};

	/* The Tenengrad operator as image quality criterion for images
	 acquired with an active camera */
	class TenengradSC : public CameraParamSC
	{
 	private:
		TenengradSM tenengrad;
		Gradients gradients;
 	public:
		TenengradSC(ActiveCamera* camera_, char* param_);
		double getValue(int x_);
	};

	/* The graylevel variance as image quality criterion for images
	 acquired with an active camera */
	class VarianceSC : public CameraParamSC
	{
 	private:
		VarianceSM variance;
		cv::Mat gray;
 	public:
		VarianceSC(ActiveCamera* camera_, char* param_);
		double getValue(int x_);
	};

	/* The graylevel variance amplitude as image quality criterion for
	 images acquired with an active camera */
	class AmplitudeSC : public CameraParamSC
	{
 	private:
		AmplitudeSM amplitude;
		cv::Mat gray;
 	public:
		AmplitudeSC(ActiveCamera* camera_, char* param_);
		double getValue(int x_);
	};

	class FibonacciSearch
	{
 	private:
		static double goldenRatio;
		SearchCriterion* criterion;
		double fibonacci(int n_);

 	public:
		FibonacciSearch(SearchCriterion* criterion_ = NULL);
		void setCriterion(SearchCriterion* criterion_);
		int optimize();
	};
}

#endif
