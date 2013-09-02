/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __SHARPNESS_MEASURES__
#define __SHARPNESS_MEASURES__

#include "active_tie/integrals.h"
#include "active_tie/gradients.h"
#include "active_tie/open_cv_includes.h"

#include <vector>

using namespace std;

namespace atie
{
	class SharpnessMeasure
	{
 	protected:
		double sharpness;
 	public:
		double getSharpness();
	};

	class AmplitudeSM : public SharpnessMeasure
	{
 	private:
		cv::Mat image;
		cv::Mat tmp;
		Integrals integrals;
 	public:
		double setImage(cv::Mat& image_);
	};

	class VarianceSM : public SharpnessMeasure
	{
 	private:
		cv::Mat image;
		Integrals integrals;
 	public:
		double setImage(cv::Mat& image_);
		double setIntegrals(Integrals& integrals_);
	};

	class TenengradSM : public SharpnessMeasure
	{
 	private:
		Gradients gradients;
		Integrals integrals;
		int threshold;
 	public:
		TenengradSM(int threshold_ = 0);
		TenengradSM(vector<cv::Mat> gradients_);
		double setGradients(Gradients& gradients_);
		void setThreshold(int threshold_);
	};

	class HistogramEntropySM : public SharpnessMeasure
	{
	};
}

#endif
