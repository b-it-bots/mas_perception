/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __INTEGRALS__
#define __INTEGRALS__

#include "active_tie/open_cv_includes.h"

namespace atie
{
	class Integrals
	{
 	private:
		cv::Mat image;
		cv::Rect ROI;
		cv::Mat sum, sqsum;
		int n;
 	public:

		Integrals();
		void setImage(cv::Mat& image_);
		void resetROI();
		void setROI(cv::Rect& ROI);
		double quickIntegral(cv::Mat& i_, cv::Rect& r_);
		double getMean();
		double getStd();
		double getVar();
		double getSum();
		double getSqSum();
		long getN();
	};
}
#endif
