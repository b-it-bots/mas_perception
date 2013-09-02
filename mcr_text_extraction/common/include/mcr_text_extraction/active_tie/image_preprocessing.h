/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __IMAGE_PREPROCESSING__
#define __IMAGE_PREPROCESSING__

#include "active_tie/open_cv_includes.h"
#include <vector>

using namespace std;

namespace atie
{
	class TemporalAverage
	{
 	private:
		cv::Mat sum;
		cv::Mat floatVersion;
		cv::Mat averagedFloat;
		int type;
		int returnType;
		int n;
 	public:
		cv::Mat averaged;

		void setImages(vector<cv::Mat> images_);
		void init(cv::Mat& m);
		void add(cv::Mat& m);
		cv::Mat& average();
	};
}
#endif
