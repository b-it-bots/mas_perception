/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __GRADIENTS__
#define __GRADIENTS__

#include "active_tie/open_cv_includes.h"

namespace atie
{
	class Gradients
	{
 	private:
		/* Instead of reusing channels and max values images we have two
		 copies. It requires more storage but makes parallelization very
		 easy. Hopefuly, for video, we can just the structure will stay
		 pre-allocated in memory, so they should be all in all quite
		 efficient.
		 */
		cv::Mat channelX[3];
		cv::Mat channelY[3];
		cv::Mat maxX, maxY;
		bool extendedP;

		void getSignedGradient();

 	public:
		cv::Mat image;
		cv::Mat dxS, dyS;
		cv::Mat dxU, dyU;
		cv::Mat magnitude;
		cv::Mat orientation;

		Gradients(bool extendedP_ = false);
		void setImage(cv::Mat& image_);
		bool isExtended();
		void setExtended(bool extendedP_);
		cv::Mat& mergeChannels(cv::Mat& src_, cv::Mat* channels_, cv::Mat& max_);
		void view();
	};
}
;

#endif
