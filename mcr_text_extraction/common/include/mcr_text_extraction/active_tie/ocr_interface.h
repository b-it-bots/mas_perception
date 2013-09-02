/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __OCR_INTERFACE__
#define __OCR_INTERFACE__

#include <vector>
#include "active_tie/open_cv_includes.h"

using namespace std;

namespace atie
{
	class OCRInterface
	{
 	public:
		virtual void cleanResults();
		virtual string recognize(cv::Mat& image_)= 0;
		virtual string readResults();
	};

	class AbbyOCR : public OCRInterface
	{
 	public:
		string recognize(cv::Mat& image_);
	};

	class TesseractOCR : public OCRInterface
	{
 	public:
		string recognize(cv::Mat& image_);
	};
}

#endif
