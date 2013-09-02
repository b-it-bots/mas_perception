/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __ACTIVE_CAMERA__

#define __ACTIVE_CAMERA__

#include <string>
#include <map>
#include <boost/thread.hpp>
#include <stdarg.h>

#include "active_tie/open_cv_includes.h"
#include "active_tie/active_camera_values.h"

using namespace std;

namespace atie
{

	class ActiveCamera
	{
 	protected:
		int nParameters;
		int* currentParameters;
		cv::Mat currentFrame;
		IplImage* tata;
		cv::Mat userFrame;
		boost::mutex frameMutex;
		bool frameAvailableP;
		bool viewP;

 	public:
		ActiveCameraValues* cameraValues;
		int getNParameters();
		virtual bool view(bool viewP_ = true);
		int setParametersV(int n_, ...);
		virtual void sync();
		virtual cv::Mat& getFrame();
		virtual int setParameters(int n_, char** names_, int* values_)= 0;
		virtual int getParameter(char* name_)= 0;
	};

	class ActiveCameraDir : public ActiveCamera
	{
 	private:
		string basePath;
		string indexPath;
		bool continuosModeP;
		map<string, int> parameterIndex;
		map<int, string> parameterColumnIndex;
		map<string, string> imageIndex;
		map<int, string> timeIndex;
		void loadIndex();
		void refreshFrame();
		string parametersToIndexEntry(int* parameters_);
		string currentImageFileName;
		int currentT;
		int columnT;
 	public:
		ActiveCameraDir(char* path_);
		void setContinuosMode(bool continuosModeP_);
		cv::Mat& getFrame();
		int setParameters(int n_, char** names_, int* values_);
		int getParameter(char* name_);
	};

}
;
#endif 
