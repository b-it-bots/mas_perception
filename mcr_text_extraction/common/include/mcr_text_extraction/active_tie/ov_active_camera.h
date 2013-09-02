/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#ifndef __OV_ACTIVE_CAMERA__
#define __OV_ACTIVE_CAMERA__

#include "active_tie/sharpness_measures.h"
#include "active_tie/active_camera.h"
#include "active_tie/acquisition_parameters_optimization.h"
#include "active_tie/open_cv_includes.h"

namespace atie
{
	class OVActiveCamera : public ActiveCamera
	{
 	private:
		ActiveCamera* camera;
		SearchCriterion* focusCriterion;
		SearchCriterion* irisCriterion;
		SearchCriterion* brightnessCriterion;
		int nFramesToAverage;
		TemporalAverage temporalAverage;
		FibonacciSearch fibonacciSearch;
		bool optimizeViewP;

 	public:
		OVActiveCamera(ActiveCamera* camera_);
		int setNFramesToAverage(int n_);
		int getNFramesToAverage();
		void optimizeView();
		bool view(bool viewP_ = true);
		int setParameters(int n_, char** names_, int* values_);
		bool setOptimizeView(bool optimizeViewP_ = true);
		void sync();
		int getParameter(char* name_);
		cv::Mat& getFrame();
	};
}

#endif
