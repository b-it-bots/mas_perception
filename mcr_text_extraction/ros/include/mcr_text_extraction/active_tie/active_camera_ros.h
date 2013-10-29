#ifndef __ACTIVE_CAMERA_ROS__
#define __ACTIVE_CAMERA_ROS__

#include "active_tie/active_camera.h"
#include "active_tie/active_camera_values.h"
#include "active_tie/open_cv_includes.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/* Includes for dynamic reconfigure */
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace atie
{

	class ActiveCameraROS : public ActiveCamera
	{
 	private:
		ros::NodeHandle nh;
		ros::ServiceClient dynamicReconfigureClient;
		string confPath;
		bool transcientStateP;
		cv::Mat lastFrame;
		string reconfigureBase;
		image_transport::ImageTransport* iTransport;
		image_transport::Subscriber iSubscriber;
		/* This allows us to wake until the setups have been acknowledged
		 by the hardware */
		boost::posix_time::ptime parameterSetSyncTime;
		int transcientC;

 	public:
		ActiveCameraROS(const char* imageTopic_, char* confTopic_, ActiveCameraValues* cv_ = NULL);
		void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
		int setParameters(int n_, char** names_, int* values_);
		cv::Mat& getFrame();
		int getParameter(char* name_);
	};
}
;

#endif
