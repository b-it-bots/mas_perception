/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include <iostream>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <stdarg.h>

#include "active_tie/sony_dfwvl500_values.h"
#include "active_tie/active_camera_ros.h"

namespace atie
{

	ActiveCameraROS::ActiveCameraROS(char* imageTopic_, char* confPath_, ActiveCameraValues* cv_)
			: nh()
	{
		cameraValues = cv_;
		transcientStateP = false;
		if (confPath_)
		{
			confPath = string(confPath_);
			reconfigureBase = nh.resolveName("parameters");
			cout << "Configuration base " << reconfigureBase << endl;
			dynamicReconfigureClient = ros::ServiceClient(nh.serviceClient < dynamic_reconfigure::Reconfigure > (reconfigureBase + string("/set_parameters")));
		}
		iTransport = new image_transport::ImageTransport(nh);
		iSubscriber = iTransport->subscribe(imageTopic_, 1, &ActiveCameraROS::imageCallback, this);
	}

	int ActiveCameraROS::setParameters(int n_, char** names_, int* values_)
	{
		frameMutex.lock();
		transcientC = 0;
		dynamic_reconfigure::Reconfigure reconfigure;
		dynamic_reconfigure::Config updatedConfig;
		updatedConfig.doubles.resize(n_);
		long maxWait = 0;
		for (int i = 0; i < n_; i++)
		{
			char* name = names_[i];
			int value = values_[i];
			value = cameraValues->translateToSensorValue(name, value);
			string nameStr = string(name);
			updatedConfig.doubles[i].name = nameStr;
			updatedConfig.doubles[i].value = value;

			string key = reconfigureBase + nameStr;
			double currentValue;
			ros::param::getCached(key, currentValue);
			double diff = abs(value - currentValue);
			double rangeDiffAmount = diff / cameraValues->getRangeSize(name);
			double waitTime = cameraValues->getParameterFRDelay(nameStr) * rangeDiffAmount;
			if (waitTime > maxWait)
				maxWait = waitTime;
		}

		reconfigure.request.config = updatedConfig;
		if (!dynamicReconfigureClient.call(reconfigure))
			ROS_ERROR("Unable to set values tilt value!");
		transcientStateP = true;
		transcientC = 2;
		frameAvailableP = false;
		parameterSetSyncTime = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::time_duration(0, 0, 0, maxWait);
		frameMutex.unlock();
		return 0;
	}

	int ActiveCameraROS::getParameter(char* name_)
	{
		/* Not yet implemented!!! */
		return 0;
	}

	void ActiveCameraROS::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		frameMutex.lock();
		if (transcientStateP)
		{
			if (boost::posix_time::microsec_clock::universal_time() > parameterSetSyncTime)
			{
				cv::Mat bridgeMat = cv_bridge::toCvCopy(msg_ptr, "bgr8")->image;
				bridgeMat.copyTo(currentFrame);
				frameAvailableP = true;
				view(viewP);
				transcientStateP = false;
				frameMutex.unlock();

				return;
			}
		}
		else
		{
			cv::Mat bridgeMat = cv_bridge::toCvCopy(msg_ptr, "bgr8")->image;
			bridgeMat.copyTo(currentFrame);
			frameAvailableP = true;
			transcientStateP = false;
			view(viewP);

		}
		frameMutex.unlock();
	}

	cv::Mat& ActiveCameraROS::getFrame()
	{
		frameMutex.lock();
		while (!frameAvailableP || transcientStateP)
		{
			frameMutex.unlock();
			ros::spinOnce();
			usleep(100);
			frameMutex.lock();
		}
		currentFrame.copyTo(userFrame);
		/* This frame was fetched already, we prevent if from
		 being returned again. */
		frameAvailableP = false;
		frameMutex.unlock();
		return userFrame;
	}
}
;
