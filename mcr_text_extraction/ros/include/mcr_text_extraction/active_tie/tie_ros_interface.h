#ifndef __TIEROSINTERFACE__
#define __TIEROSINTERFACE__

#include "ros/ros.h"
#include "mcr_text_extraction/GetRawText.h"

class TIEROSInterface
{
 protected:
	ros::NodeHandle nh;
	ros::ServiceServer getTextServer;
	ros::Publisher OCRPublisher;
 public:
	TIEROSInterface();
	virtual bool getTextCallback(mcr_text_extraction::GetRawText::Request &request, mcr_text_extraction::GetRawText::Response &response);
};

#endif
