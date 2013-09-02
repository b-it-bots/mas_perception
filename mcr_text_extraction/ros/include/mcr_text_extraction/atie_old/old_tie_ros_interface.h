#ifndef __OLD_TIE_ROS_INTERFACE__
#define __OLD_TIE_ROS_INTERFACE__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#include "active_tie/tie_ros_interface.h"
#include "active_tie/ov_active_camera.h"
#include "active_tie/ocr_interface.h"
#include "active_tie/open_cv_includes.h"

class OldTIEROSInterface : public TIEROSInterface
{
 protected:
	atie::OVActiveCamera* camera;
	atie::AbbyOCR ocr;
	image_transport::ImageTransport processImageTransport;
	image_transport::Publisher processImagePublisher;
	IplImage* processImageSmall;
 public:
	OldTIEROSInterface(atie::OVActiveCamera* camera_);
	bool getTextCallback(mcr_text_extraction::GetRawText::Request& request, mcr_text_extraction::GetRawText::Response& response);
};

#endif
