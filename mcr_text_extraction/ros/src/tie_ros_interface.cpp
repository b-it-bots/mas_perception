/* Author: Jose Antonio Alvarez Ruiz
 * email: jose.alvarez@smail.inf.h-brs jaaruiz@yahoo.com
 * Hochschule Bonn-Rhein-Sieg    
 */

#include "active_tie/tie_ros_interface.h"

#include <std_msgs/String.h>

using namespace std;

bool TIEROSInterface::getTextCallback(mcr_text_extraction::GetRawText::Request& request, mcr_text_extraction::GetRawText::Response& response)
{
	std_msgs::String str;
	str.data = "hello world!";
	OCRPublisher.publish(str);
	response.text = str;
	return true;
}

TIEROSInterface::TIEROSInterface()
		: nh()
{
	/* Advertise the topics to which we publish and services we provide */
	cout << " Calling me!!!!! " << endl;
	OCRPublisher = nh.advertise < std_msgs::String > ("ocr_results", 5);
	getTextServer = nh.advertiseService("get_text", &TIEROSInterface::getTextCallback, this);
}
