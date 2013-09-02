#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <boost/lexical_cast.hpp>

#include "atie_old/old_tie_ros_interface.h"
#include "atie_old/cc_based_tie.h"
#include "active_tie/ocr_interface.h"

using namespace std;

OldTIEROSInterface::OldTIEROSInterface(atie::OVActiveCamera* camera_)
		: processImageTransport(nh)
{

	processImagePublisher = processImageTransport.advertise("text_extraction_process", 1);
	processImageSmall = cvCreateImage(cvSize(640, 480), 8, 3);
	camera = camera_;
}

bool OldTIEROSInterface::getTextCallback(mcr_text_extraction::GetRawText::Request& request, mcr_text_extraction::GetRawText::Response& response)
{
//  camera->optimizeView();
	cv::Mat& frame = camera->getFrame();
	IplImage legacyImage = frame;
	DetectionResults *dr = processImage(&legacyImage);
	cv::Mat gray;

	/* Create an image that shows the results of the overall process */
	IplImage* processView = cvCreateImage(cvSize(frame.cols * 2, frame.rows * 2), IPL_DEPTH_8U, 3);
	IplImage* conversion = cvCreateImage(cvSize(frame.cols, frame.rows), IPL_DEPTH_8U, 3);

	/* Input frame */
	CvRect ROI = cvRect(0, 0, frame.cols, frame.rows);
	cvSetImageROI(processView, ROI);
	cvCopy(&legacyImage, processView);
	/* Segmentation results */
	ROI.x = frame.cols;

	cvSetImageROI(processView, ROI);
	cvCopy(dr->segmentedImage, processView);

	/* Classification results 1 */
	ROI.x = 0;
	ROI.y = frame.rows;
	cvSetImageROI(processView, ROI);
	cvCvtColor(dr->planes[0], conversion, CV_GRAY2BGR);
	cvCopy(conversion, processView);

	ROI.x = frame.cols;
	cvSetImageROI(processView, ROI);
	cvCvtColor(dr->planes[1], conversion, CV_GRAY2BGR);
	cvCopy(conversion, processView);
	cvResetImageROI(processView);

	string results("");
	cv::imwrite("frame.jpg", frame);
	for (int i = 0; i < dr->nPlanes; i++)
	{
		cvNot(dr->planes[i], dr->planes[i]);
		cv::Mat plane = dr->planes[i];
		results += ocr.recognize(plane) + " ";
		cv::imwrite("plane" + boost::lexical_cast<string>(i) + ".jpg", plane);
	}
	delete dr;
	std_msgs::String str;
	str.data = results;
	OCRPublisher.publish(str);

	cvResize(processView, processImageSmall, CV_INTER_CUBIC);
	cvReleaseImage(&conversion);
	cvReleaseImage(&processView);

	cv_bridge::CvImage out_msg;
	out_msg.image = processImageSmall;

	processImagePublisher.publish(out_msg.toImageMsg());
	response.text.data = results;
	return true;
}
