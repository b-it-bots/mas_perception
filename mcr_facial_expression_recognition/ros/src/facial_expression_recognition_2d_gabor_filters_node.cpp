#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <math.h>

#include <mcr_perception_msgs/GetFacialExpression.h>
#include <mcr_perception_msgs/FaceList.h>

#include "fer_live_2d_gabor_filters.cpp"

namespace enc = sensor_msgs::image_encodings;

class FacialExpressionNode
{
 protected:
	ros::Subscriber faceList_sub;
	// sensor_msgs::CvBridge bridge;
	ros::Publisher expLabel_pub;
	IplImage cv_input;
	IplImage *cv_output;
	FERLive2DGaborFilters *fer;
	CvPoint lEyePos;
	CvPoint rEyePos;
	std_msgs::String expressionLabel;
	ros::ServiceServer recognizeExpressionServer;
	int i;

 public:

	FacialExpressionNode()
	{
		ros::NodeHandle nh("~");

		i = 0;
		// Advertise label of expression to a topic.
		expLabel_pub = nh.advertise < std_msgs::String > ("facial_expressions", 1);
		// Listen for image messages on a topic and setup callback
		faceList_sub = nh.subscribe("face_list", 1, &FacialExpressionNode::imageCallback, this);

		recognizeExpressionServer = nh.advertiseService("get_expression", &FacialExpressionNode::recognizeExpression, this);

		//Create GUI Window
		//cvNamedWindow("Image window");
	}

	bool recognizeExpression(mcr_perception_msgs::GetFacialExpression::Request& request, mcr_perception_msgs::GetFacialExpression::Response& response)
	{
		response.value = expressionLabel.data;
		return true;
	}

	void imageCallback(const mcr_perception_msgs::FaceList& msg)
	{
		std::cout << "no. of faces: " << msg.num_faces << std::endl;

		if (msg.num_faces < 1)
			return;

		//mcr_perception_msgs::Face face = msg.faces[0];
		lEyePos.x = msg.faces[0].leftEyeCenterX;
		lEyePos.y = msg.faces[0].leftEyeCenterY;

		rEyePos.x = msg.faces[0].rightEyeCenterX;
		rEyePos.y = msg.faces[0].rightEyeCenterY;

		std::cout << "after read eye pose X: " << lEyePos.x << std::endl;

		// Convert ROS Imput Image Message to IplImage

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg.faces[0].image, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		/*
		 try
		 {
		 bridge.fromImage(msg.faces[0].image);
		 cv_input = bridge.toIpl();
		 }
		 catch (cv::Exception error)
		 {
		 ROS_ERROR ("CvBridge Input Error ");
		 return;
		 }
		 */
		try
		{

//	CvFont font;

			//	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

			std::stringstream ss;
			ss << i++;
			// cv::putText(cv_ptr->image, ss.str(), cv::Point(10, 130), CV_FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(255, 255, 255, 0));

			// cv::rectangle(cv_ptr->image, cv::Point(rEyePos.x-2,rEyePos.y-2), cv::Point(rEyePos.x+2,rEyePos.y+2), cv::Scalar(0, 0, 255, 0));
			// cv::rectangle(cv_ptr->image, cv::Point(lEyePos.x-2,lEyePos.y-2), cv::Point(lEyePos.x+2,lEyePos.y+2), cv::Scalar(0, 0, 255, 0));

			//    cv::imshow("Image Window", cv_ptr->image);
			//cv::waitKey(3);

			// std::cout<<"image size " << cv_ptr->image.size().height <<std::endl;

			cv_input = cv_ptr->image;
			// Use Eye Positions and image to classify expression.
			expressionLabel.data = fer->recognizeExpression(&cv_input, lEyePos, rEyePos);

			std::cout << "Expression: " << expressionLabel.data << std::endl;

		}
		catch (cv::Exception error)
		{
			ROS_ERROR("CvBridge Input Error ");
		}

		expLabel_pub.publish(expressionLabel);
	}
};

int main(int argc, char **argv)
{
	// Initialize ROS Node
	ros::init(argc, argv, "fer_live");
	// Instaniate Demo Object
	FacialExpressionNode fer2;
	// Spin ...
	ros::spin();
	// ... until done
	return 0;
}
