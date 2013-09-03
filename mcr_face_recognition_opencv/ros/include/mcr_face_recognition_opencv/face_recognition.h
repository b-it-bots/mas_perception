#ifndef FACERECOGNITIONNODE_HPP_
#define FACERECOGNITIONNODE_HPP_

// STL includes
#include <string>
#include <vector>

// Third-party includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <mcr_perception_msgs/Face.h>
#include <std_srvs/Empty.h>
#include <mcr_perception_msgs/FaceList.h>
#include <mcr_perception_msgs/SetFaceName.h>
#include <mcr_perception_msgs/GetFaceName.h>
#include <cv_bridge/cv_bridge.h>

// Package includes
#include <libface/LibFace.h>
#include <libface/Face.h>

class FaceRecognition
{
 public:
	FaceRecognition(const std::string &storagePath);
	~FaceRecognition();

 private:
	bool learnFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success);
	bool storeFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success);
	bool loadFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success);
	bool getLastFaceName(mcr_perception_msgs::GetFaceName::Request& request, mcr_perception_msgs::GetFaceName::Response& name);
	IplImage *convertRosImageToIplImage(const sensor_msgs::Image::ConstPtr &msg);
	void localizeFaces(const sensor_msgs::Image::ConstPtr& camImageROS);
	void addIdRecordsToGallery(const std::string &personName, std::vector<libface::Face> &detectedFaces);
	bool startFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool stopFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool startFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool stopFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

 private:
	ros::Subscriber _cameraImageSubscriber;
	ros::Publisher _faceListPublisher;
	ros::Publisher _personInFrontPublisher;
	ros::Publisher _recognizedFacesPublisher;
	ros::ServiceServer _learnFaceService;
	ros::ServiceServer _loadFaceService;
	ros::ServiceServer _storeFaceService;
	ros::ServiceServer _startLocalizationService;
	ros::ServiceServer _stopLocalizationService;
	ros::ServiceServer _startRecognitionService;
	ros::ServiceServer _stopRecognitionService;
	ros::ServiceServer _isPersonInFrontService;
	ros::ServiceServer _getLastFaceNameService;

	std::string _storagePath;

	bool _doFaceRecognition;
	bool _doSearchForFaces;
	bool _learnNewFace;
	std::string _toLearnedPersonName;
	std::string _toStoredPersonName;

	std::string _lastRecognizedFaceName;

	cv_bridge::CvImagePtr _cv_img;
	IplImage _ipl_img;

	std::map<int, std::string> _idToName;
	std::map<std::string, int> _nameToId;
	libface::LibFace *_libFace;
};

#endif
