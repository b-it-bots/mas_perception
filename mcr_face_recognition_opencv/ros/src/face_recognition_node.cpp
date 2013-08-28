// STL includes

// Third-party includes
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

// Package includes
#include "mcr_face_recognition_opencv/face_recognition.h"

namespace fs = boost::filesystem;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mcr_face_recognition");

	// setup the configuration path where the face database is stored
	fs::path full_path(fs::initial_path<fs::path>());
	full_path = fs::system_complete(fs::path(argv[0]));
	std::string storagePath = full_path.parent_path().parent_path().string();
	ROS_INFO("Configuration path: %s", storagePath.c_str());

	FaceRecognition faceRec(storagePath);
	ROS_INFO("Face Recognition started");
	ros::spin();

	return 0;
}
