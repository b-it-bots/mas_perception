// STL includes

// Third-party includes

// Package includes
#include "mcr_face_recognition_opencv/face_recognition.h"

FaceRecognition::FaceRecognition(const std::string &storagePath)
{
    ros::NodeHandle n("~");
    _storagePath = storagePath;
    _learnNewFace = false;
    _doFaceRecognition = false;
    _doSearchForFaces = false;
    _lastRecognizedFaceName = "";

    _libFace = new libface::LibFace(libface::ALL, _storagePath);
    _libFace->setDetectionAccuracy(1.0);
    _libFace->setDetectionSpecificity(1.0);

    // Kinect
    // _cameraImageSubscriber = n.subscribe("/camera/rgb/image_color", 1, &FaceRecognition::localizeFaces, this);
    _cameraImageSubscriber = n.subscribe("/cam3d/rgb/image_color", 1, &FaceRecognition::localizeFaces, this);

    // Pike stereo camera (INVERT IMAGE !!!!)
    // _cameraImageSubscriber = n.subscribe("/stereo/left/image_mono", 1, &FaceRecognition::localizeFaces, this);

    // Fake camera
    //_cameraImageSubscriber = n.subscribe("/camera/image", 1, &FaceRecognition::localizeFaces, this);

    // faceListPublisher = n.advertise<mcr_perception_msgs::FaceList>("face_list", 1000);
    _personInFrontPublisher = n.advertise < mcr_perception_msgs::FaceList > ("is_person_in_front", 1000);
    _recognizedFacesPublisher = n.advertise < mcr_perception_msgs::FaceList > ("recognized_faces", 1000);
    _learnFaceService = n.advertiseService("learn_face", &FaceRecognition::learnFace, this);
    _loadFaceService = n.advertiseService("load_person_face", &FaceRecognition::loadFace, this);
    _storeFaceService = n.advertiseService("store_person_face", &FaceRecognition::storeFace, this);
    _startLocalizationService = n.advertiseService("face_localization/start", &FaceRecognition::startFaceLocalization, this);
    _stopLocalizationService = n.advertiseService("face_localization/stop", &FaceRecognition::stopFaceLocalization, this);
    _startRecognitionService = n.advertiseService("face_recognition/start", &FaceRecognition::startFaceRecognition, this);
    _stopRecognitionService = n.advertiseService("face_recognition/stop", &FaceRecognition::stopFaceRecognition, this);

    // _getLastFaceNameService = n.advertiseService("get_last_face_name", &FaceRecognition::getLastFaceName, this);
}

FaceRecognition::~FaceRecognition()
{
    delete _libFace;
}

bool FaceRecognition::startFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    this->_doSearchForFaces = true;
    ROS_INFO("Start localizing face");
    return true;
}

bool FaceRecognition::stopFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    this->_doSearchForFaces = false;
    ROS_INFO("Stop localizing face");
    return true;
}

bool FaceRecognition::startFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    this->_doFaceRecognition = true;
    this->_doSearchForFaces = true;
    ROS_INFO("Start recognizing face");
    return true;
}

bool FaceRecognition::stopFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    this->_doFaceRecognition = false;
    ROS_INFO("Stop recognizing face");
    return true;
}

bool FaceRecognition::getLastFaceName(mcr_perception_msgs::GetFaceName::Request& request, mcr_perception_msgs::GetFaceName::Response& name)
{
    name.value = this->_lastRecognizedFaceName;
    ROS_INFO("get Last Face Name");
    return true;
}

bool FaceRecognition::learnFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success)
{
    ROS_INFO("Face will be learned");

    _toLearnedPersonName = name.name;
    int successfull = 0;
    _learnNewFace = true;
    for (int i = 0; i < 10; i++)
    {
        ros::spinOnce();
        if (!_learnNewFace)
        {
            successfull = 1;
            break;
        }
    }
    success.success = successfull;
    return true;
}

bool FaceRecognition::storeFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success)
{
    ROS_INFO("All learned faces will be stored to disk");

    success.success = (_libFace->saveConfig(_storagePath) == 0);

    return true;
}

bool FaceRecognition::loadFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success)
{
    ROS_INFO("All stored face will be loaded from disk");

    success.success = (_libFace->loadConfig(_storagePath) == 0);

    return true;
}

IplImage *FaceRecognition::convertRosImageToIplImage(const sensor_msgs::Image::ConstPtr &msg)
{
    IplImage *img = 0;
    std::string cv_encoding = "passthrough";

    try
    {
        _cv_img = cv_bridge::toCvCopy(msg, cv_encoding);
        _ipl_img = _cv_img->image;

        img = &_ipl_img;
    }
    catch (cv_bridge::Exception &error)
    {
        ROS_ERROR("Failed to convert sensor_msgs::Image to img_t");
        return 0;
    }

    return img;
}

void FaceRecognition::localizeFaces(const sensor_msgs::Image::ConstPtr& camImageROS)
{
    // only do the processing if requested
    if ((!_doSearchForFaces) && (!_learnNewFace))
    {
        return;
    }

    // convert to gray-scale image
    IplImage *iplImage = convertRosImageToIplImage(camImageROS);
    IplImage *im_gray = cvCreateImage(cvSize(iplImage->width, iplImage->height), IPL_DEPTH_8U, 1);
    cvCvtColor(iplImage, im_gray, CV_RGB2GRAY);
    iplImage = im_gray;

    if (iplImage == NULL)
    {
        ROS_ERROR("could not convert image");
        return;
    }

    // do face detection
    std::vector<libface::Face> detectedFaces = _libFace->detectFaces(iplImage->imageData, iplImage->width, iplImage->height, iplImage->widthStep,
            iplImage->depth, iplImage->nChannels);

    ROS_INFO("%ld Faces found", detectedFaces.size());

    // Publish the number of persons found in the image
    mcr_perception_msgs::FaceList personInFrontList;
    personInFrontList.num_faces = detectedFaces.size();
    _personInFrontPublisher.publish(personInFrontList);

    // recognize faces
    if (_doFaceRecognition)
    {
        // stop if no faces have been found
        if (detectedFaces.empty())
            return;

        std::vector<std::pair<int, double> > recognizedFaces;
        recognizedFaces = _libFace->recognise(&detectedFaces);

        mcr_perception_msgs::FaceList listOfRecognizedFaces;

        if (!recognizedFaces.empty())
        {
            for (unsigned int i = 0; i < recognizedFaces.size(); i++)
            {
                mcr_perception_msgs::Face newFace;

                std::string identifiedPersonName = _idToName[recognizedFaces[i].first];

                newFace.name = identifiedPersonName;
                newFace.ID = recognizedFaces[i].first;
                listOfRecognizedFaces.faces.push_back(newFace);
                listOfRecognizedFaces.num_faces++;

                _lastRecognizedFaceName = identifiedPersonName;
                ROS_INFO("%s has been identified", identifiedPersonName.c_str());
            }
        }
        else
        {
            mcr_perception_msgs::Face newFace;
            newFace.name = "unknown";
            listOfRecognizedFaces.faces.push_back(newFace);
            listOfRecognizedFaces.num_faces++;

            _lastRecognizedFaceName = "unknown";
            ROS_INFO("Unknown face");
        }

        // if we recognized a person, then publish
        if (!listOfRecognizedFaces.faces.empty())
        {
            _recognizedFacesPublisher.publish(listOfRecognizedFaces);
        }
    }

    // learn a new face
    if (_learnNewFace)
    {
        // we can only learn one face at a time
        if ((detectedFaces.size() != 1))
            return;

        addIdRecordsToGallery(_toLearnedPersonName, detectedFaces);

        // don't learn again in next cycle
        _learnNewFace = false;

        ROS_INFO("Face of %s has been learned", _toLearnedPersonName.c_str());
    }
}

void FaceRecognition::addIdRecordsToGallery(const std::string &personName, std::vector<libface::Face> &detectedFaces)
{
    // check if the name has already an associated face id
    if (_nameToId.find(personName) != _nameToId.end())
    {
        ROS_WARN("%s was added previously to the gallery...", personName.c_str());
        return;
    }

    // only one face may be added at a time
    if (detectedFaces.size() != 1)
    {
        return;
    }

    // learn the face
    std::vector<int> assignedIds = _libFace->update(&detectedFaces);

    // we should only get one id back, because there is only one face to be
    // learned
    if (assignedIds.size() != 1)
    {
        ROS_ERROR("The number of returned ids was %d, but only 1 is allowed", static_cast<int>(assignedIds.size()));
        return;
    }

    // associate the id with the name and vice versa
    _idToName[assignedIds[0]] = personName;
    _nameToId[personName] = assignedIds[0];
}
