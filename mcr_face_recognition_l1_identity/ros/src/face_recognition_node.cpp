#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>

#include <mcr_perception_msgs/Face.h>
#include <mcr_perception_msgs/FaceList.h>
#include <mcr_perception_msgs/SetFaceName.h>
#include <mcr_common_msgs/ReturnBool.h>
#include <mcr_common_msgs/ReturnString.h>

#include <IdentityTOOLS_SDK/visgException.h>
#include <IdentityTOOLS_SDK/Id.h>
#include <IdentityTOOLS_SDK/Image.h>
#include <IdentityTOOLS_SDK/DescriptorList.h>
#include <IdentityTOOLS_SDK/FaceLocator.h>
#include <IdentityTOOLS_SDK/FaceDescriptor.h>
#include <IdentityTOOLS_SDK/FaceInputDataQuality.h>
#include <IdentityTOOLS_SDK/Enrollment.h>
#include <IdentityTOOLS_SDK/IdRecordKeyList.h>
#include <IdentityTOOLS_SDK/IdRecordKey.h>
#include <IdentityTOOLS_SDK/IdRecord.h>
#include <IdentityTOOLS_SDK/TemplateKeyList.h>
#include <IdentityTOOLS_SDK/TemplateKey.h>
#include <IdentityTOOLS_SDK/ResultList.h>
#include <IdentityTOOLS_SDK/Result.h>
#include <IdentityTOOLS_SDK/Recognition.h>
#include <IdentityTOOLS_SDK/Landmark.h>
#include <IdentityTOOLS_SDK/LandmarkList.h>
#include <IdentityTOOLS_SDK/FacialRecordData.h>
#include <IdentityTOOLS_SDK/FaceImageGeneratorParameters.h>
#include <IdentityTOOLS_SDK/FaceImageGenerator.h>
#include <IdentityTOOLS_SDK/FeaturePoint.h>
#include <IdentityTOOLS_SDK/FaceImageQualityBitString.h>
#include <IdentityTOOLS_SDK/Position.h>
#include <ImageDevice/ImageDisplayWin32.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <sstream>

const std::string RGB8 = "rgb8";
const std::string RGBA8 = "rgba8";
const std::string BGR8 = "bgr8";
const std::string BGRA8 = "bgra8";
const std::string MONO8 = "mono8";
const std::string MONO16 = "mono16";

#define MININPUTQUALITY_FACEREC 80; //default 85
#define MININPUTQUALITY_FACELOCATION 40;

const unsigned int NoOfFacesToFind = 1;
float minInputQuality = MININPUTQUALITY_FACEREC
;
const float identificationThreshold = 70;  //default 85
const float personInFrontQuality = 10;
const unsigned int maxMatchCount = 5;

class FaceRecognition
{
 public:
	FaceRecognition();
	~FaceRecognition();

 private:
	ros::NodeHandle *n;
	ros::Subscriber cameraImageSubscriber;
	ros::Publisher faceListPublisher;
	ros::Publisher personInFrontPublisher;
	ros::Publisher recognizedFacesPublisher;
	ros::ServiceServer learnFaceService;
	ros::ServiceServer loadFaceService;
	ros::ServiceServer storeFaceService;
	ros::ServiceServer doLocalizationService;
	ros::ServiceServer doRecogniationService;
	ros::ServiceServer isPersonInFrontService;
	ros::ServiceServer getLastFaceNameService;
	bool learnFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success);
	bool storeFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success);
	bool loadFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success);
	bool getLastFaceName(mcr_common_msgs::ReturnString::Request& request, mcr_common_msgs::ReturnString::Response& name);
	void localizeFaces(const sensor_msgs::Image::ConstPtr& camImageROS);
	visg::idt::Image* convertROSImageToL1Image(const sensor_msgs::Image& ROSImage);
	void convertL1ImageToROSImage(sensor_msgs::Image& ROSImage, const visg::idt::Image& L1Image);
	void Enrollment(visg::idt::DescriptorList& foundFacesDescriptorList, visg::idt::IdRecordList& foundFacesIdRedordList);
	void Identification(visg::idt::IdRecordList& foundFacesIdRedordList, visg::idt::ResultList& foundPersons);
	void addIdRecordsToGallery(const std::string& PersonName, visg::idt::IdRecord& IdRecord);
	void storeIdRecordInFile(const std::string& PersonName, const visg::idt::IdRecord& IdRecord);
	void readIdRecordFromFileAndAddToGallery(const std::string& PersonName);
	bool startFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool stopFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool startFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool stopFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	std::string storagePath;

	//  sensor_msgs::Image msg;
	bool doFaceRecognition;
	bool doSearchForFaces;
	bool learnNewFace;
	bool storeFaceToFile;
	std::string ToLernedPersonName;
	std::string ToStoredPersonName;
	unsigned long templatesCounter;
	unsigned long imageCounter;

	std::map<unsigned long, std::string> nameToIdRecordMap;
	visg::idt::FaceLocator* faceLocator;
	visg::idt::Enrollment* enrollment;
	visg::idt::Recognition* recognition;

	unsigned char* L1ImageData;

	visg::idt::Image* L1Image;
	unsigned long LastImageSize;
	mcr_perception_msgs::FaceList FaceListHeader;

	bool invertInputImage;
	bool isPersonInFrontValue;
	std::string lastRecognizedFaceName;

};

FaceRecognition::FaceRecognition()
{
	ros::NodeHandle n("~");
	this->n = &n;

	L1Image = NULL;
	L1ImageData = NULL;
	LastImageSize = 0;
	storagePath = "C:\\RoboCupAtHome\\mcr_perception\\mcr_face_recognition_l1_identity\\common\\face_database\\";
	learnNewFace = false;
	storeFaceToFile = false;
	doFaceRecognition = false;
	doSearchForFaces = false;
	isPersonInFrontValue = false;
	lastRecognizedFaceName = "";
	templatesCounter = 0;
	imageCounter = 0;
	faceLocator = new visg::idt::FaceLocator;
	enrollment = new visg::idt::Enrollment;
	recognition = new visg::idt::Recognition;
	faceLocator->setFaceLocationMode(visg::idt::FaceLocator::SmallFace);

//	cameraImageSubscriber = n.subscribe("/stereo/left/image_mono", 1, &FaceRecognition::localizeFaces, this);

	invertInputImage = true;
	// /camera/rgb/image_color // kinect
	// /camera/image //fake camera
	// /stereo/left/image_raw // pike stereo Cam !!!! INVERT IMAGE
	//pr2_controllers_msgs/JointTrajectoryControllerState message

//	faceListPublisher = n->advertise<mcr_perception_msgs::FaceList>("face_list", 1000);
	personInFrontPublisher = n->advertise < mcr_perception_msgs::FaceList > ("is_person_in_front", 1);
	recognizedFacesPublisher = n->advertise < mcr_perception_msgs::FaceList > ("recognized_faces", 1);
	learnFaceService = n->advertiseService("learn_face", &FaceRecognition::learnFace, this);
	loadFaceService = n->advertiseService("load_person_face", &FaceRecognition::loadFace, this);
	storeFaceService = n->advertiseService("store_person_face", &FaceRecognition::storeFace, this);
	doLocalizationService = n->advertiseService("do_face_localization", &FaceRecognition::activateFaceLocalization, this);
	doRecogniationService = n->advertiseService("do_face_recognition", &FaceRecognition::activateFaceRecognition, this);
//	getLastFaceNameService = n->advertiseService("get_last_face_name", &FaceRecognition::getLastFaceName, this);

	return;
}

FaceRecognition::~FaceRecognition()
{
	delete faceLocator;
	delete enrollment;
	delete recognition;
}

bool FaceRecognition::startFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	this->doSearchForFaces = true;

	cameraImageSubscriber = n->subscribe("/stereo/left/image_mono", 1, &FaceRecognition::localizeFaces, this);
	minInputQuality = MININPUTQUALITY_FACELOCATION;
	ROS_INFO("Activate Face Localization");

	return true;
}

bool FaceRecognition::stopFaceLocalization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	this->doSearchForFaces = false;

	cameraImageSubscriber.shutdown();
	ROS_INFO("Deactivate Face Localization");

	return true;
}

bool FaceRecognition::startFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	this->doFaceRecognition = true;

	this->startFaceLocalization(request, response);

	minInputQuality = MININPUTQUALITY_FACEREC;
	ROS_INFO("Activate Face Recognition");
}

bool FaceRecognition::stopFaceRecognition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	this->doFaceRecognition = false;
	this->stopFaceLocalization(request, response);

	ROS_INFO("Deactivate Face Recognition");
	return true;
}

bool FaceRecognition::getLastFaceName(mcr_common_msgs::ReturnString::Request& request, mcr_common_msgs::ReturnString::Response& name)
{
	name.value = this->lastRecognizedFaceName;
	ROS_INFO("get Last Face Name");
	return true;
}

bool FaceRecognition::learnFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success)
{

	ROS_INFO("Face will be learned");

	ToLernedPersonName = name.name;
	learnNewFace = true;
	while (learnNewFace)
	{
		ros::spinOnce();
	}
	success.success = 1;
	return true;
}

bool FaceRecognition::storeFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success)
{

	ROS_INFO("Face will be stored to disk");

	int successfull = 0;
	ToStoredPersonName = name.name;
	storeFaceToFile = true;
	for (int i = 0; i < 10; i++)
	{
		ros::spinOnce();
		if (!storeFaceToFile)
		{
			successfull = 1;
			break;
		}
	}
	success.success = successfull;
	return true;
}

bool FaceRecognition::loadFace(mcr_perception_msgs::SetFaceName::Request& name, mcr_perception_msgs::SetFaceName::Response& success)
{

	ROS_INFO("Face will be loaded from disk");
	try
	{
		success.success = 1;
		this->readIdRecordFromFileAndAddToGallery(name.name);
	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
		success.success = 0;
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
		success.success = 0;
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
		success.success = 0;
	}

	return true;
}

//**********************************		convertROSImageToL1Image()		**********************************************
//convert from the ros image format to the L1Identity format
visg::idt::Image* FaceRecognition::convertROSImageToL1Image(const sensor_msgs::Image& ROSImage)
{
	try
	{

		unsigned long ImageSize = ROSImage.step * ROSImage.height;

		if (LastImageSize != ImageSize && L1ImageData != NULL)
		{
			delete L1ImageData;
			L1ImageData = NULL;
		}

		if (L1ImageData == NULL)
		{
			L1ImageData = new unsigned char[ImageSize];
		}

		if (ImageSize != ROSImage.data.size())
		{
			throw std::exception("invalid image format");
		}

		//	FaceListHeader.header = ROSImage.header;
		LastImageSize = ImageSize;

		for (unsigned int i = 0; i < ROSImage.data.size(); i++)
		{
			if (invertInputImage)
			{
				L1ImageData[i] = ROSImage.data[(ROSImage.data.size() - 1) - i];
			}
			else
			{
				L1ImageData[i] = ROSImage.data[i];
			}
		}

		L1Image = new visg::idt::Image(ROSImage.width,                               // width
		        ROSImage.height,                               // height
		        ImageSize / (ROSImage.width * ROSImage.height),                                // depth, bits per pixel
		        visg::idt::Image::ImageColorSpaceGray8C,     // color space
		        L1ImageData                           // raw Data
		        );

		return L1Image;

	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}
}

//**********************************		convertL1ImageToROSImage()		**********************************************
//convert from L1Identity image to ROS format
void FaceRecognition::convertL1ImageToROSImage(sensor_msgs::Image& ROSImage, const visg::idt::Image& L1Image)
{
	try
	{
		std::vector<uint8_t> ROSImageData;

		for (unsigned int i = 0; i < L1Image.dataSize(); i++)
		{
			ROSImageData.push_back((L1Image.data())[i]);
		}

		ROSImage.data = ROSImageData;
		ROSImage.encoding = MONO8;
		ROSImage.height = L1Image.height();
		ROSImage.width = L1Image.width();
		ROSImage.step = L1Image.dataSize() / L1Image.height();

		return;

	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}
}

//**********************************		localizeFaces()		**********************************************
void FaceRecognition::localizeFaces(const sensor_msgs::Image::ConstPtr& camImageROS)
{
	try
	{

		//ROS_INFO("got image");
//		ros::Duration framedelaytime = ros::Time::now() - camImageROS->header.stamp; //sec
//		std::cout << "image time diff " <<  framedelaytime << std::endl;
		//	printf("image time diff %d sec %d nsec \n", framedelaytime.sec, framedelaytime.nsec);

		if (doSearchForFaces || learnNewFace)
		{
			ROS_INFO("do face localization");
			std::cout << "minInputQuality: " << minInputQuality << std::endl;

			visg::idt::Image* imageL = convertROSImageToL1Image(*camImageROS);

			if (imageL == NULL)
			{
				ROS_ERROR("could not convert image");
				return;
			}

			//imageL->write("test.bmp",visg::idt::Image::ImageDataTypeBmpC);
			visg::idt::DescriptorList foundFacesDescriptorList;

			//locate faces in image
			faceLocator->locateFaces(*imageL, foundFacesDescriptorList, NoOfFacesToFind);

			visg::idt::FaceDescriptor* foundFaceDescriptor;
			std::vector<visg::idt::FaceDescriptor> goodQualityFaces;
			mcr_perception_msgs::FaceList personInFrontList;

			for (unsigned int i = 0; i < foundFacesDescriptorList.size(); i++)
			{

				foundFaceDescriptor = dynamic_cast<visg::idt::FaceDescriptor*>(foundFacesDescriptorList[i]);

				if (foundFaceDescriptor == 0 || !foundFaceDescriptor->isValid())
				{
					ROS_INFO("could not create a valid FaceDescriptor");
					break;
				}
				ROS_INFO("%f face quality", foundFaceDescriptor->faceInputDataQuality().quality());

				if (foundFaceDescriptor->faceInputDataQuality().quality() >= minInputQuality)
				{
					goodQualityFaces.push_back(*foundFaceDescriptor);
				}
				if (foundFaceDescriptor->faceInputDataQuality().quality() >= personInFrontQuality)
				{
					personInFrontList.num_faces++;
				}

			}
			personInFrontPublisher.publish(personInFrontList);
			mcr_perception_msgs::FaceList listOfFaces;
			mcr_perception_msgs::Face newFace;
			visg::idt::LandmarkList listOfLandmarks;

			ROS_INFO("%ld Faces found", goodQualityFaces.size());
			//put all good faces in a ROS FaceList
			for (unsigned int i = 0; i < goodQualityFaces.size(); i++)
			{
				listOfLandmarks = goodQualityFaces[i].landmarkList();

				//FaceRecognition::convertL1ImageToROSImage(newFace.image, *imageL );
				//		newFace.image = *camImageROS;

				newFace.name = "unkown";

				//get left and right eye center from the landmarks
				for (unsigned int i = 0; i < listOfLandmarks.size(); i++)
				{
					visg::idt::Landmark lmL = listOfLandmarks[i];
					if (lmL.isValid())
					{
						if (lmL.type() == visg::idt::Landmark::leftEyeCenterC)
						{
							newFace.leftEyeCenterX = lmL.position().x();
							newFace.leftEyeCenterY = lmL.position().y();

						}
						if (lmL.type() == visg::idt::Landmark::rightEyeCenterC)
						{
							newFace.rightEyeCenterX = lmL.position().x();
							newFace.rightEyeCenterY = lmL.position().y();
						}
					}
				}
				listOfFaces.faces.push_back(newFace);
				//		listOfFaces.header = FaceListHeader.header;
				listOfFaces.num_faces++;

			}

			//	if(listOfFaces.faces.size() > 0){
			//		faceListPublisher.publish(listOfFaces);
			//	}

			//delete image it is not needed any more
			delete imageL;

			lastRecognizedFaceName = "";

			if ((doFaceRecognition || learnNewFace) && goodQualityFaces.size() == 1)
			{
				ROS_INFO("do face recognition");
				visg::idt::IdRecordList foundFacesIdRedordList;
				visg::idt::ResultList foundPersons;

				Enrollment(foundFacesDescriptorList, foundFacesIdRedordList);

				if (foundFacesIdRedordList.size() == 1 && learnNewFace == true)
				{
					this->addIdRecordsToGallery(ToLernedPersonName, foundFacesIdRedordList[0]);
					learnNewFace = false;
					ROS_INFO("Face of %s has been learned", ToLernedPersonName.c_str());
				}

				if (foundFacesIdRedordList.size() > 0 && learnNewFace == false)
				{
					mcr_perception_msgs::FaceList listOfRecognizedFaces;

					Identification(foundFacesIdRedordList, foundPersons);
					if (foundPersons.size() > 0)
					{
						for (unsigned int i = 0; i < foundPersons.size(); i++)
						{
							mcr_perception_msgs::Face newFace;

							std::string identifiedPersonName = nameToIdRecordMap[foundPersons[i].personId().low()];
							ROS_INFO("%s has been identified", identifiedPersonName.c_str());
							newFace.name = identifiedPersonName;
							lastRecognizedFaceName = identifiedPersonName;
							newFace.ID = foundPersons[i].personId().low();
							listOfRecognizedFaces.faces.push_back(newFace);
							//			listOfRecognizedFaces.header = FaceListHeader.header;
							listOfRecognizedFaces.num_faces++;
						}
					}
					else
					{
						ROS_INFO("Unknown face");
						mcr_perception_msgs::Face newFace;
						newFace.name = "unknown";
						lastRecognizedFaceName = "unknown";
						listOfRecognizedFaces.faces.push_back(newFace);
						//			listOfRecognizedFaces.header = FaceListHeader.header;
						listOfRecognizedFaces.num_faces++;
					}
					if (listOfRecognizedFaces.faces.size() > 0)
					{
						recognizedFacesPublisher.publish(listOfRecognizedFaces);
					}
				}

			}
			if (this->storeFaceToFile)
			{

				std::map<unsigned long, std::string>::const_iterator iter;
				for (iter = nameToIdRecordMap.begin(); iter != nameToIdRecordMap.end(); ++iter)
				{
					if (iter->second == ToStoredPersonName)
					{
						visg::idt::Id IDToStore;
						IDToStore.setHigh(0);
						IDToStore.setLow(iter->first);
						visg::idt::IdRecordList IDList = recognition->idRecords(IDToStore);
						this->storeIdRecordInFile(iter->second, IDList[0]);
						storeFaceToFile = false;
						break;
					}
				}
			}

		}

	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}

}

//**********************************		Enrollment()		**********************************************
void FaceRecognition::Enrollment(visg::idt::DescriptorList& foundFacesDescriptorList, visg::idt::IdRecordList& foundFacesIdRedordList)
{
	try
	{
		visg::idt::Id templateId; 					//Template Id and Person Id
		visg::idt::IdRecord idRecordTemp;

		if (foundFacesIdRedordList.size() != 0)
		{
			ROS_WARN("lost old ID records");
			foundFacesIdRedordList.clear();
		}

		if (foundFacesDescriptorList.size() == 0)
		{
			ROS_WARN("No faces found");
			return;
		}

		templateId.setHigh(0);

		for (unsigned int i = 0; i < foundFacesDescriptorList.size(); i++)
		{

			templatesCounter++;
			templateId.setLow(templatesCounter);
			foundFacesDescriptorList[i]->setInstance(visg::idt::IdRecord::frontalC);  //Biometric instances face frontal
			foundFacesDescriptorList[i]->setTemplateId(templateId);

			enrollment->createTemplate(foundFacesDescriptorList[i],  // input data
			        idRecordTemp,							                              // output data
			        visg::idt::IdRecord::TemplatePurposeAllC                // Template for all purposes (Probe and Gallery)
			        );

			foundFacesIdRedordList.push_back(idRecordTemp);
		}
	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}

}

//**********************************		Identification()		**********************************************
void FaceRecognition::Identification(visg::idt::IdRecordList& foundFacesIdRedordList, visg::idt::ResultList& foundPersons)
{

	try
	{
		recognition->setIdentificationThreshold(identificationThreshold);
		// Set the max match count to reduce the identification result lists
		recognition->setMaxMatchCount(maxMatchCount);
		visg::idt::ResultList foundPersonForOneFace;
		foundPersons.clear();

		for (unsigned int i = 0; i < foundFacesIdRedordList.size(); i++)
		{
			// Identify IdRecord vs all Persons (Person Matching):
			// Matches the given IdRecord against all IdRecords in the gallery, returning a 
			// result list with a result for each person, i.e. fuses multiple IdRecord 
			// matching results for the same Person-Id.
			if (recognition->numberOfIdRecords() > 0)
			{
				foundPersonForOneFace = recognition->identifyPerson(foundFacesIdRedordList[i], visg::idt::IdRecord::faceC, visg::idt::IdRecord::frontalC);

				//copy best match for every face
				if (foundPersonForOneFace.size() == 1)
				{
					if (foundPersonForOneFace[0].isValid())
					{
						foundPersons.push_back(foundPersonForOneFace[0]);

						foundPersonForOneFace.clear();
					}
				}
			}
		}
	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}
}

void FaceRecognition::addIdRecordsToGallery(const std::string& PersonName, visg::idt::IdRecord& IdRecord)
{
	try
	{
		if (!recognition->hasIdRecord(IdRecord.personId()))
		{
			templatesCounter++;
			visg::idt::Id tempID = IdRecord.personId();
			tempID.setLow(templatesCounter);
			IdRecord.setPersonId(tempID);
			recognition->addIdRecord(IdRecord);
			nameToIdRecordMap[IdRecord.personId().low()] = PersonName;

		}
		else
		{
			ROS_WARN("%s was added previously in the Gallery...", PersonName);
		}
	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}
}

void FaceRecognition::storeIdRecordInFile(const std::string& PersonName, const visg::idt::IdRecord& IdRecord)
{
	try
	{
		IdRecord.write((storagePath + PersonName).c_str());

	}
	catch (visg::idt::Exception e)
	{
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}
}

void FaceRecognition::readIdRecordFromFileAndAddToGallery(const std::string& PersonName)
{
	visg::idt::IdRecord IdRecord;
	IdRecord.read((storagePath + PersonName).c_str());
	addIdRecordsToGallery(PersonName, IdRecord);
}

int main(int argc, char **argv)
{

	try
	{
		SetPriorityClass(GetCurrentProcess(), IDLE_PRIORITY_CLASS);

		//set master URI
		//	ros::master::setURI("http://cob3-1-pc1:11311/");
		//set ros ip
		//	ros::network::setHost("cob3-1-pc3");

		ros::init(argc, argv, "mcr_face_recognition");

		FaceRecognition faceRec;
		ROS_INFO("Face Recognition started");
		ros::spin();

	}
	catch (visg::idt::Exception e)
	{
		std::cout << e.where() << ": " << e.what() << std::endl;
		ROS_ERROR("%s: %s", e.where(), e.what());
	}
	catch (std::exception e)
	{
		ROS_ERROR("%s", e.what());
	}
	catch (...)
	{
		ROS_ERROR("unhandelt exception");
	}
	char c;
	std::cin >> c;
	return 0;
}

