/*
 *  object_recognition_height_based_node.cpp
 *
 *  Created on: 11.08.2011
 *      Author: Christian Mueller and Frederik Hegger
 */
#define DO_CANDIDATION 1
#define SHOW_OBJECTS 1
#define SHOW_PLANES 1
#define EIGEN_DONT_VECTORIZE 
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include <mcr_tabletop_segmentation/object_candidate_extraction.h>
#include <mcr_tabletop_segmentation/toolbox_ros.h>
#include <mcr_tabletop_segmentation/struct_planar_surface.h>

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <unistd.h>

#include <mcr_algorithms/statistics/minmax.hpp>
#include <mcr_perception_msgs/Object.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/GetObjectList.h>

pcl::PointCloud<pcl::PointXYZ> cloud_Inliers;
ros::Publisher pmd_pub0, pmd_pub1, pmd_pub2, pmd_pub4, pmd_pub5;

CObjectCandidateExtraction *objectCandidateExtractor;
CToolBoxROS toolBox;

int counter = 0;

int max_augment = 0;
pcl::PointCloud<pcl::PointXYZRGB> augmentPointCloud;

ros::NodeHandle* g_nh_ptr = NULL;
ros::Subscriber sub;

#define SAMPLING_DISTANCE 0.025

#define SPHERICAL_DISTANCE 2.5f

#define X_DISTANCE_MIN -1.5f
#define X_DISTANCE_MAX 1.5f

#define Y_DISTANCE_MIN -1.5f
#define Y_DISTANCE_MAX 1.5f

#define Z_DISTANCE_MIN 0.3f
#define Z_DISTANCE_MAX 1.5f

#define KINECT_ANGLE_INIT -30
#define KINECT_ANGLE_STEP 10

#define MIN_POINTS_FOR_BEST_OBJECT_CANDIDATE 100
#define MIN_DIST_TO_GRASP 1.0

#define OPENNI 0

tf::TransformListener *listenerKinectRGBToKinect;
bool setup_tf = true;
ros::ServiceClient *dynamicReconfigureClientKinectTilt;

std::string toFrame = std::string("/base_link");   //ToDO: change back "base_link
std::string kinectTopicToSubscribe = std::string("/cam3d/rgb/points");  //ToDO: change back "cam3d
//std::string kinectTopicToSubscribe = std::string("/kinect/rgb/points2");

std::vector<sensor_msgs::PointCloud2> finalClusteredObjectsMsg;
std::vector<geometry_msgs::Point> finalClusteredObjectsCenroidsMsg;
int finalBestObjectsCentroidMsg;

std::map<std::string, double> known_objects;

#define KINECT_MAX_TILT (30)
#define KINECT_MIN_TILT (-30)

int findBestObject(std::vector<geometry_msgs::Point> clusteredObjectsCentroidsMsg, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredObjects)
{

	int bestCandidateIdx = 0;
	if (clusteredObjectsCentroidsMsg.size() > 0 && clusteredObjects.size() > 0)
	{
		std::vector<int> candidatesIdx;
		for (unsigned int i = 0; i < clusteredObjects.size(); ++i)
		{
			if (clusteredObjects[i].points.size() > MIN_POINTS_FOR_BEST_OBJECT_CANDIDATE)
			{
				candidatesIdx.push_back(i);
			}
		}

		if (candidatesIdx.size() > 0)
		{
			double minDist = 99999;
			bestCandidateIdx = candidatesIdx[0];
			for (unsigned int i = 0; i < candidatesIdx.size(); ++i)
			{
				double dist = sqrt(
				        pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].x, 2) + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].y, 2)
				                + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].z, 2));
				if (dist < minDist)
				{
					bestCandidateIdx = candidatesIdx[i];
					minDist = dist;
				}

			}
		}
	}
	return bestCandidateIdx;
}

int findBestObject2(std::vector<geometry_msgs::Point> clusteredObjectsCentroidsMsg, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredObjects)
{

	int bestCandidateIdx = 0;
	if (clusteredObjectsCentroidsMsg.size() > 0 && clusteredObjects.size() > 0)
	{
		std::vector<int> candidatesIdx;
		for (unsigned int i = 0; i < clusteredObjects.size(); ++i)
		{
			if (clusteredObjects[i].points.size() > MIN_POINTS_FOR_BEST_OBJECT_CANDIDATE)
			{
				candidatesIdx.push_back(i);
			}
		}

		if (candidatesIdx.size() > 0)
		{
			double minDistToGrasp = MIN_DIST_TO_GRASP;
			bestCandidateIdx = candidatesIdx[0];
			unsigned int maxPointCloudSize = 0;
			for (unsigned int i = 0; i < candidatesIdx.size(); ++i)
			{
				double dist = sqrt(
				        pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].x, 2) + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].y, 2)
				                + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].z, 2));
				if (dist < minDistToGrasp && clusteredObjects[candidatesIdx[i]].points.size() > maxPointCloudSize)
				{
					bestCandidateIdx = candidatesIdx[i];
					maxPointCloudSize = clusteredObjects[candidatesIdx[i]].points.size();
				}

			}
		}
	}
	return bestCandidateIdx;
}

//TEST OBJECT DETECTION
void objectCandidateExtractionCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	ROS_DEBUG("objectCandidateExtractionCallback started...");
	std::vector<sensor_msgs::PointCloud2> clusteredObjectsMsg;
	std::vector<geometry_msgs::Point> clusteredObjectsCentroidsMsg;
	sensor_msgs::PointCloud2 pointsCloudMsg;
	int bestCandidateIdx = 0;

	if (point_cloud_msg->width <= 0 && point_cloud_msg->height <= 0)
	{
		ROS_DEBUG("pointCloud Msg empty");
		return;
	}

	ros::Time start, finish;
	start = ros::Time::now();

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredObjects;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredPlanes;
	pcl::PointCloud < pcl::PointXYZRGBNormal > planar_point_cloud;
	pcl::PointCloud < pcl::PointXYZRGB > point_cloud;
	pcl::PointCloud < pcl::PointXYZRGBNormal > point_cloud_RGB;
	pcl::PointCloud < pcl::PointXYZRGBNormal > point_cloud_RGB2;
	pcl::PointCloud < pcl::PointXYZRGB > augmentPointCloudCopy;
	std::vector<StructPlanarSurface*> hierarchyPlanes;
	sensor_msgs::PointCloud2 pointCloud2MsgTransformed;

	std::string fromFrame = std::string(point_cloud_msg->header.frame_id);
	ROS_DEBUG("pointCloud tf transform... ");

	if (!toolBox.transformPointCloud(*listenerKinectRGBToKinect, fromFrame, toFrame, *point_cloud_msg, pointCloud2MsgTransformed, 2))
	{
		ROS_DEBUG("pointCloud tf transform...failed");
		counter = 0;
		return;
	}
	ROS_DEBUG("pointCloud tf transform...done");
	pcl::fromROSMsg(pointCloud2MsgTransformed, point_cloud);

	if (point_cloud.points.size() <= 0)
	{
		ROS_DEBUG("pointCloud empty ");
		return;
	}

	if (counter == 0)
	{
		augmentPointCloud = point_cloud;
		augmentPointCloud.header.frame_id = point_cloud.header.frame_id;
		//augmentPointCloud.header.stamp=ros::Time::now();
		ROS_DEBUG("augment pointCloud = %d points", (int) augmentPointCloud.size());
		//		kinectTiltDegree += KINECT_ANGLE_STEP;
	}
	else if (max_augment == (counter - 1))
	{

		//	kinectTiltDegree = KINECT_ANGLE_INIT;
		//setKinectTilt(dynamicReconfigureClientKinectTilt,kinectTiltDegree);
		ROS_DEBUG("extract object candidates");

		augmentPointCloud = toolBox.filterDistance(augmentPointCloud, X_DISTANCE_MIN, X_DISTANCE_MAX, "x");
		augmentPointCloud = toolBox.filterDistance(augmentPointCloud, Y_DISTANCE_MIN, Y_DISTANCE_MAX, "y");
		augmentPointCloud = toolBox.filterDistance(augmentPointCloud, Z_DISTANCE_MIN, Z_DISTANCE_MAX, "z");

		//toolBox.subsampling(augmentPointCloud, 0.004); //0.01
		toolBox.subsampling(augmentPointCloud, 0.008);  //0.01

		augmentPointCloudCopy = augmentPointCloud;

		if (augmentPointCloudCopy.points.size() == 0)
		{
			ROS_DEBUG("point cloud empty after filtering");
			counter = 0;
			return;
		}

		ROS_DEBUG("Point cloud size: %i", augmentPointCloudCopy.points.size());

		objectCandidateExtractor->extractObjectCandidates(augmentPointCloudCopy, planar_point_cloud, hierarchyPlanes);

		if (DO_CANDIDATION || SHOW_OBJECTS || SHOW_PLANES)
		{
			mcr_perception_msgs::ObjectList outList;

			if (hierarchyPlanes.size() <= 0)
				ROS_WARN("no plane detected");

			unsigned int obj_count = 0;
			for (unsigned int iterPlanes = 0; iterPlanes < hierarchyPlanes.size(); iterPlanes++)
			{
				for (unsigned int iterObject = 0; iterObject < hierarchyPlanes[iterPlanes]->clusteredObjects.size(); iterObject++, obj_count++)
				{
					if (DO_CANDIDATION || SHOW_OBJECTS)
					{
						pcl::PointCloud < pcl::PointXYZRGBNormal > current_obj = hierarchyPlanes[iterPlanes]->clusteredObjects[iterObject];
						clusteredObjects.push_back(current_obj);

						//std::cout << "-----------SIZE:" << current_obj.points.size() << std::endl;

						double min_x, max_x, min_y, max_y, min_z, max_z;
						MinMax::determineMinMax3D(clusteredObjects[iterObject], min_x, max_x, min_y, max_y, min_z, max_z);

						//std::cout << "obj " << iterObject << " HEIGHT: " << fabs((min_z - max_z)) << std::endl;

						std::map<std::string, double>::iterator end = known_objects.end();
						std::string closest_name = "";
						double closest_height = DBL_MAX;
						for (std::map<std::string, double>::const_iterator it = known_objects.begin(); it != end; ++it)
						{
							if (fabs(fabs((min_z - max_z)) - it->second) < 0.015)
							{
								closest_name = it->first;
								closest_height = it->second;
							}
						}

						if (closest_name.length() > 0)
						{
							//std::cout << "closest: " << closest_name << " height: " << closest_height << std::endl;

							mcr_perception_msgs::Object out;  // Our output msg

							pcl_conversions::fromPCL(clusteredObjects[iterObject].header, out.pose.header);

							out.name = closest_name;

							out.pose.pose.orientation.x = 0;
							out.pose.pose.orientation.y = 0;
							out.pose.pose.orientation.z = 0;
							out.pose.pose.orientation.w = 0;

							pcl::PointXYZ centroid = toolBox.pointCloudCentroid(clusteredObjects[iterObject]);
							out.pose.pose.position.x = centroid.x;
							out.pose.pose.position.y = centroid.y;
							out.pose.pose.position.z = centroid.z;

							pcl::toROSMsg(clusteredObjects[iterObject], pointsCloudMsg);
							out.pointcloud = pointsCloudMsg;

							outList.objects.push_back(out);
							ROS_INFO_STREAM(
							        " Found object: " << out.name << " with height: " << fabs(min_z - max_z) << " (closest height: " << closest_height << ")");
						}
						else
						{
							ROS_INFO_STREAM("unkown object with height: " << fabs(min_z - max_z));
						}

					}

					if (DO_CANDIDATION)
					{
						pcl::toROSMsg(clusteredObjects.back(), pointsCloudMsg);
						pointsCloudMsg.header.frame_id = pointsCloudMsg.header.frame_id;
						pointsCloudMsg.header.stamp = ros::Time::now();
						clusteredObjectsMsg.push_back(pointsCloudMsg);

						pcl::PointXYZ centroid = toolBox.pointCloudCentroid(clusteredObjects.back());
						geometry_msgs::Point pointMsg;
						pointMsg.x = centroid.x;
						pointMsg.y = centroid.y;
						pointMsg.z = centroid.z;
						clusteredObjectsCentroidsMsg.push_back(pointMsg);
					}

				}
				if (SHOW_PLANES)
					clusteredPlanes.push_back(hierarchyPlanes[iterPlanes]->pointCloud);
			}

			if (obj_count <= 0)
				ROS_WARN("no objects extracted");

			pmd_pub5.publish(outList);
		}

		if (DO_CANDIDATION)
		{
			bestCandidateIdx = findBestObject(clusteredObjectsCentroidsMsg, clusteredObjects);
			finalClusteredObjectsMsg = clusteredObjectsMsg;
			finalClusteredObjectsCenroidsMsg = clusteredObjectsCentroidsMsg;
			finalBestObjectsCentroidMsg = bestCandidateIdx;

			mcr_perception_msgs::ObjectList pointcloud_3d_msg;

			for (size_t i = 0; i < clusteredObjectsMsg.size(); ++i)
			{
				mcr_perception_msgs::Object obj;
				obj.pointcloud = clusteredObjectsMsg[i];

				pointcloud_3d_msg.objects.push_back(obj);
			}

			pmd_pub4.publish(pointcloud_3d_msg);
		}

		if (SHOW_OBJECTS)
		{
			toolBox.markClusteredPointCloud(clusteredObjects, point_cloud_RGB);
			pcl::toROSMsg(point_cloud_RGB, pointsCloudMsg);
			pointsCloudMsg.header.frame_id = point_cloud.header.frame_id;
			pointsCloudMsg.header.stamp = ros::Time::now();
			pmd_pub1.publish(pointsCloudMsg);
		}

		if (SHOW_PLANES)
		{
			toolBox.markClusteredPointCloud(clusteredPlanes, point_cloud_RGB2);
			pcl::toROSMsg(point_cloud_RGB2, pointsCloudMsg);
			pointsCloudMsg.header.frame_id = point_cloud.header.frame_id;
			pointsCloudMsg.header.stamp = ros::Time::now();
			pmd_pub2.publish(pointsCloudMsg);

		}
		counter = -1;

	}
	else
	{
		augmentPointCloud.header.frame_id = point_cloud.header.frame_id;
		augmentPointCloud += point_cloud;
		ROS_DEBUG("augment pointCloud = %d points", (int) augmentPointCloud.size());
	}

	++counter;
	finish = ros::Time::now();

	ROS_DEBUG("[objectCandidateExtractionCallback] Execution time =  %lfsec", (finish.toSec() - start.toSec()));
}

/*Service getKinect_objectCandidateList*/
bool getKinectObjectCandidates3D(mcr_perception_msgs::GetObjectList::Request &req, mcr_perception_msgs::GetObjectList::Response &res)
{
	for (size_t i = 0; i < finalClusteredObjectsMsg.size(); ++i)
	{
		mcr_perception_msgs::Object obj;

		obj.pointcloud = finalClusteredObjectsMsg[i];
		obj.pose.pose.position.x = finalClusteredObjectsCenroidsMsg[i].x;
		obj.pose.pose.position.y = finalClusteredObjectsCenroidsMsg[i].y;
		obj.pose.pose.position.z = finalClusteredObjectsCenroidsMsg[i].z;

		res.objects.push_back(obj);
	}

	//res.bestPointCloudCentroidIndex = finalBestObjectsCentroidMsg;
	finalClusteredObjectsMsg.clear();
	finalClusteredObjectsCenroidsMsg.clear();

	return true;
}

bool start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	sub = g_nh_ptr->subscribe(kinectTopicToSubscribe, 1, objectCandidateExtractionCallback);

	ROS_INFO("object recognition height based ENABLED");
	return true;
}

bool stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	sub.shutdown();
	ROS_INFO("object recognition height based DISABLED");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_recognition_height_based");

	ros::NodeHandle n("~");
	g_nh_ptr = &n;

	ros::ServiceServer srvGetKinectObjectCandidates;

	listenerKinectRGBToKinect = new tf::TransformListener();
	objectCandidateExtractor = new CObjectCandidateExtraction(n, SPHERICAL_DISTANCE);

	pmd_pub1 = n.advertise < sensor_msgs::PointCloud2 > ("visualize_detected_objects", 1);
	pmd_pub2 = n.advertise < sensor_msgs::PointCloud2 > ("visualize_detected_planes", 1);
	pmd_pub4 = n.advertise < mcr_perception_msgs::ObjectList > ("detected_objects", 1);
	pmd_pub5 = n.advertise < mcr_perception_msgs::ObjectList > ("recognized_objects", 1);

	ros::ServiceServer srv_start = n.advertiseService("start", start);
	ros::ServiceServer srv_stop = n.advertiseService("stop", stop);

	srvGetKinectObjectCandidates = n.advertiseService("get_recognized_objects", getKinectObjectCandidates3D);

	//ros::Subscriber sub = n.subscribe(kinectTopicToSubscribe, 1, objectCandidateExtractionCallback);

	int num_objects = 0;
	n.getParam("num_known_objects", num_objects);
	ROS_INFO_STREAM("Number of known objects: " << num_objects);

	std::string obj_name;
	double obj_height;
	for (int i = 0; i < num_objects; ++i)
	{
		std::ostringstream name_field, height_field;

		name_field << "obj_" << (i + 1) << "_name";
		n.getParam(name_field.str(), obj_name);

		height_field << "obj_" << (i + 1) << "_height";
		n.getParam(height_field.str(), obj_height);

		ROS_INFO_STREAM("     object name: " << obj_name << ", height: " << obj_height);

		known_objects[obj_name] = obj_height;
	}

	ros::spin();

	return 0;
}

