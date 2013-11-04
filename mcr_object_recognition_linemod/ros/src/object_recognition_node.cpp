/**
 * object_recognition_linemod.cpp
 *
 * Created on: 7 March 2013
 *     Author: Alexander Hagg
 *
 *
 * This node provides a service to recognize objects that have been learned
 * by the object_learning_linemod.cpp node, using template matching based on
 * LINE_MOD by Stefan Hinterstoisser (TUM).
 *
 * The input is an RGB, depth image and a point cloud. The user can configure the
 * matching threshold, which is defined as a percentage. A strong threshold would be 98%,
 * 90% would be much weaker. Values below 85% are meaningless. "needed_voting_rounds" has no use yet.
 *
 * Provides services:
 *   1) "get_recognized_objects"
 *
 * Publishes:
 *   1) "recognized_objects"
 *      For debugging purposes. Shows point cloud of objects
 *
 * A part of the code was taken from the OpenCV (v2.4.4) example
 *
 */

#include <string>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>

#include <mcr_object_recognition_linemod/ObjectRecognitionLinemodConfig.h>

//#include <mcr_perception_msgs/UpdateObjectList.h>
#include <mcr_perception_msgs/GetObjectList.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/Object.h>
#include <mcr_tabletop_segmentation/toolbox_ros.h>

#include "linemod_recognition.h"

#define UPDATE_NO_ERROR 0
#define UPDATE_ERROR_NO_OBJECTS_PROVIDED 1
#define UPDATE_ERROR_OBJECT_ALREADY_IN_DB 2
#define UPDATE_ERROR_OBJECT_NOT_IN_DB 3

#ifndef M_PI
#define M_PI    3.14159f
#endif

using namespace std;
using sensor_msgs::PointCloud;
namespace enc = sensor_msgs::image_encodings;

ros::Publisher pub_scan;
ros::Publisher pub_objects_in_db;
ros::Publisher pub_objects_active;
ros::Subscriber sub;
tf::TransformListener *tf_listener;
LinemodRecognition* linemod_recognition;
mcr_perception_msgs::ObjectList recognized_objects, objects_searched, objects_in_db;
int image_topic_wait, cloud_topic_wait, controller_topic_wait;
string filename, input_color, input_cloud, input_depth, input_head_controller_status;
int max_retries, needed_voting_rounds;
double matching_threshold;
CToolBoxROS toolBox;
string frame_id, filename_categories;
std::vector<std::string> class_ids, class_ids_in_database;
bool head_forward;
std::vector<mcr_perception_msgs::Object> hypothesis;
double minimum_object_distance;

void callback(mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig &config, uint32_t level)
{
	filename = config.filename;
	filename_categories = config.filename_categories;
	max_retries = config.max_retries;
	needed_voting_rounds = config.needed_voting_rounds;
	image_topic_wait = config.image_topic_wait;
	cloud_topic_wait = config.cloud_topic_wait;
	controller_topic_wait = config.controller_topic_wait;
	input_color = config.input_color;
	input_depth = config.input_depth;
	input_cloud = config.input_cloud;
	input_head_controller_status = config.input_head_controller_status;
	minimum_object_distance = config.minimum_object_distance;
	matching_threshold = config.matching_threshold;
	if (linemod_recognition)
	{
		linemod_recognition->setMatchingThreshold(matching_threshold);
	}
	ROS_INFO_STREAM("matching_threshold " << matching_threshold);
}

bool getObjects(mcr_perception_msgs::GetObjectList::Request &req, mcr_perception_msgs::GetObjectList::Response &res)
{
	ROS_INFO("service call triggered");
	ros::NodeHandle nh("~");

	if (class_ids.empty())
	{
		if (!linemod_recognition->getObjectList().empty())
		{
			ROS_INFO_STREAM("currently searching for an empty set of objects although database is non empty");
			return false;
		}
		else
		{
			ROS_INFO_STREAM("currently searching for an empty set of objects because database is empty");
			return false;
		}
	}

	/*
	pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr jointState;
	jointState = ros::topic::waitForMessage < pr2_controllers_msgs::JointTrajectoryControllerState
	        > (input_head_controller_status, ros::Duration(controller_topic_wait));

	if (jointState && jointState->actual.positions.at(0) < -0.5 * M_PI)
	{
		head_forward = false;
	}
	else
	{
		head_forward = true;
	}*/

	for (size_t retry_ctr = 0; retry_ctr < max_retries; retry_ctr++)
	{
		sensor_msgs::Image::ConstPtr const_input_color, const_input_depth;
		sensor_msgs::PointCloud2::ConstPtr const_input_cloud;
		pcl::PointCloud < pcl::PointXYZ > const_output_cloud;

		const_input_cloud = ros::topic::waitForMessage < sensor_msgs::PointCloud2 > (input_cloud, ros::Duration(cloud_topic_wait));
		const_input_color = ros::topic::waitForMessage < sensor_msgs::Image > (input_color, ros::Duration(image_topic_wait));
		const_input_depth = ros::topic::waitForMessage < sensor_msgs::Image > (input_depth, ros::Duration(image_topic_wait));

		cv_bridge::CvImagePtr cv_color_ptr, cv_depth_ptr;
		ROS_INFO("Retry number: %lu ...", retry_ctr);
		if (const_input_color && const_input_depth && const_input_cloud)
		{
			try
			{

				cv::Mat color, depth;
				cv_color_ptr = cv_bridge::toCvCopy(const_input_color, enc::BGR8);
				cv_depth_ptr = cv_bridge::toCvCopy(const_input_depth, enc::MONO16);

				if (!head_forward)
				{
					flip(cv_color_ptr->image, cv_color_ptr->image, -1);
					flip(cv_depth_ptr->image, cv_depth_ptr->image, -1);
				}

				color = cv_color_ptr.get()->image;
				depth = cv_depth_ptr.get()->image;
				std::vector<ObjectRegion> object_2d_list;
				object_2d_list = linemod_recognition->recognizeObjects(color, depth, class_ids);

				ROS_INFO("Recognized %lu objects", object_2d_list.size());
				for (size_t i = 0; i < object_2d_list.size(); i++)
				{
					mcr_perception_msgs::Object object;
					object.name = object_2d_list.at(i)._class_id;
					object.probability = object_2d_list.at(i)._similarity;

					pcl::PointCloud < pcl::PointXYZRGB > cloud;
					pcl::PointCloud < pcl::PointXYZ > subcloud;
					pcl::fromROSMsg(*const_input_cloud, cloud);
					for (size_t modality_id = 0; modality_id < object_2d_list.at(i)._num_modalities; modality_id++)
					{
						int offset_x = object_2d_list.at(i)._x;
						int offset_y = object_2d_list.at(i)._y;

						for (size_t feature_id = 0; feature_id < object_2d_list.at(i)._templates.at(modality_id).features.size(); feature_id++)
						{
							pcl::PointXYZRGB point;
							if (!head_forward)
							{
								point = cloud.at(
								        cv_color_ptr->image.cols - (offset_x + object_2d_list.at(i)._templates.at(modality_id).features.at(feature_id).x),
								        cv_color_ptr->image.rows - (offset_y + object_2d_list.at(i)._templates.at(modality_id).features.at(feature_id).y));
							}
							else
							{
								point = cloud.at((offset_x + object_2d_list.at(i)._templates.at(modality_id).features.at(feature_id).x),
								                 (offset_y + object_2d_list.at(i)._templates.at(modality_id).features.at(feature_id).y));
							}

							pcl::PointXYZ reduced_point;
							reduced_point.x = point.x;
							reduced_point.y = point.y;
							reduced_point.z = point.z;

							if (modality_id == 1 && !isnan(point.x) && !isnan(point.y) && !isnan(point.z))
							{
								subcloud.push_back(reduced_point);
							}

						}
					}

					pcl::toROSMsg(cloud, object.pointcloud);
					object.pointcloud.header.frame_id = const_input_cloud->header.frame_id;
					object.pose.header.frame_id = const_input_cloud->header.frame_id;

					geometry_msgs::PointStamped centroid, centroid_world;
					centroid.header.frame_id = object.pointcloud.header.frame_id;
					centroid.header.stamp = object.pointcloud.header.stamp;
					pcl::PointXYZ point = toolBox.pointCloudCentroid(subcloud);
					centroid.point.x = point.x;
					centroid.point.y = point.y;
					centroid.point.z = point.z;

					tf_listener->transformPoint("base_link", centroid, centroid_world);

					object.pose.pose.position.x = centroid_world.point.x;
					object.pose.pose.position.y = centroid_world.point.y;
					object.pose.pose.position.z = centroid_world.point.z;

					hypothesis.push_back(object);

					for (size_t i = 0; i < subcloud.size(); i++)
					{
						if (isnan(subcloud.points[i].x) || isnan(subcloud.points[i].y) || isnan(subcloud.points[i].z))
						{
							continue;
						}
						const_output_cloud.push_back(subcloud.points[i]);
					}

					static tf::TransformBroadcaster br;
					tf::Transform transform;
					transform.setOrigin(tf::Vector3(object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z));
					transform.setRotation(tf::Quaternion(0, 0, 0));
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "object"));
					const_input_color.reset();
					const_input_depth.reset();
				}

				/* Select only the best object matches per position
				 * Problem: 	linemod object recognition returns a set of objects but since it uses pattern matching,
				 * 				false positives (multiple objects on same location) can be returned
				 * Solution:	solve this by only taking the most probable objects around a centroid position
				 * 				(plus a certain tolerance minimum_object_distance)
				 */
				for (size_t hypothesized_object = 0; hypothesized_object < hypothesis.size(); hypothesized_object++)
				{
					bool valid_object_hypothesis = true;
					for (size_t detected_object = 0; detected_object < res.objects.size(); detected_object++)
					{

						geometry_msgs::Point hypothesis_position = hypothesis.at(hypothesized_object).pose.pose.position;
						geometry_msgs::Point object_position = res.objects.at(detected_object).pose.pose.position;

						double position_difference = sqrt(
						        pow(hypothesis_position.x - object_position.x, 2) + pow(hypothesis_position.y - object_position.y, 2)
						                + pow(hypothesis_position.z - object_position.z, 2));

						if (position_difference < minimum_object_distance
						        && hypothesis.at(hypothesized_object).probability < res.objects.at(detected_object).probability)
						{
							valid_object_hypothesis = false;
							break;
						}
					}
					if (valid_object_hypothesis)
					{
						res.objects.push_back(hypothesis.at(hypothesized_object));
					}
				}

				if (object_2d_list.size() > 0)
				{
					pcl_conversions::toPCL(const_input_cloud->header, const_output_cloud.header);

					sensor_msgs::PointCloud2 out_msg;
					pcl::toROSMsg(const_output_cloud, out_msg);
					pub_scan.publish(out_msg);
					const_output_cloud.clear();
					hypothesis.clear();
					break;
				}
				hypothesis.clear();

			}
			catch (cv_bridge::Exception &ex)
			{
				ROS_ERROR("cv_bridge exception: %s", ex.what());
			}
			catch (tf::TransformException &ex)
			{
				ROS_WARN("No tf available: %s", ex.what());
			}
			catch (ros::Exception &ex)
			{
				ROS_WARN("General exception caught: %s", ex.what());
			}
			catch (...)
			{
				printf("fatal error");
			}

		}
		else
		{
			ROS_INFO("Waiting for stereo camera topics: ");
			if (!const_input_color)
				ROS_INFO("const_input_color");
			if (!const_input_depth)
				ROS_INFO("const_input_depth");
			if (!const_input_cloud)
				ROS_INFO("const_input_cloud");
		}
	}

	return true;
}

/*
bool updateObjectSet(mcr_perception_msgs::UpdateObjectList::Request &req, mcr_perception_msgs::UpdateObjectList::Response &res)
{
	std::set<std::string> objectSet(class_ids.begin(), class_ids.end());
	if (req.objects.size() == 0)
	{
		res.error = UPDATE_ERROR_NO_OBJECTS_PROVIDED;
		return false;
	}

	for (size_t requestedObject = 0; requestedObject < req.objects.size(); requestedObject++)
	{
		if (req.action.at(requestedObject))
		{
			if (!objectSet.insert(req.objects.at(requestedObject)).second)
				return UPDATE_ERROR_OBJECT_ALREADY_IN_DB;
		}
		else
		{
			if (objectSet.erase(req.objects.at(requestedObject)) != 1)
			{
				return UPDATE_ERROR_OBJECT_NOT_IN_DB;
			}
		}
	}

	class_ids = std::vector<std::string>(objectSet.begin(), objectSet.end());
	res.error = UPDATE_NO_ERROR;
	return true;
}*/

bool start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("DEPRECATED SERVICE: object recognition ENABLED");
	return true;
}

bool stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("DEPRECATED SERVICE: object recognition DISABLED");
	return true;
}

void publish_object_lists()
{
	pub_objects_in_db.publish(objects_in_db);
	for (size_t i = 0; i < class_ids.size(); i++)
	{
		mcr_perception_msgs::Object current_object;
		current_object.name = class_ids[i];
		objects_searched.objects.push_back(current_object);
	}
	pub_objects_active.publish(objects_searched);
}

int main(int argc, char **argv)
{
	/* init ROS node with a name and a node handle*/
	ros::init(argc, argv, "mcr_object_recognition_linemod");
	ros::NodeHandle nh("~");
	ros::ServiceServer srvGetObjectList, srvUpdateObjectSet, srv_start, srv_stop;
	tf_listener = new tf::TransformListener();

	pub_scan = nh.advertise < sensor_msgs::PointCloud2 > ("recognized_objects", 1);
	pub_objects_in_db = nh.advertise < mcr_perception_msgs::ObjectList > ("objects_in_database", 1);
	pub_objects_active = nh.advertise < mcr_perception_msgs::ObjectList > ("objects_in_search", 1);

	dynamic_reconfigure::Server < mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig > server;
	dynamic_reconfigure::Server<mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	linemod_recognition = new LinemodRecognition(filename, matching_threshold);
	class_ids_in_database = class_ids = linemod_recognition->getObjectList();
	for (size_t i = 0; i < class_ids_in_database.size(); i++)
	{
		mcr_perception_msgs::Object current_object;
		current_object.name = class_ids_in_database[i];
		objects_in_db.objects.push_back(current_object);
	}

	srv_start = nh.advertiseService("start", start);
	srv_stop = nh.advertiseService("stop", stop);
	srvGetObjectList = nh.advertiseService("get_recognized_objects", getObjects);
	//srvUpdateObjectSet = nh.advertiseService("update_object_set", updateObjectSet);

	head_forward = true;

	ros::Rate loop_rate(2);

	while (ros::ok())
	{
		ros::spinOnce();
		publish_object_lists();
		loop_rate.sleep();
	}

	return 0;
}
