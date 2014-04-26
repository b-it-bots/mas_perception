/*
 * integration_test.cpp
 *
 *  Created on: 19.08.2013
 *      Author: Frederik Hegger
 */

#include <map>
#include <math.h>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <mcr_perception_msgs/PersonList.h>


#define ROUND_TWO_DIGITS(x) ( floor(x * 100.0) / 100.0 )

ros::NodeHandle *nh_ptr = NULL;
bool people_msg_received = false;
mcr_perception_msgs::PersonList person_list;

typedef std::map<std::string, geometry_msgs::Point> DetectionMap;
typedef std::pair<std::string, geometry_msgs::Point> DetectionPair;

void peopleDetectionCallback(const mcr_perception_msgs::PersonList::ConstPtr& msg)
{
	people_msg_received = true;
	person_list = *msg;
}

TEST(BodyDetection3D, integrationTest)
{
	std::string dataset_path = "";
	std_srvs::Empty srv;
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	sensor_msgs::PointCloud2 sensor_msgs_cloud;
	DetectionMap expected_detections;
	geometry_msgs::Point point;

	// get Parameter from server
	ASSERT_TRUE(nh_ptr->getParam("dataset_path", dataset_path));

	ROS_INFO("register publisher and subscriber");
	ros::Publisher pub_pointcloud = nh_ptr->advertise<sensor_msgs::PointCloud2> ("input_pointcloud", 1);
	ros::Subscriber sub_person_msg = nh_ptr->subscribe < mcr_perception_msgs::PersonList > ("people_positions", 1, peopleDetectionCallback);
	ros::ServiceClient srv_start = nh_ptr->serviceClient < std_srvs::Empty > ("/mcr_perception/body_detection_3d/start");
	ros::ServiceClient srv_stop = nh_ptr->serviceClient < std_srvs::Empty > ("/mcr_perception/body_detection_3d/stop");

	// fill map
	point.x = 1.41975; point.y = 0.0241818; point.z = 1.10918;
	expected_detections.insert(DetectionPair("person_in_apartment_environment_1.pcd", point));
	point.x = 1.43539; point.y = -0.0805824; point.z = 1.10826;
	expected_detections.insert(DetectionPair("person_in_apartment_environment_2.pcd", point));
	point.x = 2.95234; point.y = -0.429274; point.z = 1.04201;
	expected_detections.insert(DetectionPair("person_in_apartment_environment_3.pcd", point));

	ROS_INFO("wait for service start to become available");
	// wait for and call "start" service
	ASSERT_TRUE(ros::service::waitForService("/mcr_perception/body_detection_3d/start", ros::Duration(60)));

	ROS_INFO("call service start");
	ASSERT_TRUE(srv_start.call(srv));

	for(DetectionMap::iterator it = expected_detections.begin(); it != expected_detections.end(); ++it)
	{
		ROS_INFO("load pcd file");
		// load pcd file
		ASSERT_TRUE(pcl::io::loadPCDFile<pcl::PointXYZ> (dataset_path + it->first, pcl_cloud) != -1);

		pcl::toROSMsg(pcl_cloud, sensor_msgs_cloud);

		sensor_msgs_cloud.header.frame_id = "/base_link";
		sensor_msgs_cloud.header.stamp = ros::Time::now();

		ROS_INFO("publish pointcloud");
		// publish pointcloud
		pub_pointcloud.publish(sensor_msgs_cloud);

		ROS_INFO("wait for detection results");
		// process callbacks
		do
		{
			ros::spinOnce();
			sleep(0.01);
		}while(!people_msg_received);

		ROS_INFO("detection results reveiced");

		ASSERT_TRUE(people_msg_received);
		ASSERT_EQ(person_list.persons.size(), 1);

		ASSERT_EQ(ROUND_TWO_DIGITS(person_list.persons[0].pose.pose.position.x), ROUND_TWO_DIGITS(it->second.x));
		ASSERT_EQ(ROUND_TWO_DIGITS(person_list.persons[0].pose.pose.position.y), ROUND_TWO_DIGITS(it->second.y));
		ASSERT_EQ(ROUND_TWO_DIGITS(person_list.persons[0].pose.pose.position.z), ROUND_TWO_DIGITS(it->second.z));

		people_msg_received = false;
	}

	ROS_INFO("wait for service stop to become available");
	// wait for and call "stop" service
	ASSERT_TRUE(ros::service::waitForService("/mcr_perception/body_detection_3d/stop", ros::Duration(60)));
  
	ROS_INFO("call service stop");
	ASSERT_TRUE(srv_stop.call(srv));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mcr_body_detection_3d_integration_test");
	ros::NodeHandle nh("~");
	nh_ptr = &nh;

	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

