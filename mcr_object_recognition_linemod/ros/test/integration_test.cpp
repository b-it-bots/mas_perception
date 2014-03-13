/** @file integration_test.cpp
 *  @brief Integration test for linemod object recognition using gtest
 *  @author    Alexander Hagg
 *  @memberof  b-it-bots RoboCup@Home team
 *  @version   1.0
 *  @date      25 february 2014
 *  @pre       First initialize the system.
 *  @copyright GNU Public License.
 *
 */

#include <math.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/Object.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace {

bool objects_msg_received;
bool event_msg_received;
mcr_perception_msgs::ObjectList object_list;
std::string event_msg;

void objectRecognitionCallback(const mcr_perception_msgs::ObjectList::ConstPtr &msg)
{
    object_list = *msg;
    objects_msg_received = true;
}

void eventCallback(const std_msgs::String::ConstPtr &msg)
{
    event_msg = msg->data;
    event_msg_received = true;
}


class ObjectRecognitionLinemodTest : public ::testing::Test 
{
 protected:
    ObjectRecognitionLinemodTest() 
    {
        component_namespace = "";
        dataset_path = "";
        database_path = "";
        objects_msg_received = false;
        event_msg_received = false;
    }

    virtual void SetUp() 
    {
        ros::NodeHandle nh("~");
        ASSERT_TRUE(nh.getParam("component_namespace", component_namespace));
        ASSERT_TRUE(nh.getParam("dataset_path", dataset_path));
        ASSERT_TRUE(nh.getParam("database_path", database_path));
        
        pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);
        sub_object_msg = nh.subscribe<mcr_perception_msgs::ObjectList>(component_namespace + "/objects", 1, objectRecognitionCallback);
        sub_event_msg = nh.subscribe<std_msgs::String>(component_namespace + "/event_out", 1, eventCallback);
        pub_event_msg = nh.advertise<std_msgs::String>(component_namespace + "/event_in", 1);
    }

    std::string component_namespace;
    std::string dataset_path;
    std::string database_path;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    sensor_msgs::PointCloud2 sensor_msgs_cloud;
    geometry_msgs::Point object_centroid;
    std::string object_name;
    ros::Publisher pub_pointcloud;
    ros::Subscriber sub_object_msg;
    ros::Subscriber sub_event_msg;
    ros::Publisher pub_event_msg;
};


// call service on empty point cloud. Expect no objects in response
TEST_F(ObjectRecognitionLinemodTest, ResponseEmpty)
{
    ASSERT_TRUE(pcl::io::loadPCDFile<pcl::PointXYZRGB> (dataset_path + "no_object.pcd", pcl_cloud) != -1);
    pcl::toROSMsg(pcl_cloud, sensor_msgs_cloud);
    sensor_msgs_cloud.header.frame_id = "/camera_rgb_optical_frame";
    sensor_msgs_cloud.header.stamp = ros::Time::now();

    std_msgs::String msg;
    msg.data = "e_init";
    pub_event_msg.publish(msg);
    do
    {    
        ros::spinOnce();
        sleep(0.1);
    } while (!event_msg_received);
    event_msg_received = false;

    msg.data = "e_recognize";
    do
    {
        pub_event_msg.publish(msg);
        pub_pointcloud.publish(sensor_msgs_cloud);
        ros::spinOnce();
        sleep(0.1);
    } while (!objects_msg_received);

    ASSERT_EQ(object_list.objects.size(), 0);

    objects_msg_received = false;
    event_msg_received = false;
}

// call service on a point cloud with object A and database B. Expect no objects in response
TEST_F(ObjectRecognitionLinemodTest, ResponseSingleObjectNoMatch) 
{
    ASSERT_TRUE(pcl::io::loadPCDFile<pcl::PointXYZRGB> (dataset_path + "unknown_object.pcd", pcl_cloud) != -1);
    pcl::toROSMsg(pcl_cloud, sensor_msgs_cloud);
    sensor_msgs_cloud.header.frame_id = "/camera_rgb_optical_frame";
    sensor_msgs_cloud.header.stamp = ros::Time::now();
    
    std_msgs::String msg;
    msg.data = "e_init";
    pub_event_msg.publish(msg);
    do
    {    
        ros::spinOnce();
        sleep(0.1);
    } while (!event_msg_received);
    event_msg_received = false;

    msg.data = "e_recognize";
    do
    {
        pub_event_msg.publish(msg);
        pub_pointcloud.publish(sensor_msgs_cloud);
        ros::spinOnce();
        sleep(0.1);
    } while (!event_msg_received || !objects_msg_received);

    ASSERT_EQ(object_list.objects.size(), 0);

    objects_msg_received = false;
    event_msg_received = false;
}

// call service on a point cloud with A,B and database A. Expect A and its position to be correct
TEST_F(ObjectRecognitionLinemodTest, ResponseMultipleObjectsSingleMatch)
{
    ASSERT_TRUE(pcl::io::loadPCDFile<pcl::PointXYZRGB> (dataset_path + "small_ketchup_unknown_object.pcd", pcl_cloud) != -1);
    pcl::toROSMsg(pcl_cloud, sensor_msgs_cloud);
    sensor_msgs_cloud.header.frame_id = "/camera_rgb_optical_frame";
    sensor_msgs_cloud.header.stamp = ros::Time::now();
    
    std_msgs::String msg;
    msg.data = "e_init";
    pub_event_msg.publish(msg);
    do
    {    
        ros::spinOnce();
        sleep(0.1);
    } while (!event_msg_received);
    event_msg_received = false;

    msg.data = "e_recognize";
    do
    {
        pub_event_msg.publish(msg);
        pub_pointcloud.publish(sensor_msgs_cloud);
        ros::spinOnce();
        sleep(0.1);
    } while (!event_msg_received || !objects_msg_received);

    ASSERT_EQ(object_list.objects.size(), 1);
    ASSERT_STREQ("small_ketchup", object_list.objects.at(0).name.c_str());
    ASSERT_NEAR(0.805, object_list.objects.at(0).pose.pose.position.x, 0.5);
    ASSERT_NEAR(0.022, object_list.objects.at(0).pose.pose.position.y, 0.5);
    ASSERT_NEAR(0.800, object_list.objects.at(0).pose.pose.position.z, 0.5);

    objects_msg_received = false;
    event_msg_received = false;
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mcr_object_recognition_linemod_integration_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

