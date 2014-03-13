/** @file object_learning_node.cpp
 *  @brief Wrapper for LINEMOD object recognition
 *  @author    Alexander Hagg
 *  @memberof  b-it-bots RoboCup@Home team
 *  @version   0.1.0
 *  @date      7 March 2014
 *  @pre       First initialize the system.
 *  @copyright GNU Public License.
 *
 *  This node provides a service to learn objects using template matching based on
 *  LINE_MOD by Stefan Hinterstoisser (TUM).
 *
 *  Published topics:
 *  - visualization_marker_object: visualization of ROI and object label markers
 *  - object_candidates: object candidate point clouds
 *  - event_in: event handling
 *  - event_out: event handling
 *
 *  Events handled:
 *  - "e_init": starts node
 *  - "e_test": test current templates on scene
 *  - "e_learn": learn new template for a known object
 *  - "e_stop": halts node until resume with e_init
 *
 *  Events output:
 *  - <event_name>_done: for e_init, e_learn and e_stop
 *  - "e_test_ok"/"e_test_fail"
 */

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <pcl/filters/statistical_outlier_removal.h>

#include <mcr_tabletop_segmentation/toolbox_ros.h>
#include <mcr_object_recognition_linemod/ObjectRecognitionLinemodConfig.h>
#include <mcr_object_recognition_linemod/linemod.h>

ros::Publisher pub_scan_objects;
ros::Publisher pub_marker_object;
ros::Publisher pub_event_handling;
ros::Subscriber sub_event_handling;
ros::Subscriber sub_pointcloud;
boost::shared_ptr<tf::TransformListener> tf_listener;
boost::shared_ptr<Linemod> linemod;
CToolBoxROS tool_box;
mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig dyn_recfg_parameters;
std::vector<ObjectRegion> object_candidates;
std::vector<ObjectRegion> objects;
sensor_msgs::PointCloud2::ConstPtr pointcloud_saved_msg;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

enum states
{ 
    STOP,
    INIT,
    WAIT,
    READY,
    LEARN,
    TEST
} current_state;

/** @brief Node reconfiguration
 * 
 *  @param config configuration container
 *  @param level
 *
 */
void reconfigureCallback(mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig &config, uint32_t level)
{
    dyn_recfg_parameters = config;
    linemod.reset(new Linemod(dyn_recfg_parameters.database_folder_name, dyn_recfg_parameters.database_file_name, 
                              dyn_recfg_parameters.detection_threshold, dyn_recfg_parameters.num_modalities));
    linemod->setMask(dyn_recfg_parameters.roi_min_x, dyn_recfg_parameters.roi_max_x, dyn_recfg_parameters.roi_min_y, 
                     dyn_recfg_parameters.roi_max_y, dyn_recfg_parameters.roi_min_z, dyn_recfg_parameters.roi_max_z);
}

/** @brief callback for rgbd camera
 * 
 *  @param pointcloud_msg holding incoming pointcloud
 *
 */
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg)
{
    if (!pointcloud_msg)
    {
        return;
    }

    if (current_state == WAIT)
    {
        pointcloud_saved_msg = pointcloud_msg;
        current_state = READY;
    }
}

/** @brief event output handling
 * 
 *  @param event_msg string holding the event description
 *
 */
void eventOut(const std::string &event_msg)
{
    std_msgs::String event_out;
    event_out.data = event_msg;
    pub_event_handling.publish(event_out);
}

/** @brief event input handling
 * 
 *  @param event_msg string message holding the event description
 *
 */
bool eventCallback(const std_msgs::String::ConstPtr &event_msg)
{
    switch(current_state) 
    {
        case STOP:
        {
            if (event_msg->data == "e_init")
            {
                current_state = INIT;
            } 
            else if (event_msg->data == "e_stop")
            {
                current_state = STOP;
            }
            break;
        }
        case INIT:
        {
            if (event_msg->data == "e_stop")
            {
                current_state = STOP;
                eventOut("e_stop_done");
            }
            break;
        }
        case READY:
        {
            if (event_msg->data == "e_test")
            {
                current_state = TEST;
            }
            else if (event_msg->data == "e_learn")
            {
                current_state = LEARN;
            } 
            else if (event_msg->data == "e_stop")
            {
                current_state = STOP;
                eventOut("e_stop_done");
            }
            break;
        }
        case LEARN: case TEST:
        {
            if (event_msg->data == "e_stop")
            {
                current_state = STOP;
                eventOut("e_stop_done");
            }
            break;
        }
        default:
        {}
    }
    return true;
}

/** @brief conversion of 2D object region to pointcloud
 * 
 *  @param object 2D object description as defined in linemod.h
 *  @param cloud input cloud needed
 *  @param hue_multiplier 
 *
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert2DTemplateTo3DPointCloud(ObjectRegion object, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, int hue_multiplier)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    object_cloud->header = cloud->header;
    int offset_x = object.x;
    int offset_y = object.y;
    for (size_t feature_id = 0; feature_id < object.template_found->features.size(); feature_id++)
    {
        int point_x = offset_x + object.template_found->features.at(feature_id).x;
        int point_y = offset_y + object.template_found->features.at(feature_id).y;
        pcl::PointXYZRGBA a_point = cloud->at(point_x, point_y);
        pcl::PointXYZRGB point;
        point.x = a_point.x;
        point.y = a_point.y;
        point.z = a_point.z;
        point.r = 0;
        point.g = (255 - 20 * hue_multiplier) % 255;
        point.b = 0;
        if (!isnan(point.x) && !isnan(point.y) && !isnan(point.z))
        {
            object_cloud->push_back(point);
        }
    }
    return object_cloud;
}

void visualize(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    visualization_msgs::Marker marker_object;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker_object.type = shape;
    marker_object.header.frame_id = dyn_recfg_parameters.base_link;
    marker_object.header.stamp = ros::Time::now();
    marker_object.ns = "basic_shapes";
    marker_object.id = 0;
    marker_object.action = visualization_msgs::Marker::ADD;
    marker_object.pose.position.x = dyn_recfg_parameters.roi_min_x + 0.5 * (dyn_recfg_parameters.roi_max_x - dyn_recfg_parameters.roi_min_x);
    marker_object.pose.position.y = dyn_recfg_parameters.roi_min_y + 0.5 * (dyn_recfg_parameters.roi_max_y - dyn_recfg_parameters.roi_min_y);
    marker_object.pose.position.z = dyn_recfg_parameters.roi_min_z + 0.5 * (dyn_recfg_parameters.roi_max_z - dyn_recfg_parameters.roi_min_z);
    marker_object.pose.orientation.x = 0.0;
    marker_object.pose.orientation.y = 0.0;
    marker_object.pose.orientation.z = 0.0;
    marker_object.pose.orientation.w = 1.0;
    marker_object.scale.x = dyn_recfg_parameters.roi_max_x - dyn_recfg_parameters.roi_min_x;
    marker_object.scale.y = dyn_recfg_parameters.roi_max_y - dyn_recfg_parameters.roi_min_y;
    marker_object.scale.z = dyn_recfg_parameters.roi_max_z - dyn_recfg_parameters.roi_min_z;
    marker_object.color.r = 1.0f;
    marker_object.color.g = 1.0f;
    marker_object.color.b = 1.0f;
    marker_object.color.a = 0.5f;
    marker_object.lifetime = ros::Duration();
    pub_marker_object.publish(marker_object);

    pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
    output_cloud.header = cloud->header;
    for (size_t i = 0; i < objects.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud = convert2DTemplateTo3DPointCloud(objects.at(i), cloud, i);
        output_cloud += *object_cloud;

        visualization_msgs::Marker marker_object_label;
        marker_object_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_object_label.header.frame_id = dyn_recfg_parameters.base_link;
        marker_object_label.header.stamp = ros::Time::now();
        marker_object_label.ns = "basic_labels";
        marker_object_label.id = i + 1;
        marker_object_label.text = objects.at(i).name;
        marker_object_label.action = visualization_msgs::Marker::ADD;
        pcl::PointXYZRGBA point = cloud->at(objects.at(i).x,objects.at(i).y);
        marker_object_label.pose.position.x = objects.at(i).world_x;
        marker_object_label.pose.position.y = objects.at(i).world_y;
        marker_object_label.pose.position.z = objects.at(i).world_z;
        marker_object_label.scale.z = 0.05f;
        marker_object_label.color.r = 1.0f;
        marker_object_label.color.g = 1.0f * ((100.0 - i) / 100.0);
        marker_object_label.color.b = 1.0f;
        marker_object_label.color.a = 1.0f;
        marker_object_label.lifetime = ros::Duration(3.0);
        pub_marker_object.publish(marker_object_label);
    }
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(output_cloud, out_msg);
    pub_scan_objects.publish(out_msg);
}

void convertPointCloud()
{
    pcl::fromROSMsg(*pointcloud_saved_msg, *cloud);
    tf::StampedTransform center_transform;
    try
    {
        tf_listener->waitForTransform(dyn_recfg_parameters.base_link, pointcloud_saved_msg->header.frame_id, pointcloud_saved_msg->header.stamp, ros::Duration(5.0));
        tf_listener->lookupTransform(dyn_recfg_parameters.base_link, pointcloud_saved_msg->header.frame_id, pointcloud_saved_msg->header.stamp, center_transform);
        pcl_ros::transformPointCloud(*cloud, *cloud, center_transform);
        cloud->header.frame_id = dyn_recfg_parameters.base_link;
    } 
    catch (tf::TransformException &ex)
    {
        ROS_WARN("TF lookup failed: %s",ex.what());
    }
}

void computeDetections()
{
    object_candidates = linemod->computeDetections(cloud);
    ROS_INFO_STREAM("object_candidates.size() " << object_candidates.size());
    for (size_t i = 0; i < object_candidates.size(); i++) 
    {
        ROS_INFO_STREAM("object_candidates.at(i).name " << object_candidates.at(i).name);
        ROS_INFO_STREAM("object_candidates.at(i).similarity " << object_candidates.at(i).similarity);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud = convert2DTemplateTo3DPointCloud(object_candidates.at(i), cloud, i);
        if (object_cloud->size() == 0)
        {
            break;
        }
        pcl::PointCloud<pcl::PointXYZRGB> object_cloud_filtered;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statoutlier_filter;
        statoutlier_filter.setInputCloud(object_cloud);
        statoutlier_filter.setMeanK(25);
        statoutlier_filter.setStddevMulThresh(0.001);
        statoutlier_filter.filter(object_cloud_filtered);
        pcl::PointXYZRGB centroid = tool_box.pointCloudCentroid(*object_cloud);

        if (dyn_recfg_parameters.roi_min_x > centroid.x || centroid.x > dyn_recfg_parameters.roi_max_x || 
            dyn_recfg_parameters.roi_min_y > centroid.y || centroid.y > dyn_recfg_parameters.roi_max_y || 
            dyn_recfg_parameters.roi_min_z > centroid.z || centroid.z > dyn_recfg_parameters.roi_max_z)
        {
            continue;
        } 

        object_candidates.at(i).world_x = centroid.x;
        object_candidates.at(i).world_y = centroid.y;
        object_candidates.at(i).world_z = centroid.z;

        if (object_candidates.at(i).similarity > dyn_recfg_parameters.detection_threshold)
        {
            objects.push_back(object_candidates.at(i));
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mcr_object_learning");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    tf_listener = boost::make_shared<tf::TransformListener>();
    pub_marker_object = nh.advertise<visualization_msgs::Marker>("visualization_marker_object", 1);
    pub_scan_objects = nh.advertise<sensor_msgs::PointCloud2>("object_candidates", 1);
    pub_event_handling = nh.advertise<std_msgs::String>("event_out", 1);
    sub_event_handling = nh.subscribe<std_msgs::String>("event_in", 1, eventCallback);
    dynamic_reconfigure::Server<mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig> server;
    dynamic_reconfigure::Server<mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);
    linemod.reset(new Linemod(dyn_recfg_parameters.database_folder_name, dyn_recfg_parameters.database_file_name, 
                        dyn_recfg_parameters.detection_threshold, dyn_recfg_parameters.num_modalities));
    linemod->setMask(dyn_recfg_parameters.roi_min_x, dyn_recfg_parameters.roi_max_x, dyn_recfg_parameters.roi_min_y, 
                                dyn_recfg_parameters.roi_max_y, dyn_recfg_parameters.roi_min_z, dyn_recfg_parameters.roi_max_z);
    current_state = STOP;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        switch(current_state) 
        {
            case STOP:
            {
                sub_pointcloud.shutdown();
                break;
            }
            case INIT:
            {
                sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>(dyn_recfg_parameters.input_cloud_topic, 1, pointcloudCallback);
                current_state = WAIT;
                eventOut("e_init_done");
                break;    
            }
            case WAIT:
            {
                break;
            }
            case LEARN:
            {
                convertPointCloud();
                computeDetections();

                if (dyn_recfg_parameters.mode_visualization)
                {
                    visualize(cloud);
                }

                if (objects.size() == 0)
                {
                    std::vector<std::string> names = linemod->getObjectNames();                  
                    if (std::find(names.begin(), names.end(), dyn_recfg_parameters.learnable_object_name) != names.end())
                    {
                        linemod->addTemplate(cloud, dyn_recfg_parameters.learnable_object_name, false);
                    }
                    else
                    {
                        linemod->addTemplate(cloud, dyn_recfg_parameters.learnable_object_name, true);
                    }
                }

                eventOut("e_learn_done");
                objects.clear();
                object_candidates.clear();
                current_state = READY;
                break;
            }
            case TEST:
            {
                convertPointCloud();
                computeDetections();

                if (dyn_recfg_parameters.mode_visualization)
                {
                    visualize(cloud);
                }

                std::vector<ObjectRegion> same_objects;
                for (size_t obj = 0; obj < objects.size(); obj++)
                {
                    if (objects.at(obj).name == dyn_recfg_parameters.learnable_object_name)
                    {
                        same_objects.push_back(objects.at(obj));
                    }
                }
                if (same_objects.size() == 0)
                {
                    eventOut("e_test_fail");
                }
                else
                {
                    eventOut("e_test_success");
                }

                objects.clear();
                object_candidates.clear();
                current_state = READY;
                break;
            }
            default:
            {}
        }
    }
    return 0;
}
