/** @file object_recognition_node.cpp
 *  @brief Wrapper for LINEMOD object recognition
 *  @author    Alexander Hagg
 *  @memberof  b-it-bots RoboCup@Home team
 *  @version   0.1.0
 *  @date      7 March 2014
 *  @pre       First initialize the system.
 *  @copyright GNU Public License.
 * 
 *  This node provides a service to recognize objects that have been learned
 *  by the object_learning_linemod.cpp node, using template matching based on
 *  LINE_MOD by Stefan Hinterstoisser (TUM).
 *
 *  The user can configure the matching threshold, which is defined as a percentage. 
 *  A strong threshold would be 98%, 90% would be much weaker. Values below 85% are meaningless. 
 *
 *  Published topics:
 *  - visualization_marker_object: visualization of ROI and object label markers
 *  - objects_visualization: object pointclouds for visualization
 *  - objects: identified objects (pointclouds, position of centroids, name)
 *  - event_in: event handling
 *  - event_out: event handling
 *
 *  Events handled:
 *  - "e_init": initialize system, subscribe to pointcloud topic
 *  - "e_recognize": recognize objects
 *  - "e_stop": stop system and return to pre-initialization
 *
 */

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/Object.h>
#include <mcr_tabletop_segmentation/toolbox_ros.h>
#include <mcr_object_recognition_linemod/ObjectRecognitionLinemodConfig.h>
#include <mcr_object_recognition_linemod/linemod.h>

ros::Publisher pub_scan_objects;
ros::Publisher pub_marker_object;
ros::Publisher pub_objects;
ros::Publisher pub_event_handling;
ros::Subscriber sub_event_handling;
ros::Subscriber sub_pointcloud;
boost::shared_ptr<tf::TransformListener> tf_listener;
std::vector<mcr_perception_msgs::Object> hypothesis;
std::vector<ObjectRegion> object_candidates_unfiltered, object_candidates, objects;
boost::shared_ptr<Linemod> linemod;
CToolBoxROS tool_box;
bool publish_object_pointclouds;
mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig dyn_recfg_parameters;
sensor_msgs::PointCloud2::ConstPtr pointcloud_saved_msg;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

enum states
{ 
    STOP,
    INIT,
    WAIT,
    READY,
    RECOGNIZE
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

    pointcloud_saved_msg = pointcloud_msg;
    if (current_state == WAIT)
    {
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
            if (event_msg->data == "e_recognize")
            {
                current_state = RECOGNIZE;
            }
            else if (event_msg->data == "e_stop")
            {
                current_state = STOP;
                eventOut("e_stop_done");
            }
            break;
        }
        case RECOGNIZE:
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

void visualize(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, std::vector<ObjectRegion> objects)
{
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mcr_object_recognition");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    tf_listener = boost::make_shared<tf::TransformListener>();
    pub_marker_object = nh.advertise<visualization_msgs::Marker>("visualization_marker_object", 1);
    pub_scan_objects = nh.advertise<sensor_msgs::PointCloud2>("objects_visualization", 1);
    pub_objects = nh.advertise<mcr_perception_msgs::ObjectList>("objects", 1);
    pub_event_handling = nh.advertise<std_msgs::String>("event_out", 1);
    sub_event_handling = nh.subscribe<std_msgs::String>("event_in", 1, eventCallback);
    dynamic_reconfigure::Server<mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig> server;
    dynamic_reconfigure::Server<mcr_object_recognition_linemod::ObjectRecognitionLinemodConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);
    linemod.reset(new Linemod(dyn_recfg_parameters.database_folder_name, dyn_recfg_parameters.database_file_name, 
                        dyn_recfg_parameters.detection_threshold, dyn_recfg_parameters.num_modalities));
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
                linemod->setMask(dyn_recfg_parameters.roi_min_x, dyn_recfg_parameters.roi_max_x, dyn_recfg_parameters.roi_min_y, 
                                dyn_recfg_parameters.roi_max_y, dyn_recfg_parameters.roi_min_z, dyn_recfg_parameters.roi_max_z);
                current_state = WAIT;
                eventOut("e_init_done");
                break;    
            }
            case WAIT:
            {
                break;
            }
            case READY:
            {
                break;
            }
            case RECOGNIZE:
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
                object_candidates.clear();
                object_candidates_unfiltered = linemod->computeDetections(cloud);
                
                // Make sure we have unique objects
                for (size_t i = 0; i < object_candidates_unfiltered.size(); i++) 
                {
                    if (object_candidates_unfiltered.at(i).similarity <= dyn_recfg_parameters.detection_threshold)
                    {
                        continue;
                    }
                    
                    int found_object = -1;
                    double found_object_similarity = 0.0f;
                    for (size_t candidate = 0; candidate < object_candidates.size(); candidate++)
                    {
                        if (object_candidates.at(candidate).name != object_candidates_unfiltered.at(i).name)
                        {
                            continue;
                        }
                        if (object_candidates.at(candidate).similarity > found_object_similarity) 
                        {
                            found_object = candidate;
                            found_object_similarity = object_candidates.at(candidate).similarity;
                        }
                    }
                    if (found_object == -1)
                    {
                        object_candidates.push_back(object_candidates_unfiltered.at(i));  
                        found_object == -1;
                    } 
                    else if (object_candidates_unfiltered.at(i).similarity > object_candidates.at(found_object).similarity)
                    {
                        object_candidates.erase(object_candidates.begin() + found_object);
                        object_candidates.push_back(object_candidates_unfiltered.at(i));  
                        found_object == -1;
                    }
                }

                for (size_t i = 0; i < object_candidates.size(); i++)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud = convert2DTemplateTo3DPointCloud(object_candidates.at(i), cloud, i);
                    pcl::PointXYZRGB point = tool_box.pointCloudCentroid(*object_cloud);
                    object_candidates.at(i).world_x = point.x;
                    object_candidates.at(i).world_y = point.y;
                    object_candidates.at(i).world_z = point.z;
                }

                for (size_t hypothesized_object = 0; hypothesized_object < object_candidates.size(); hypothesized_object++)
                {
                    bool valid_object_hypothesis = true;
                    for (size_t detected_object = 0; detected_object < objects.size(); detected_object++)
                    {
                        double position_difference = sqrt(pow(object_candidates.at(hypothesized_object).world_x - objects.at(detected_object).world_x, 2) + 
                                                          pow(object_candidates.at(hypothesized_object).world_y - objects.at(detected_object).world_y, 2) +
                                                          pow(object_candidates.at(hypothesized_object).world_z - objects.at(detected_object).world_z, 2));

                        if (position_difference < dyn_recfg_parameters.minimum_object_distance)
                        {
                            valid_object_hypothesis = false;
                            break;
                        }
                    }
                    if (valid_object_hypothesis)
                    {
                        objects.push_back(object_candidates.at(hypothesized_object));
                    }
                }

                // Gather objects for publishing
                mcr_perception_msgs::ObjectList objects_msg;
                for (size_t object = 0; object < objects.size(); object++)
                {
                    mcr_perception_msgs::Object msg_object;
                    msg_object.name = objects.at(object).name;
                    msg_object.probability = objects.at(object).similarity;
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud = convert2DTemplateTo3DPointCloud(object_candidates.at(object), cloud, object);
                    toROSMsg(*object_cloud, msg_object.pointcloud);
                    msg_object.pointcloud.header.frame_id = dyn_recfg_parameters.base_link;
                    msg_object.pose.header.frame_id = dyn_recfg_parameters.base_link;

                    geometry_msgs::PointStamped centroid;
                    centroid.header.frame_id = dyn_recfg_parameters.base_link;
                    centroid.header.stamp = msg_object.pointcloud.header.stamp;
                    pcl::PointXYZRGB point = tool_box.pointCloudCentroid(*object_cloud);

                    msg_object.pose.pose.position.x = point.x;
                    msg_object.pose.pose.position.y = point.y;
                    msg_object.pose.pose.position.z = point.z;
                    objects_msg.objects.push_back(msg_object);
                }
                
                pub_objects.publish(objects_msg);

                if (dyn_recfg_parameters.mode_visualization)
                {
                    visualize(cloud, objects);
                }

                current_state = READY;
                eventOut("e_recognize_done");
                object_candidates_unfiltered.clear();
                object_candidates.clear();
                objects.clear();
                break;
            }
            default:
            {}
        }

    }
    return 0;
}
