/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_SCENE_SEGMENTATION_NODE_H
#define MCR_SCENE_SEGMENTATION_SCENE_SEGMENTATION_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <mcr_scene_segmentation/scene_segmentation.h>
#include <mcr_scene_segmentation/clustered_point_cloud_visualizer.h>
#include <mcr_scene_segmentation/bounding_box_visualizer.h>
#include <mcr_scene_segmentation/label_visualizer.h>
#include <mcr_scene_segmentation/cloud_accumulation.h>

#include <dynamic_reconfigure/server.h>
#include <mcr_scene_segmentation/SceneSegmentationConfig.h>
#include <string>

using mcr::visualization::BoundingBoxVisualizer;
using mcr::visualization::ClusteredPointCloudVisualizer;
using mcr::visualization::LabelVisualizer;
using mas_perception_libs::Color;

/**
 * This node subscribes to pointcloud topic.
 * Inputs:
 * ~event_in:
 *      - e_start: starts subscribing to pointcloud topic
 *      - e_add_cloud_start: adds pointcloud to octree, if it is on dataset collection mode,
 *                           the node will start segmenting the pointcloud.
 *      - e_add_cloud_stop: stops adding pointcloud to octree
 *      - e_find_plane: finds the plane and publishes workspace height
 *      - e_segment: starts segmentation and publish ObjectList
 *      - e_reset: clears accumulated cloud
 *      - e_stop: stops subscribing and clears accumulated pointcloud
 * Outputs:
 * ~event_out:
 *      - e_started: started listening to new messages
 *      - e_add_cloud_started: started adding the cloud to octree
 *      - e_add_cloud_stopped: stopped adding the cloud to octree
 *      - e_done: started finding the plane or started segmenting the pointcloud
 *      - e_stopped: stopped subscribing and cleared accumulated pointcloud
 */

class SceneSegmentationNode
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_debug_;
        ros::Publisher pub_boxes_;
        ros::Publisher pub_object_list_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_workspace_height_;
        ros::Publisher pub_input_for_debug_;

        ros::Subscriber sub_cloud_;
        ros::Subscriber sub_event_in_;

        ros::ServiceClient recognize_service;

        dynamic_reconfigure::Server<mcr_scene_segmentation::SceneSegmentationConfig> server_;

        tf::TransformListener transform_listener_;

        SceneSegmentation scene_segmentation_;
        CloudAccumulation::UPtr cloud_accumulation_;

        BoundingBoxVisualizer bounding_box_visualizer_;
        ClusteredPointCloudVisualizer cluster_visualizer_;
        LabelVisualizer label_visualizer_;

        bool add_to_octree_;
        std::string frame_id_;
        int object_id_;
        double octree_resolution_;
        double object_height_above_workspace_;
        bool dataset_collection_;
        bool debug_mode_;
        std::string logdir_;

    private:
        void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);
        void eventCallback(const std_msgs::String::ConstPtr &msg);
        void configCallback(mcr_scene_segmentation::SceneSegmentationConfig &config, uint32_t level);
        void segment();
        void findPlane();
        geometry_msgs::PoseStamped getPose(const mas_perception_libs::BoundingBox &box);
        void savePcd(const PointCloud::ConstPtr &cloud, std::string obj_name);

    public:
        SceneSegmentationNode();
        virtual ~SceneSegmentationNode();
};

#endif  // MCR_SCENE_SEGMENTATION_SCENE_SEGMENTATION_NODE_H
