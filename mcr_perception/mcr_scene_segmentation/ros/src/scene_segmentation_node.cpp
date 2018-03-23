#include <mcr_scene_segmentation/scene_segmentation.h>
#include <mcr_scene_segmentation/scene_segmentation_node.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>

#include <mcr_perception_msgs/BoundingBox.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/RecognizeObject.h>
#include "mcr_scene_segmentation/impl/helpers.hpp"
#include "mcr_scene_segmentation/bounding_box.h"

#include <Eigen/Dense>

#include "pcl_ros/point_cloud.h"

SceneSegmentationNode::SceneSegmentationNode(): nh_("~"), bounding_box_visualizer_("bounding_boxes", Color(Color::SEA_GREEN)),
                                                          cluster_visualizer_("tabletop_clusters"), label_visualizer_("labels", mcr::visualization::Color(mcr::visualization::Color::TEAL)),
                                                          add_to_octree_(false), object_id_(0)
{
    pub_debug_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_object_list_ = nh_.advertise<mcr_perception_msgs::ObjectList>("object_list", 1);
    sub_event_in_ = nh_.subscribe("event_in", 1, &SceneSegmentationNode::eventCallback, this);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);

    dynamic_reconfigure::Server<mcr_scene_segmentation::SceneSegmentationConfig>::CallbackType f =
                            boost::bind(&SceneSegmentationNode::config_callback, this, _1, _2);
    server_.setCallback(f);

    recognize_service = nh_.serviceClient<mcr_perception_msgs::RecognizeObject>("/mcr_perception/object_recognizer/recognize_object");
    recognize_service.waitForExistence(ros::Duration(5));
    if (!recognize_service.exists())
    {
        ROS_WARN("Object recognition service is not available. Will return 'unknown' for all objects");
    }

    nh_.param("octree_resolution", octree_resolution_, 0.05);
    cloud_accumulation_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));
}

SceneSegmentationNode::~SceneSegmentationNode()
{
}


void SceneSegmentationNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (add_to_octree_)
    {
        PointCloud::Ptr cloud(new PointCloud);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*msg, pc2);
        pcl::fromPCLPointCloud2(pc2, *cloud);

        cloud_accumulation_->addCloud(cloud);

        frame_id_ = msg->header.frame_id;

		std_msgs::String event_out;
        add_to_octree_ = false;
        event_out.data = "e_add_cloud_stopped";
        pub_event_out_.publish(event_out);
    }

}

void SceneSegmentationNode::segment()
{
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = frame_id_;
    cloud_accumulation_->getAccumulatedCloud(*cloud);

    std::vector<PointCloud::Ptr> clusters;
    std::vector<BoundingBox> boxes;
    PointCloud::Ptr debug = scene_segmentation_.segment_scene(cloud, clusters, boxes);
    pub_debug_.publish(*debug);

    mcr_perception_msgs::BoundingBoxList bounding_boxes;
    mcr_perception_msgs::ObjectList object_list;
    geometry_msgs::PoseArray poses;


    bounding_boxes.bounding_boxes.resize(boxes.size());
    object_list.objects.resize(boxes.size());

    std::vector<std::string> labels;

    ros::Time now = ros::Time::now();
    for (int i = 0; i < boxes.size(); i++)
    {
        convertBoundingBox(boxes[i], bounding_boxes.bounding_boxes[i]);

        sensor_msgs::PointCloud2 ros_cloud;
        pcl::PCLPointCloud2 pc2;
        pcl::toPCLPointCloud2(*clusters[i], pc2);
        pcl_conversions::fromPCL(pc2, ros_cloud);

        if (recognize_service.exists())
        {
            mcr_perception_msgs::RecognizeObject srv;
            srv.request.cloud = ros_cloud;
            srv.request.dimensions = bounding_boxes.bounding_boxes[i].dimensions;
            if (recognize_service.call(srv))
            {
                object_list.objects[i].name = srv.response.name;
                object_list.objects[i].probability = srv.response.probability;
            }
            else
            {
                ROS_WARN("Object recognition service call failed");
                object_list.objects[i].name = "unknown";
                object_list.objects[i].probability = 0.0;
            }
        }
        else
        {
            object_list.objects[i].name = "unknown";
            object_list.objects[i].probability = 0.0;
        }
        labels.push_back(object_list.objects[i].name);

        geometry_msgs::PoseStamped pose = getPose(boxes[i]);
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_;


        std::string target_frame_id;
        if (nh_.hasParam("target_frame_id"))
        {
            nh_.param("target_frame_id", target_frame_id, frame_id_);
            if (target_frame_id != frame_id_)
            {
                try
                {
                    ros::Time common_time;
                    transform_listener_.getLatestCommonTime(frame_id_, target_frame_id, common_time, NULL);
                    pose.header.stamp = common_time;
                    transform_listener_.waitForTransform(target_frame_id, frame_id_, common_time, ros::Duration(0.1));
                    geometry_msgs::PoseStamped pose_transformed;
                    transform_listener_.transformPose(target_frame_id, pose, pose_transformed);
                    object_list.objects[i].pose = pose_transformed;
                }
                catch(tf::LookupException& ex)
                {
                    ROS_WARN("Failed to transform pose: (%s)", ex.what());
                    pose.header.stamp = now;
                    object_list.objects[i].pose = pose;
                }
            }
            else
            {
                object_list.objects[i].pose = pose;
            }
        }
        else
        {
            object_list.objects[i].pose = pose;
        }
        poses.poses.push_back(object_list.objects[i].pose.pose);
        poses.header = object_list.objects[i].pose.header;

        object_list.objects[i].database_id = object_id_;
        object_id_++;
    }
    pub_object_list_.publish(object_list);
    bounding_box_visualizer_.publish(bounding_boxes.bounding_boxes, frame_id_);
    cluster_visualizer_.publish<PointT>(clusters, frame_id_);
    label_visualizer_.publish(labels, poses);
}

geometry_msgs::PoseStamped SceneSegmentationNode::getPose(const BoundingBox &box)
{
    BoundingBox::Points vertices = box.getVertices();
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();
    if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm())
    {
        n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
    }
    else
    {
        n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
    }
    n2 = n3.cross(n1);
    ROS_INFO_STREAM("got norms");
    Eigen::Matrix3f m;
    m << n1 , n2 , n3;
    Eigen::Quaternion<float> q(m);
    q.normalize();


    Eigen::Vector3f centroid = box.getCenter();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = centroid(0);
    pose.pose.position.y = centroid(1);
    pose.pose.position.z = centroid(2);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

void SceneSegmentationNode::eventCallback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String event_out;
    if(msg->data == "e_start")
    {
        sub_cloud_ = nh_.subscribe("input", 1, &SceneSegmentationNode::pointcloudCallback, this);
		event_out.data = "e_started";
	}
    else if(msg->data == "e_add_cloud_start")
    {
        add_to_octree_ = true;
        // Not needed so that not to affect the action server
        return;
    }
    else if(msg->data == "e_add_cloud_stop")
    {
        add_to_octree_ = false;
        event_out.data = "e_add_cloud_stopped";
    }
    else if(msg->data == "e_segment")
    {
        segment();
        event_out.data = "e_done";
    }
    else if (msg->data == "e_reset")
    {
        cloud_accumulation_->reset();
        event_out.data = "e_reset";
    }
    else if(msg->data == "e_stop")
    {
		sub_cloud_.shutdown();
		event_out.data = "e_stopped";
	}
    else
    {
        return;
    }
	pub_event_out_.publish(event_out);

}

void SceneSegmentationNode::config_callback(mcr_scene_segmentation::SceneSegmentationConfig &config, uint32_t level)
{
    scene_segmentation_.setVoxelGridParams(config.voxel_leaf_size, config.voxel_filter_field_name, config.voxel_filter_limit_min, config.voxel_filter_limit_max);
    scene_segmentation_.setPassthroughParams(config.passthrough_filter_field_name, config.passthrough_filter_limit_min, config.passthrough_filter_limit_max);
    scene_segmentation_.setNormalParams(config.normal_radius_search);
    scene_segmentation_.setSACParams(config.sac_max_iterations, config.sac_distance_threshold, config.sac_optimize_coefficients, config.sac_eps_angle, config.sac_normal_distance_weight);
    scene_segmentation_.setPrismParams(config.prism_min_height, config.prism_max_height);
    scene_segmentation_.setOutlierParams(config.outlier_radius_search, config.outlier_min_neighbors);
    scene_segmentation_.setClusterParams(config.cluster_tolerance, config.cluster_min_size, config.cluster_max_size, config.cluster_min_height, config.cluster_max_height, config.cluster_max_length, config.cluster_min_distance_to_polygon);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scene_segmentation_node");
    SceneSegmentationNode scene_seg;
    ros::spin();
    return 0;
}

