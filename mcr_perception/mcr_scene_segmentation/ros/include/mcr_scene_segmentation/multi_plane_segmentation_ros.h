#ifndef MCR_SCENE_SEGMENTATION_MUlTI_PLANE_SEGMENTATION_ROS_H
#define MCR_SCENE_SEGMENTATION_MUlTI_PLANE_SEGMENTATION_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <vector>

class MultiPlaneSegmentationRos
{
public:
    MultiPlaneSegmentationRos();

    virtual ~MultiPlaneSegmentationRos();

    void update();

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);
    void extract_plane();
    void cropPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pointcloud, std::vector<int> &indices);

private:
    /**
     * Subscribers
     */
    ros::Subscriber sub_pointcloud_;

    /**
     * Publishers
     */
    ros::Publisher pub_polygon_;
    ros::Publisher pub_pointcloud_;
    ros::Publisher marker_publisher_;
    ros::Publisher normal_marker_pub_;

    ros::Publisher pub_polygon_vector_;
    ros::Publisher pub_model_coefficients_vector_;

    /**
     * Used to store pointcloud message
     */
    sensor_msgs::PointCloud2::Ptr pointcloud_msg_;

    //pcl::IntegralImageNormalEstimation<PointT, PointNT> ne_;
    //pcl::OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT> mps_;
    //pcl::ProjectInliers<PointT> pi_;

    bool pointcloud_msg_received_;
    bool sort_by_area_;
    bool enable_region_visualization_;

    Eigen::Vector3f z_axis;
    float angular_threshold_;
    float min_distance_to_workspace_;
    float workspace_area_threshold_;

};

#endif
