/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mas_perception_libs/aliases.h>
#include <mas_perception_libs/bounding_box_2d.h>
#include <mas_perception_libs/point_cloud_utils.h>
#include <mas_perception_libs/point_cloud_utils_ros.h>

namespace mas_perception_libs
{
    cv::Mat
    cloudMsgToCvImage(const sensor_msgs::PointCloud2 &pCloudMsg)
    {
        // check for organized cloud and extract image message
        if (pCloudMsg.height <= 1)
        {
            throw std::runtime_error("Input point cloud is not organized!");
        }
        sensor_msgs::Image imageMsg;
        pcl::toROSMsg(pCloudMsg, imageMsg);

        // convert to OpenCV image
        cv_bridge::CvImagePtr cvImagePtr;
        try
        {
            cvImagePtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            std::ostringstream msgStream;
            msgStream << "cv_bridge exception: " << e.what();
            throw std::runtime_error(msgStream.str());
        }

        return cvImagePtr->image;
    }

    cv::Mat
    cropCloudMsgToXYZ(const sensor_msgs::PointCloud2 &pCloudMsg, BoundingBox2D &pBox)
    {
        // convert to PCL cloud
        PointCloud origCloud;
        pcl::fromROSMsg(pCloudMsg, origCloud);

        return cropCloudToXYZ(origCloud, pBox);
    }

    void
    cropOrganizedCloudMsg(const sensor_msgs::PointCloud2 &pCloudMsg, BoundingBox2D &pBox,
                          sensor_msgs::PointCloud2& pCroppedCloudMsg)
    {
        // check for organized cloud and extract image message
        if (pCloudMsg.height <= 1)
            throw std::runtime_error("Input point cloud is not organized!");

        // convert to PCL cloud
        PointCloud origCloud;
        pcl::fromROSMsg(pCloudMsg, origCloud);

        // crop and convert back to ROS message
        PointCloud croppedCloud = cropOrganizedCloud(origCloud, pBox);
        pcl::toROSMsg(croppedCloud, pCroppedCloudMsg);
    }

    CloudFilterParams
    cloudFilterConfigToParam(const CloudFilterConfig& pConfig)
    {
        CloudFilterParams params;
        params.mVoxelLeafSize = static_cast<float>(pConfig.voxel_leaf_size);
        params.mPassThroughFieldName = pConfig.passthrough_filter_field_name;
        params.mPassThroughLimitMin = static_cast<float>(pConfig.passthrough_filter_limit_min);
        params.mPassThroughLimitMax = static_cast<float>(pConfig.passthrough_filter_limit_max);
        return params;
    }

}   // namespace mas_perception_libs
