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

namespace mas_perception_libs
{

    cv::Mat
    cloudMsgToCvImage(sensor_msgs::PointCloud2 &pCloudMsg)
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

    PointCloud
    cropOrganizedCloud(const PointCloud &pCloud, BoundingBox2D &pBox)
    {
        if (!pCloud.isOrganized())
            throw std::runtime_error("input cloud is not organized");

        fitBoxToImage(cv::Size(pCloud.width, pCloud.height), pBox);

        PointCloud croppedCloud(static_cast<uint32_t>(pBox.mWidth), static_cast<uint32_t>(pBox.mHeight));
        croppedCloud.header = pCloud.header;
        for (int x = pBox.mX; x < pBox.mX + pBox.mWidth; x++)
        {
            for (int y = pBox.mY; y < pBox.mY + pBox.mHeight; y++)
            {
                PointT& croppedPoint = croppedCloud.at(x - pBox.mX, y - pBox.mY);
                const PointT& origPoint = pCloud.at(x, y);
                croppedPoint.x = origPoint.x;
                croppedPoint.y = origPoint.y;
                croppedPoint.z = origPoint.z;
                croppedPoint.r = origPoint.r;
                croppedPoint.g = origPoint.g;
                croppedPoint.b = origPoint.b;
            }
        }
        return croppedCloud;
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

}   // namespace mas_perception_libs
