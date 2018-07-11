/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mas_perception_libs/point_cloud_utils.h>

namespace mas_perception_libs
{

cv::Mat
cloudMsgToCvImage(sensor_msgs::PointCloud2 &pCloudMsg)
{
    // check for organized cloud and extract image message
    if (pCloudMsg.height <= 1)
    {
        throw std::invalid_argument("Input point cloud is not organized!");
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

}   // namespace mas_perception_libs
