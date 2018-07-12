/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#ifndef MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H
#define MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H

#include <opencv/cv.h>
#include <sensor_msgs/PointCloud2.h>

namespace mas_perception_libs
{

    cv::Mat
    cloudMsgToCvImage(sensor_msgs::PointCloud2 &pCloudMsg);

    PointCloud
    cropOrganizedCloud(const PointCloud &pCloud, BoundingBox2D &pBox);

    void
    cropOrganizedCloudMsg(const sensor_msgs::PointCloud2 &pCloudMsg, BoundingBox2D &pBox,
                          sensor_msgs::PointCloud2& pCroppedCloudMsg);

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H
