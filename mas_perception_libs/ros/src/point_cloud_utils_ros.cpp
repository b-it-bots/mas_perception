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

sensor_msgs::PointCloud2::Ptr
PlaneSegmenterROS::filterCloud(const sensor_msgs::PointCloud2::ConstPtr &pCloudPtr)
{
    PointCloud::Ptr pclCloudPtr = boost::make_shared<PointCloud>();
    pcl::fromROSMsg(*pCloudPtr, *pclCloudPtr);

    PointCloud::Ptr filteredCloudPtr = mCloudFilter.filterCloud(pclCloudPtr);

    sensor_msgs::PointCloud2::Ptr filteredMsgPtr = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*filteredCloudPtr, *filteredMsgPtr);
    return filteredMsgPtr;
}

mcr_perception_msgs::Plane
planeModelToMsg(const PlaneModel &pModel)
{
    mcr_perception_msgs::Plane planeMsg;
    // plane normal
    planeMsg.header = pcl_conversions::fromPCL(pModel.mHeader);
    planeMsg.coefficients[0] = pModel.mCoefficients[0];
    planeMsg.coefficients[1] = pModel.mCoefficients[1];
    planeMsg.coefficients[2] = pModel.mCoefficients[2];
    planeMsg.coefficients[3] = pModel.mCoefficients[3];
    // plane pose, assuming horizontal plane and z is plane height
    planeMsg.plane_point.x = pModel.mCenter.x;
    planeMsg.plane_point.y = pModel.mCenter.y;
    planeMsg.plane_point.z = pModel.mCenter.z;
    for (auto& hullPoint : pModel.mHullPointsPtr->points)
    {
        geometry_msgs::Point32 hullPointMsg;
        hullPointMsg.x = hullPoint.x;
        hullPointMsg.y = hullPoint.y;
        hullPointMsg.z = hullPoint.z;
        planeMsg.convex_hull.push_back(hullPointMsg);
    }
    return planeMsg;
}

void
PlaneSegmenterROS::setParams(const PlaneFittingConfig &pConfig)
{
    CloudFilterParams cloudFilterParams;
    cloudFilterParams.mVoxelLeafSize = static_cast<float>(pConfig.voxel_leaf_size);
    cloudFilterParams.mPassThroughFieldName = pConfig.passthrough_filter_field_name;
    cloudFilterParams.mPassThroughLimitMin = static_cast<float>(pConfig.passthrough_filter_limit_min);
    cloudFilterParams.mPassThroughLimitMax = static_cast<float>(pConfig.passthrough_filter_limit_max);
    mCloudFilter.setParams(cloudFilterParams);

    SacPlaneSegmenterParams planeFitParams;
    planeFitParams.mNormalRadiusSearch = pConfig.normal_radius_search;
    planeFitParams.mSacMaxIterations = pConfig.sac_max_iterations;
    planeFitParams.mSacDistThreshold = pConfig.sac_distance_threshold;
    planeFitParams.mSacOptimizeCoeffs = pConfig.sac_optimize_coefficients;
    planeFitParams.mSacEpsAngle = pConfig.sac_eps_angle;
    planeFitParams.mSacNormalDistWeight = pConfig.sac_normal_distance_weight;
    mPlaneSegmenter.setParams(planeFitParams);
}

mcr_perception_msgs::PlaneList
PlaneSegmenterROS::findPlanes(const sensor_msgs::PointCloud2::ConstPtr &pCloudPtr,
                              sensor_msgs::PointCloud2::Ptr &pFilteredCloudMsgPtr)
{
    auto pclCloudPtr = boost::make_shared<PointCloud>();
    pcl::fromROSMsg(*pCloudPtr, *pclCloudPtr);
    auto filteredCloudPtr = mCloudFilter.filterCloud(pclCloudPtr);
    pcl::toROSMsg(*filteredCloudPtr, *pFilteredCloudMsgPtr);

    auto planeModel = mPlaneSegmenter.findPlane(filteredCloudPtr);
    mcr_perception_msgs::PlaneList planeList;
    planeList.planes.push_back(planeModelToMsg(planeModel));
    return planeList;
}

}   // namespace mas_perception_libs
