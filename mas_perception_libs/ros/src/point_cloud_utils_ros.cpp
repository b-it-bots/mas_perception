/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <string>
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

visualization_msgs::Marker::Ptr
planeMsgToMarkers(const mcr_perception_msgs::Plane &pPlaneMsg, const std::string &pNamespace,
                  Color pColor, float pThickness, int pId)
{
    if (pPlaneMsg.convex_hull.empty())
        throw std::invalid_argument("plane message has empty convex hull");

    auto markerPtr = boost::make_shared<visualization_msgs::Marker>();

    markerPtr->type = visualization_msgs::Marker::LINE_LIST;
    markerPtr->action = visualization_msgs::Marker::ADD;
    markerPtr->lifetime = ros::Duration(2.0);
    markerPtr->header.frame_id = pPlaneMsg.header.frame_id;
    markerPtr->scale.x = pThickness;
    markerPtr->scale.y = pThickness;
    markerPtr->color.a = 2.0;
    markerPtr->ns = pNamespace;
    markerPtr->id = pId;
    markerPtr->color = std_msgs::ColorRGBA(pColor);

    geometry_msgs::Point firstPoint;
    firstPoint.x = pPlaneMsg.convex_hull[0].x;
    firstPoint.y = pPlaneMsg.convex_hull[0].y;
    firstPoint.z = pPlaneMsg.convex_hull[0].z;
    markerPtr->points.push_back(firstPoint);

    for (size_t i = 1; i < pPlaneMsg.convex_hull.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = pPlaneMsg.convex_hull[i].x;
        pt.y = pPlaneMsg.convex_hull[i].y;
        pt.z = pPlaneMsg.convex_hull[i].z;
        markerPtr->points.push_back(pt);
        markerPtr->points.push_back(pt);
    }

    markerPtr->points.push_back(firstPoint);
    return markerPtr;
}


mcr_perception_msgs::Plane::Ptr
planeModelToMsg(const PlaneModel &pModel)
{
    auto planeMsgPtr = boost::make_shared<mcr_perception_msgs::Plane>();
    planeMsgPtr->header = pcl_conversions::fromPCL(pModel.mHeader);

    // plane coefficients
    planeMsgPtr->coefficients[0] = pModel.mCoefficients[0];
    planeMsgPtr->coefficients[1] = pModel.mCoefficients[1];
    planeMsgPtr->coefficients[2] = pModel.mCoefficients[2];
    planeMsgPtr->coefficients[3] = pModel.mCoefficients[3];

    // plane center point
    planeMsgPtr->plane_point.x = pModel.mCenter.x;
    planeMsgPtr->plane_point.y = pModel.mCenter.y;
    planeMsgPtr->plane_point.z = pModel.mCenter.z;

    // convex hull points
    for (auto& hullPoint : pModel.mHullPointsPtr->points)
    {
        geometry_msgs::Point32 hullPointMsg;
        hullPointMsg.x = hullPoint.x;
        hullPointMsg.y = hullPoint.y;
        hullPointMsg.z = hullPoint.z;
        planeMsgPtr->convex_hull.push_back(hullPointMsg);
    }
    return planeMsgPtr;
}

void
PlaneSegmenterROS::setParams(const PlaneFittingConfig &pConfig)
{
    CloudFilterParams cloudFilterParams;
    cloudFilterParams.mPassThroughLimitMinX = static_cast<float>(pConfig.passthrough_limit_min_x);
    cloudFilterParams.mPassThroughLimitMaxX = static_cast<float>(pConfig.passthrough_limit_max_x);
    cloudFilterParams.mPassThroughLimitMinY = static_cast<float>(pConfig.passthrough_limit_min_y);
    cloudFilterParams.mPassThroughLimitMaxY = static_cast<float>(pConfig.passthrough_limit_max_y);
    cloudFilterParams.mPassThroughLimitMinZ = static_cast<float>(pConfig.passthrough_limit_min_z);
    cloudFilterParams.mPassThroughLimitMaxZ = static_cast<float>(pConfig.passthrough_limit_max_z);
    cloudFilterParams.mVoxelLeafSize = static_cast<float>(pConfig.voxel_leaf_size);
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

mcr_perception_msgs::PlaneList::Ptr
PlaneSegmenterROS::findPlanes(const sensor_msgs::PointCloud2::ConstPtr &pCloudPtr,
                              sensor_msgs::PointCloud2::Ptr &pFilteredCloudMsgPtr)
{
    auto pclCloudPtr = boost::make_shared<PointCloud>();
    pcl::fromROSMsg(*pCloudPtr, *pclCloudPtr);
    auto filteredCloudPtr = mCloudFilter.filterCloud(pclCloudPtr);
    pcl::toROSMsg(*filteredCloudPtr, *pFilteredCloudMsgPtr);

    auto planeModel = mPlaneSegmenter.findPlane(filteredCloudPtr);
    auto planeListPtr = boost::make_shared<mcr_perception_msgs::PlaneList>();
    auto planeMsgPtr = planeModelToMsg(planeModel);
    planeListPtr->planes.push_back(*planeMsgPtr);
    return planeListPtr;
}

}   // namespace mas_perception_libs
