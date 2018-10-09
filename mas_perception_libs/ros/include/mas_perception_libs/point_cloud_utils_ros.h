/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 */
#ifndef MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_ROS_H
#define MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_ROS_H

#include <opencv/cv.h>
#include <sensor_msgs/PointCloud2.h>
#include <mas_perception_libs/PlaneFittingConfig.h>
#include <mas_perception_libs/bounding_box_2d.h>
#include <mas_perception_libs/point_cloud_utils.h>
#include <mas_perception_libs/sac_plane_segmenter.h>

namespace mas_perception_libs
{
/*!
 * @brief extract a CV image from a sensor_msgs/PointCloud2 object
 */
cv::Mat
cloudMsgToCvImage(const sensor_msgs::PointCloud2 &pCloudMsg);

/*!
 * @brief extract a CV matrix containing (x, y, z) coordinates from a sensor_msgs/PointCloud2 object within a region
 *        defined by a BoundingBox2D object
 * @param pBox: bounding box for cropping cloud, may be adjusted to fit the cloud width and height
 */
cv::Mat
cropCloudMsgToXYZ(const sensor_msgs::PointCloud2 &pCloudMsg, BoundingBox2D &pBox);

/*!
 * @brief crops a sensor_msgs/PointCloud2 object object to a region defined by a BoundingBox2D object
 * @param pBox: bounding box for cropping cloud, may be adjusted to fit the cloud width and height
 * @param pCroppedCloudMsg: cropped cloud returned as parameter
 */
void
cropOrganizedCloudMsg(const sensor_msgs::PointCloud2 &pCloudMsg, BoundingBox2D &pBox,
                      sensor_msgs::PointCloud2& pCroppedCloudMsg);

class PlaneSegmenterROS
{
public:
    PlaneSegmenterROS() = default;

    virtual void
    setParams(const PlaneFittingConfig &pConfig);

    virtual sensor_msgs::PointCloud2::Ptr
    filterCloud(const sensor_msgs::PointCloud2::ConstPtr &pCloudPtr);

    virtual void
    findPlane(const sensor_msgs::PointCloud2::ConstPtr &pCloudPtr, PointCloud::Ptr &pHullPtr,
              pcl::ModelCoefficients::Ptr &pCoefficientsPtr, double &pPlaneHeight);

protected:
    CloudFilter mCloudFilter;
    SacPlaneSegmenter mPlaneSegmenter;
};

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_ROS_H
