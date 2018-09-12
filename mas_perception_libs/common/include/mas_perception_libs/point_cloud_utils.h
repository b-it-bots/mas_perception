/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 * @brief Header file for point cloud utility functions
 */
#ifndef MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H
#define MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H

#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <mas_perception_libs/aliases.h>
#include <mas_perception_libs/bounding_box_2d.h>

namespace mas_perception_libs
{

/*!
 * @brief extract a CV matrix containing (x, y, z) coordinates from a pcl::PointCloud object within a region
 *        defined by a BoundingBox2D object
 * @param pBox: bounding box for cropping cloud, may be adjusted to fit the cloud width and height
 */
cv::Mat
cropCloudToXYZ(const PointCloud &pCloud, BoundingBox2D &pBox);

/*!
 * @brief crops a pcl::PointCloud object to a region defined by a BoundingBox2D object
 * @param pBox: bounding box for cropping cloud, may be adjusted to fit the cloud width and height
 */
PointCloud
cropOrganizedCloud(const PointCloud &pCloud, BoundingBox2D &pBox);

/*!
 * @brief struct containing parameters necessary for filtering point clouds
 */
struct CloudFilterParams
{
    /* TODO(minhnh) describe fields */
    /* VoxelGrid filter parameters */
    float mVoxelLeafSize;
    std::string mVoxelFilterFieldName;
    double mVoxelLimitMin;
    double mVoxelLimitMax;
    /* PassThrough filter parameters */
    std::string mPassThroughFieldName;
    float mPassThroughLimitMin;
    float mPassThroughLimitMax;
};

/*!
 * @brief class containing definition for filtering point clouds
 */
class CloudFilter
{
public:
    CloudFilter() = default;

    /*! @brief set parameters relevant to filtering cloud */
    void setParams(const CloudFilterParams& pParams);

    /*!
    * @brief filter point cloud
    */
    PointCloud::Ptr filterCloud(const PointCloud::ConstPtr &pCloud);

private:
    pcl::PassThrough<PointT> mPassThroughFilter;
    pcl::VoxelGrid<PointT> mVoxelGridFilter;
};

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H
