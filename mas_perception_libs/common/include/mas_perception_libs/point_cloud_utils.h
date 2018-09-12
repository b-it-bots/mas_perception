/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 * @brief Header file for point cloud utility functions
 */
#ifndef MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H
#define MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H

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

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_POINT_CLOUD_UTILS_H
