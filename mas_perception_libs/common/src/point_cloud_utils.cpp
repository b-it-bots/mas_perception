/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 *
 * @brief File contains C++ definitions for processing point clouds
 */
#include <mas_perception_libs/point_cloud_utils.h>

namespace mas_perception_libs
{

    cv::Mat
    cropCloudToXYZ(const PointCloud &pCloud, BoundingBox2D &pBox)
    {
        fitBoxToImage(cv::Size(pCloud.width, pCloud.height), pBox);

        // create coords
        int dims[] = { pBox.mWidth, pBox.mHeight, 3 };
        cv::Mat coordinates(3, dims, CV_32FC1);
        for (int x = pBox.mX; x < pBox.mX + pBox.mWidth; x++)
        {
            for (int y = pBox.mY; y < pBox.mY + pBox.mHeight; y++)
            {
                const PointT& origPoint = pCloud.at(x, y);
                coordinates.at<float>(x - pBox.mX, y - pBox.mY, 0) = origPoint.x;
                coordinates.at<float>(x - pBox.mX, y - pBox.mY, 1) = origPoint.y;
                coordinates.at<float>(x - pBox.mX, y - pBox.mY, 2) = origPoint.z;
            }
        }
        return coordinates;
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
    CloudFilter::setParams(const mas_perception_libs::CloudFilterParams &pParams)
    {
        /* voxel-grid params */
        mVoxelGridFilter.setLeafSize(pParams.mVoxelLeafSize, pParams.mVoxelLeafSize, pParams.mVoxelLeafSize);
        mVoxelGridFilter.setFilterFieldName(pParams.mVoxelFilterFieldName);
        mVoxelGridFilter.setFilterLimits(pParams.mVoxelLimitMin, pParams.mVoxelLimitMax);
        /* pass-through params */
        mPassThroughFilter.setFilterFieldName(pParams.mPassThroughFieldName);
        mPassThroughFilter.setFilterLimits(pParams.mPassThroughLimitMin, pParams.mPassThroughLimitMax);
    }


    PointCloud::Ptr
    CloudFilter::filterCloud(const PointCloud::ConstPtr &pCloud)
    {
        PointCloud::Ptr filtered(new PointCloud);

        mVoxelGridFilter.setInputCloud(pCloud);
        mVoxelGridFilter.filter(*filtered);

        mPassThroughFilter.setInputCloud(filtered);
        mPassThroughFilter.filter(*filtered);

        return filtered;
    }

}   // namespace mas_perception_libs
