/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 * @author Santosh Thoduka
 * @author Mohammad Wasil
 *
 * @brief File contains definitions for extracting a plane from a point cloud
 */
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <mas_perception_libs/sac_plane_segmenter.h>

namespace mas_perception_libs
{

    void
    SacPlaneSegmenter::setParams(const SacPlaneSegmenterParams &pPlaneParams)
    {
        mNormalEstimation.setRadiusSearch(pPlaneParams.mNormalRadiusSearch);

        mSacSegment.setMaxIterations(pPlaneParams.mSacMaxIterations);
        mSacSegment.setDistanceThreshold(pPlaneParams.mSacDistThreshold);
        mSacSegment.setEpsAngle(pPlaneParams.mSacEpsAngle);
        mSacSegment.setOptimizeCoefficients(pPlaneParams.mSacOptimizeCoeffs);
        mSacSegment.setNormalDistanceWeight(pPlaneParams.mSacNormalDistWeight);
    }

    void
    SacPlaneSegmenter::findPlane(const PointCloud::ConstPtr &pCloudPtr, PointCloud::Ptr &pHullPtr,
            pcl::ModelCoefficients::Ptr &pCoefficients, double &pPlaneHeight)
    {
        // estimate normals at each point, needed for the SAC segmentation
        PointCloudN::Ptr normals(new PointCloudN);
        mNormalEstimation.setInputCloud(pCloudPtr);
        mNormalEstimation.compute(*normals);

        // run the SAC segmentation algorithm
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        mSacSegment.setModelType(mSacModel);
        mSacSegment.setMethodType(cSacMethodType);
        mSacSegment.setAxis(mPlaneAxis);
        mSacSegment.setInputCloud(pCloudPtr);
        mSacSegment.setInputNormals(normals);
        mSacSegment.segment(*inliers, *pCoefficients);
        if (inliers->indices.empty())
        {
            return;
        }

        // filter out al points that does not fit the calculated plane coefficients
        pcl::ProjectInliers<PointT> projectInliers;
        PointCloud::Ptr plane(new PointCloud);
        projectInliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
        projectInliers.setInputCloud(pCloudPtr);
        projectInliers.setModelCoefficients(pCoefficients);
        projectInliers.setIndices(inliers);
        projectInliers.setCopyAllData(false);
        projectInliers.filter(*plane);

        // calculate the convex hull of the fitted plane
        pcl::ConvexHull<PointT> convexHull;
        convexHull.setInputCloud(plane);
        convexHull.reconstruct(*pHullPtr);
        // connect end point and first point, not sure if this is necessary TODO(minhnh) test this
        pHullPtr->points.push_back(pHullPtr->points.front());
        pHullPtr->width += 1;

        // calculate plane height as the average of convex hull points
        double z = 0.0;
        for (auto &point : pHullPtr->points)
        {
            z += point.z;
        }
        if (!pHullPtr->points.empty())
        {
            z /= pHullPtr->points.size();
        }
        pPlaneHeight = z;
    }
}   // namespace mas_perception_libs
