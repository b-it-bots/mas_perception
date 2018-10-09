/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Minh Nguyen
 * @author Santosh Thoduka
 * @author Mohammad Wasil
 *
 * @brief Header file for sac_plane_model.cpp
 */
#ifndef MAS_PERCEPTION_LIBS_SAC_PLANE_SEGMENTER_H
#define MAS_PERCEPTION_LIBS_SAC_PLANE_SEGMENTER_H

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <mas_perception_libs/aliases.h>

namespace mas_perception_libs
{

/*!
 * @brief structure containing the parameters necessary for plane segmentation
 * TODO(minhnh) add field documentation
 */
struct SacPlaneSegmenterParams
{
    double mNormalRadiusSearch = 0.0;

    int mSacMaxIterations = 0;
    double mSacDistThreshold = 0.0;
    bool mSacOptimizeCoeffs = false;
    double mSacEpsAngle = 0.0;
    double mSacNormalDistWeight = 0.0;
};

struct PlaneModel
{
public:
    explicit PlaneModel(pcl::ModelCoefficients::Ptr pPlaneCoeffsPtr);
    ~PlaneModel() = default;

public:
    const Eigen::Vector3f mNormal;
    PointCloud::Ptr mHullPointsPtr;
    double mPlaneHeight = 0.0;
};

/*!
 * @brief use a sample consensus (SAC, or in this case RANSAC) algorithm to fit a plane in point clouds
 *        see http://docs.pointclouds.org/1.7.0/group__sample__consensus.html for descriptions of the different
 *        SAC algorithms and model types.
 */
class SacPlaneSegmenter
{
public:
    SacPlaneSegmenter() : mSacModel(pcl::SACMODEL_PARALLEL_PLANE), mPlaneAxis(0.0f, 0.0f, 1.0f)
    { }

    /*! @brief set parameters of the SAC algorithm */
    void setParams(const SacPlaneSegmenterParams &pPlaneParams);

    /*!
     * @brief fit a plane from a point cloud and extract normal, plane height
     * TODO(minhnh) add parameter documentation
     */
    PlaneModel
    findPlane(const PointCloud::ConstPtr &pCloudPtr);

private:
    void
    findPlaneInliers(const PointCloud::ConstPtr &pCloudPtr, pcl::PointIndices::Ptr &pInliers,
                     pcl::ModelCoefficients::Ptr &pCoefficients);

    void
    findConvexHull(const PointCloud::ConstPtr &pCloudPtr, const pcl::PointIndices::Ptr &pInlierIndicesPtr,
                   const pcl::ModelCoefficients::Ptr &pCoefficientsPtr, PointCloud::Ptr &pHullPtr,
                   double &pPlaneHeight);

private:
    const int cSacMethodType = pcl::SAC_RANSAC;

    pcl::NormalEstimation<PointT, pcl::PointNormal> mNormalEstimation;
    pcl::SACSegmentationFromNormals<PointT, pcl::PointNormal> mSacSegment;

    pcl::SacModel mSacModel;
    Eigen::Vector3f mPlaneAxis;
};

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_SAC_PLANE_SEGMENTER_H
