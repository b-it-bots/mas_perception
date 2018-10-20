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

#include <mas_perception_libs/aliases.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

namespace mas_perception_libs
{

/*!
 * @brief structure containing the parameters necessary for plane segmentation
 */
struct SacPlaneSegmenterParams
{
    /* parameter for pcl::NormalEstimation */
    double mNormalRadiusSearch = 0.0;

    /* parameters for pcl::SACSegmentationFromNormals, see parameter descriptions at
     * http://docs.pointclouds.org/1.7.0/sac__segmentation_8h_source.html#l00262  */
    int mSacMaxIterations = 0;              // limit on number of iteration for the SAC algorithm
    double mSacDistThreshold = 0.0;         // outlier threshold for the SAC algorithm
    bool mSacOptimizeCoeffs = false;
    double mSacEpsAngle = 0.0;              // threshold on the angle between the normal at a point and the given
                                            // plane axis to consider it an inlier
    double mSacNormalDistWeight = 0.0;
};

struct PlaneModel
{
public:
    explicit PlaneModel(pcl::ModelCoefficients::Ptr pPlaneCoeffsPtr);
    ~PlaneModel() = default;

public:
    const Eigen::Vector4f mCoefficients;    // plane coefficients in Hessian normal form
    const pcl::PCLHeader mHeader;           // contain frame info from the point cloud
    PointCloud::Ptr mHullPointsPtr;         // pointer to convex hull points of the plane
    PointT mCenter;                         // mean of the convex hull points
    Eigen::Vector2f mRangeX;                // range of x coordinates
    Eigen::Vector2f mRangeY;                // range of y coordinates
};

/*!
 * @brief use a sample consensus (SAC, or in this case RANSAC) algorithm to fit a plane in point clouds
 *        see http://docs.pointclouds.org/1.7.0/group__sample__consensus.html for descriptions of the different
 *        SAC algorithms and model types.
 */
class SacPlaneSegmenter
{
public:
    SacPlaneSegmenter() : mSacModel(pcl::SACMODEL_NORMAL_PARALLEL_PLANE), mPlaneAxis(0.0f, 0.0f, 1.0f)
    {
        mNormalEstimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
    }

    /*! @brief set parameters of the SAC algorithm */
    void setParams(const SacPlaneSegmenterParams &pPlaneParams);

    /*!
     * @brief fit a plane from a point cloud and extract a PlaneModel object
     */
    PlaneModel
    findPlane(const PointCloud::ConstPtr &pCloudPtr);

private:
    /*!
     * @brief find inliers in a point cloud for a plane model using SAC
     * @param pCloudPtr: (in) cloud for finding inliers
     * @param pInliers: (out) indices of points that are considered inliers for the plane model
     * @param pCoefficients: (out) plane coefficients in the Hessian normal form
     */
    void
    findPlaneInliers(const PointCloud::ConstPtr &pCloudPtr, pcl::PointIndices::Ptr &pInliers,
                     pcl::ModelCoefficients::Ptr &pCoefficients);

    /*!
     * @brief find the bounding convex hull of the plane using the inlaying points returned from findPlaneInliers()
     *        and calculate plane center and limits using this convex hull
     * @param pCloudPtr: (in) cloud for finding hull
     * @param pInlierIndicesPtr: (in) indices of the inliers of the plane model as returned from findPlaneInliers()
     * @param pCoefficientsPtr: (in) fitted plane's coefficients in the Hessian form
     * @param pPlaneModel: (out) model with additional info about plane boundaries added
     */
    void
    findConvexHull(const PointCloud::ConstPtr &pCloudPtr, const pcl::PointIndices::Ptr &pInlierIndicesPtr,
                   const pcl::ModelCoefficients::Ptr &pCoefficientsPtr, PlaneModel &pPlaneModel);

private:
    const int cSacMethodType = pcl::SAC_RANSAC;

    pcl::NormalEstimation<PointT, pcl::PointNormal> mNormalEstimation;
    pcl::SACSegmentationFromNormals<PointT, pcl::PointNormal> mSacSegment;

    pcl::SacModel mSacModel;
    Eigen::Vector3f mPlaneAxis;
};

}   // namespace mas_perception_libs

#endif  // MAS_PERCEPTION_LIBS_SAC_PLANE_SEGMENTER_H
