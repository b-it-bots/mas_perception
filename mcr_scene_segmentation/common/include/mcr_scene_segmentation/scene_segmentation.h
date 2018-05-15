#ifndef SCENE_SEGMENTATION_H
#define SCENE_SEGMENTATION_H

#include <mcr_scene_segmentation/aliases.h>
#include <mcr_scene_segmentation/bounding_box.h>
#include <vector>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/ModelCoefficients.h>

class SceneSegmentation
{
private:
    pcl::PassThrough<PointT> pass_through;
    pcl::VoxelGrid<PointT> voxel_grid;
    pcl::NormalEstimation<PointT, PointNT> normal_estimation;
    pcl::SACSegmentationFromNormals<PointT, PointNT> sac;
    pcl::ProjectInliers<PointT> project_inliers;
    pcl::ConvexHull<PointT> convex_hull;
    pcl::ExtractPolygonalPrismData<PointT> extract_polygonal_prism;
    pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
    pcl::RadiusOutlierRemoval<PointT> radius_outlier;

public:
    SceneSegmentation();
    virtual ~SceneSegmentation();

    PointCloud::Ptr segment_scene(const PointCloud::ConstPtr &cloud, std::vector<PointCloud::Ptr> &clusters, std::vector<BoundingBox> &boxes, double &workspace_height);

    void setVoxelGridParams(double leaf_size, const std::string &field_name, double limit_min, double limit_max);
    void setPassthroughParams(const std::string &field_name, double limit_min, double limit_max);
    void setNormalParams(double radius_search);
    void setSACParams(int max_iterations, double distance_threshold, bool optimize_coefficients, double eps_angle, double normal_distance_weight);

    void setPrismParams(double min_height, double max_height);
    void setOutlierParams(double radius_search, int min_neighbors);
    void setClusterParams(double cluster_tolerance, int cluster_min_size, int cluster_max_size, double cluster_min_height, double cluster_max_height,  double max_length, double cluster_min_distance_to_polygon);
};

#endif /* SCENE_SEGMENTATION_H */
