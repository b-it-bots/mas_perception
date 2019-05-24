/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_SCENE_SEGMENTATION_H
#define MCR_SCENE_SEGMENTATION_SCENE_SEGMENTATION_H

#include <mas_perception_libs/bounding_box.h>
#include <mas_perception_libs/sac_plane_segmenter.h>
#include <mas_perception_libs/point_cloud_utils.h>
#include <mcr_scene_segmentation/aliases.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/ModelCoefficients.h>
#include <vector>

namespace mpl = mas_perception_libs;

class SceneSegmentation
{
private:
    pcl::ExtractPolygonalPrismData<PointT> extract_polygonal_prism;
    pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
    pcl::RadiusOutlierRemoval<PointT> radius_outlier;
    mpl::CloudFilter cloud_filter;
    mpl::SacPlaneSegmenter plane_segmenter;

public:
    SceneSegmentation();
    virtual ~SceneSegmentation();

    PointCloud::Ptr segment_scene(const PointCloud::ConstPtr &cloud, std::vector<PointCloud::Ptr> &clusters,
    std::vector<mas_perception_libs::BoundingBox> &boxes, double &workspace_height);
    PointCloud::Ptr findPlane(const PointCloud::ConstPtr &cloud, PointCloud::Ptr &hull, double &workspace_height);

    void setCloudFilterParams(const mpl::CloudFilterParams&);
    void setPlaneSegmenterParams(const mpl::SacPlaneSegmenterParams&);
    void setPrismParams(double min_height, double max_height);
    void setOutlierParams(double radius_search, int min_neighbors);
    void setClusterParams(double cluster_tolerance, int cluster_min_size, int cluster_max_size,
                          double cluster_min_height, double cluster_max_height,  double max_length,
                          double cluster_min_distance_to_polygon);
public:
    // Make model coefficients public for 3d bbox from rgb bbox propossal
    Eigen::Vector4f coefficients_;
};

#endif  // MCR_SCENE_SEGMENTATION_SCENE_SEGMENTATION_H
