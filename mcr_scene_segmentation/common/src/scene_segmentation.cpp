/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#include <iostream>
#include <mcr_scene_segmentation/scene_segmentation.h>
#include <string>
#include <vector>

using mas_perception_libs::BoundingBox;

SceneSegmentation::SceneSegmentation()
{
    cluster_extraction.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
}

SceneSegmentation::~SceneSegmentation() = default;

PointCloud::Ptr SceneSegmentation::segment_scene(const PointCloud::ConstPtr &cloud,
        std::vector<PointCloud::Ptr> &clusters, std::vector<BoundingBox> &boxes, double &workspace_height)
{
    PointCloud::Ptr hull;
    //Eigen::Vector4f coefficients;
    PointCloud::Ptr filtered = findPlane(cloud, hull, workspace_height);

    pcl::PointIndices::Ptr segmented_cloud_inliers = boost::make_shared<pcl::PointIndices>();
    extract_polygonal_prism.setInputPlanarHull(hull);
    extract_polygonal_prism.setInputCloud(cloud);
    extract_polygonal_prism.setViewPoint(0.0, 0.0, 2.0);
    extract_polygonal_prism.segment(*segmented_cloud_inliers);

    std::vector<pcl::PointIndices> clusters_indices;
    cluster_extraction.setInputCloud(cloud);
    cluster_extraction.setIndices(segmented_cloud_inliers);
    cluster_extraction.extract(clusters_indices);

    for (const auto &cluster_indices : clusters_indices)
    {
        PointCloud::Ptr cluster = boost::make_shared<PointCloud>();
        pcl::copyPointCloud(*cloud, cluster_indices, *cluster);
        clusters.push_back(cluster);
        Eigen::Vector3f normal(coefficients_[0], coefficients_[1], coefficients_[2]);
        BoundingBox box = BoundingBox::create(cluster->points, normal);
        boxes.push_back(box);
    }
    return filtered;
}

PointCloud::Ptr SceneSegmentation::findPlane(const PointCloud::ConstPtr &cloud, PointCloud::Ptr &hull,
                                             double &workspace_height)
{
    PointCloud::Ptr filtered = cloud_filter.filterCloud(cloud);
    try
    {
        mpl::PlaneModel planeModel = plane_segmenter.findPlane(filtered);
        coefficients_ = planeModel.mCoefficients;
        hull = planeModel.mHullPointsPtr;
        workspace_height = planeModel.mCenter.z;
    }
    catch (std::runtime_error &ex)
    {
        // return filtered cloud if cannot find plane
        return filtered;
    }
}

void SceneSegmentation::setCloudFilterParams(const mpl::CloudFilterParams& params)
{
    cloud_filter.setParams(params);
}

void SceneSegmentation::setPlaneSegmenterParams(const mpl::SacPlaneSegmenterParams& params)
{
    plane_segmenter.setParams(params);
}

void SceneSegmentation::setPrismParams(double min_height, double max_height)
{
    extract_polygonal_prism.setHeightLimits(min_height, max_height);
}

void SceneSegmentation::setOutlierParams(double radius_search, int min_neighbors)
{
    radius_outlier.setRadiusSearch(radius_search);
    radius_outlier.setMinNeighborsInRadius(min_neighbors);
}
void SceneSegmentation::setClusterParams(double cluster_tolerance, int cluster_min_size,
        int cluster_max_size, double cluster_min_height, double cluster_max_height,
        double max_length, double cluster_min_distance_to_polygon)
{
    cluster_extraction.setClusterTolerance(cluster_tolerance);
    cluster_extraction.setMinClusterSize(cluster_min_size);
    cluster_extraction.setMaxClusterSize(cluster_max_size);
}
