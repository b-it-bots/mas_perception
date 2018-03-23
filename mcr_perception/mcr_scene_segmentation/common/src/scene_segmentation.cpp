#include <mcr_scene_segmentation/scene_segmentation.h>

SceneSegmentation::SceneSegmentation()
{
    cluster_extraction.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
    normal_estimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
}

SceneSegmentation::~SceneSegmentation()
{
}

PointCloud::Ptr SceneSegmentation::segment_scene(const PointCloud::ConstPtr &cloud, std::vector<PointCloud::Ptr> &clusters, std::vector<BoundingBox> &boxes)
{
    PointCloud::Ptr filtered(new PointCloud);
    PointCloud::Ptr plane(new PointCloud);
    PointCloud::Ptr hull(new PointCloud);
    pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);
    std::vector<pcl::PointIndices> clusters_indices;

    PointCloudN::Ptr normals(new PointCloudN);

    voxel_grid.setInputCloud(cloud);
    voxel_grid.filter(*filtered);

    pass_through.setInputCloud(filtered);
    pass_through.filter(*filtered);

    normal_estimation.setInputCloud(filtered);
    normal_estimation.compute(*normals);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    sac.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));

    sac.setInputCloud(filtered);
    sac.setInputNormals(normals);
    sac.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        return filtered;
    }

    project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    project_inliers.setInputCloud(filtered);
    project_inliers.setModelCoefficients(coefficients);
    project_inliers.setIndices(inliers);
    project_inliers.setCopyAllData(false);
    project_inliers.filter(*plane);

    convex_hull.setInputCloud(plane);
    convex_hull.reconstruct(*hull);

    // not sure if this is necessary
    hull->points.push_back(hull->points.front());
    hull->width += 1;

    extract_polygonal_prism.setInputPlanarHull(hull);
    extract_polygonal_prism.setInputCloud(cloud);
    extract_polygonal_prism.setViewPoint(0.0, 0.0, 2.0);
    extract_polygonal_prism.segment(*segmented_cloud_inliers);

    cluster_extraction.setInputCloud(cloud);
    cluster_extraction.setIndices(segmented_cloud_inliers);

    cluster_extraction.extract(clusters_indices);

    const Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        PointCloud::Ptr cluster(new PointCloud);
        pcl::copyPointCloud(*cloud, cluster_indices, *cluster);
        clusters.push_back(cluster);
        BoundingBox box = BoundingBox::create(cluster->points, normal);
        boxes.push_back(box);
    }
    return filtered;
}

void SceneSegmentation::setVoxelGridParams(double leaf_size, const std::string &filter_field, double limit_min, double limit_max)
{
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setFilterFieldName(filter_field);
    voxel_grid.setFilterLimits(limit_min, limit_max);
}

void SceneSegmentation::setPassthroughParams(const std::string &field_name, double limit_min, double limit_max)
{
    pass_through.setFilterFieldName(field_name);
    pass_through.setFilterLimits(limit_min, limit_max);
}

void SceneSegmentation::setNormalParams(double radius_search)
{
    normal_estimation.setRadiusSearch(radius_search);
}
void SceneSegmentation::setSACParams(int max_iterations, double distance_threshold, bool optimize_coefficients, double eps_angle, double normal_distance_weight)
{
    sac.setMaxIterations(max_iterations);
    sac.setDistanceThreshold(distance_threshold);
    sac.setEpsAngle(eps_angle);
    sac.setOptimizeCoefficients(optimize_coefficients);
    sac.setNormalDistanceWeight(normal_distance_weight);
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
void SceneSegmentation::setClusterParams(double cluster_tolerance, int cluster_min_size, int cluster_max_size, double cluster_min_height, double cluster_max_height, double max_length, double cluster_min_distance_to_polygon)
{
    cluster_extraction.setClusterTolerance(cluster_tolerance);
    cluster_extraction.setMinClusterSize(cluster_min_size);
    cluster_extraction.setMaxClusterSize(cluster_max_size);
}
