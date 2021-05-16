// PCL lib Functions for processing point clouds 

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void voxelGrid(typename pcl::PointCloud<PointT>::Ptr inputCloud, 
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered,
        float filterRes) {
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*cloud_filtered);
}

template<typename PointT>
void cropBox(typename pcl::PointCloud<PointT>::Ptr inputCloud, 
         typename pcl::PointCloud<PointT>::Ptr box_cloud, 
         Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
    pcl::CropBox<PointT> cropFilter(true);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.setInputCloud(inputCloud);
    cropFilter.filter(*box_cloud);
}

template<typename PointT>
void removeRoof(typename pcl::PointCloud<PointT>::Ptr inputCloud,
        typename pcl::PointCloud<PointT>::Ptr cloudRegion) {
    std::vector<int> indices;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(inputCloud);
    roof.filter(indices);
    for (int point: indices) {
        inliers->indices.push_back(point);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(true); //get all non-roof points
    extract.filter(*cloudRegion);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr crop_box_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    ::voxelGrid<PointT>(cloud, filtered_cloud, filterRes);
    ::cropBox<PointT>(filtered_cloud, crop_box_cloud, minPoint, maxPoint);
    ::removeRoof<PointT>(crop_box_cloud, cloud_region);
    typename pcl::PointCloud<PointT>::Ptr out_cloud = cloud_region;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return out_cloud;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::ExtractIndices<PointT> extractor;
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>()), plane(new pcl::PointCloud<PointT>);
    for (int index : inliers->indices) 
        plane->points.push_back(cloud->points[index]);

    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);
    /*extractor.setNegative(false); //get the plane
    extractor.filter(*plane);*/
    std::cout << "1" << std::endl;
    extractor.setNegative(true);
    extractor.filter(*obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());
    typename pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers ->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    std::cout << "minSize: " << minSize << ", maxSize: " << maxSize << std::endl;
    std::cout << "cloud size: " << cloud->size() << std::endl;

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); 
    tree->setInputCloud(cloud);
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    std::cout << "cluster_indices size: " << cluster_indices.size() << std::endl;
    int i = 0;
    //convert the point indices to PointT
    //for (auto itr = cluster_indices.begin(); itr != cluster_indices.end(); ++itr) {
    for (const auto & pointIndices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        clusters.push_back(cloud_cluster);
        //for (auto pit = itr->indices.begin(); pit != itr->indices.end(); pit++) {
        for (const auto & pit : pointIndices.indices) {
            cloud_cluster->points.push_back(cloud->points[pit]);
            //std::cout << (*cloud)[*pit] << std::endl;
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "retreived cluster with size: " << i << " -- " << cloud_cluster->size() << std::endl;
        ++ i;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
