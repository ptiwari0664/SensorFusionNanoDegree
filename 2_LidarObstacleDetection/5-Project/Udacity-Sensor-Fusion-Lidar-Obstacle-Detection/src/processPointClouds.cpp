// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {
}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {
}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
    // Time segmentation process
    const auto startTime{ std::chrono::steady_clock::now() };

    // TODO:: Fill in the function to do voxel grid point reduction and region
    // based filtering

    pcl::VoxelGrid<PointT> voxelGrid;

    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);

    typename pcl::PointCloud<PointT>::Ptr filteredVoxelCloud{ new pcl::PointCloud<PointT>() };
    voxelGrid.filter(*filteredVoxelCloud);

    pcl::CropBox<PointT> cropBox(true);

    cropBox.setInputCloud(filteredVoxelCloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);

    typename pcl::PointCloud<PointT>::Ptr filteredCroppedCloud{ new pcl::PointCloud<PointT>() };
    cropBox.filter(*filteredCroppedCloud);

    pcl::CropBox<PointT> vehicleRoof(true);

    vehicleRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    vehicleRoof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    vehicleRoof.setInputCloud(filteredCroppedCloud);

    std::vector<int> inlierIndices;
    vehicleRoof.filter(inlierIndices);

    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };

    for (int index : inlierIndices) { inliers->indices.push_back(index); }

    pcl::ExtractIndices<PointT> extractedIndices;

    extractedIndices.setInputCloud(filteredCroppedCloud);
    extractedIndices.setIndices(inliers);
    extractedIndices.setNegative(true);
    extractedIndices.filter(*filteredCroppedCloud);

    const auto endTime{ std::chrono::steady_clock::now() };
    const auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };

    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCroppedCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud) {
    // TODO: Create two new point clouds, one cloud with obstacles and other
    // with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud{ new pcl::PointCloud<PointT> };
    typename pcl::PointCloud<PointT>::Ptr roadCloud{ new pcl::PointCloud<PointT> };

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*roadCloud);

    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(
        obstacleCloud, roadCloud);

    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                         float distanceThreshold) {
    // Time segmentation process
    auto startTime{ std::chrono::steady_clock::now() };

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices() };
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT> segmentation;

    segmentation.setOptimizeCoefficients(true);
    segmentation.setMaxIterations(maxIterations);
    segmentation.setDistanceThreshold(distanceThreshold);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);

    if (cloud->points.empty()) { std::cout << "Failed to segment points." << std::endl; }

    auto endTime{ std::chrono::steady_clock::now() };
    auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };

    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult{
        SeparateClouds(inliers, cloud)
    };

    return segResult;
}

// MY ALGORITHM IMPLEMENTATIONS : START

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                        const float distanceTol) {

    std::unordered_set<int> inliersResult;

    // TODO: Fill in this function
    srand(time(NULL));

    while (maxIterations--) {

        std::unordered_set<int> inliers; // temporary map

        // insert three random indicies
        for (int i{ 0 }; i < 3; i++) { inliers.insert(rand() % cloud->points.size()); }

        auto iterator{ std::begin(inliers) };

        const float x1{ cloud->points.at((*iterator)).x };
        const float y1{ cloud->points.at((*iterator)).y };
        const float z1{ cloud->points.at((*iterator)).z };

        ++iterator;

        const float x2{ cloud->points.at((*iterator)).x };
        const float y2{ cloud->points.at((*iterator)).y };
        const float z2{ cloud->points.at((*iterator)).z };

        // I have ABSOLUTELY no clue why the map was sometimes only inserting 2 indices
        // I shouldn't have to do this, but a seg fault was occurring without this check
        if (inliers.size() == 2) { inliers.insert(rand() % cloud->points.size()); }

        ++iterator;

        const float x3{ cloud->points.at((*iterator)).x };
        const float y3{ cloud->points.at((*iterator)).y };
        const float z3{ cloud->points.at((*iterator)).z };

        const float i{ static_cast<float>(((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1))) };
        const float j{ static_cast<float>(((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1))) };
        const float k{ static_cast<float>(((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1))) };

        // yeah, it's stupid, but it helps with following the given equations
        const float A{ i }, B{ j }, C{ k };

        const float D{ static_cast<float>(-((i * x1) + (j * y1) + (k * z1))) };

        const float euclideanDist{ std::sqrt((A * A) + (B * B) + (C * C)) };

        for (int index{ 0 }; index < cloud->points.size(); index++) {
            const float plane{ std::fabs((A * cloud->points.at(index).x) + (B * cloud->points.at(index).y) +
                                         (C * cloud->points.at(index).z) + D) };

            if ((plane / euclideanDist) <= distanceTol) { inliers.insert(index); }
        }

        if (inliers.size() > inliersResult.size()) { inliersResult = inliers; }
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers{ new pcl::PointCloud<PointT>() };
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers{ new pcl::PointCloud<PointT>() };

    if (!inliersResult.empty()) {
        for (int index{ 0 }; index < cloud->points.size(); index++) {
            const PointT point{ cloud->points.at(index) };

            if (inliersResult.count(index)) {
                cloudInliers->points.push_back(point);
            } else {
                cloudOutliers->points.push_back(point);
            }
        }
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
        cloudOutliers, cloudInliers);
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(const std::vector<std::vector<float>> &points,
                                           std::vector<int> &cluster, std::vector<bool> &processed,
                                           const int index, const float tolerance, KdTree *tree) {

    processed.at(index) = true;
    cluster.push_back(index);

    const std::vector<int> nearbyPoints{ tree->search(points.at(index), tolerance) };

    for (int nearbyIndex : nearbyPoints) {
        if (!processed.at(nearbyIndex)) {
            proximity(points, cluster, processed, nearbyIndex, tolerance, tree);
        }
    }
}

template <typename PointT>
std::vector<std::vector<int>>
ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree,
                                             float distanceTol) {

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (int i{ 0 }; i < points.size(); i++) {
        if (!processed.at(i)) {
            std::vector<int> cluster;
            proximity(points, cluster, processed, i, distanceTol, tree);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       const float clusterTolerance, const int minSize, const int maxSize) {

    const auto startTime{ std::chrono::steady_clock::now() };

    int i{ 0 };
    KdTree *kdTree{ new KdTree() };

    std::vector<std::vector<float>> points;

    for (auto point : cloud->points) {
        const std::vector<float> p{ point.x, point.y, point.z };
        kdTree->insert(p, i++);
        points.push_back(p);
    }

    const std::vector<std::vector<int>> listOfIndices{ euclideanCluster(points, kdTree, clusterTolerance) };

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (auto indices : listOfIndices) {

        if (indices.size() < minSize || indices.size() > maxSize) { continue; }

        typename pcl::PointCloud<PointT>::Ptr cluster{ new pcl::PointCloud<PointT> };

        for (auto index : indices) { cluster->points.push_back(cloud->points[index]); }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    const auto endTime{ std::chrono::steady_clock::now() };
    const auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };

    std::cout << "clustering took " << elapsedTime.count();
    std::cout << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// MY ALGORITHM IMPLEMENTATIONS : END

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);

    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
        PCL_ERROR("Couldn't read file \n");

    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{ dataPath },
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
