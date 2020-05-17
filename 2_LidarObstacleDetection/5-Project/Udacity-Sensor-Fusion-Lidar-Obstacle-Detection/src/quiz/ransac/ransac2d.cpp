/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <ctime>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter{ 0.6 };

    for (int i{ -5 }; i < 5; i++) {
        const double rx{ 2 * (((double)rand() / (RAND_MAX)) - 0.5) };
        const double ry{ 2 * (((double)rand() / (RAND_MAX)) - 0.5) };

        pcl::PointXYZ point;

        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }

    // Add outliers
    int numOutliers{ 10 };

    while (numOutliers--) {
        const double rx{ 2 * (((double)rand() / (RAND_MAX)) - 0.5) };
        const double ry{ 2 * (((double)rand() / (RAND_MAX)) - 0.5) };

        pcl::PointXYZ point;

        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));

    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);

    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations,
                               float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    srand(time(NULL));

    // For max iterations
    for (int i{ 0 }; i < maxIterations; i++) {
        std::unordered_set<int> inliers; // temporary map

        inliers.insert(rand() % cloud->points.size()); // take two random indicies
        inliers.insert(rand() % cloud->points.size());

        auto iterator{ inliers.begin() }; // start at the beginning of the map

        const float x1{ cloud->points[(*iterator)].x }; // get the x and y of those indices
        const float y1{ cloud->points[(*iterator)].y };

        ++iterator; // move to the next index of the map

        const float x2{ cloud->points[(*iterator)].x }; // get the x and y of those indices as well
        const float y2{ cloud->points[(*iterator)].y };

        const float a{ y1 - y2 };
        const float b{ x2 - x1 };
        const float c{ (x1 * y2) - (x2 - y1) };

        const float euclideanDist{ std::sqrt(a * a + b * b) };

        for (int j{ 0 }; j < cloud->points.size(); j++) {
            const float line{ std::fabs((a * cloud->points.at(j).x) + (b * cloud->points.at(j).y) + c) };

            const float currentdistance{ line / euclideanDist };

            if (currentdistance <= distanceTol) { inliers.insert(j); }
        }

        if (inliers.size() > inliersResult.size()) { inliersResult = inliers; }
    }

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations,
                                    float distanceTol) {

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    srand(time(NULL));

    // For max iterations
    while (maxIterations--) {

        std::unordered_set<int> inliers; // temporary map

        inliers.insert(rand() % cloud->points.size()); // three random indicies
        inliers.insert(rand() % cloud->points.size());
        inliers.insert(rand() % cloud->points.size());

        auto iterator{ inliers.begin() };

        const float x1{ cloud->points[(*iterator)].x }; // get the x and y of those indices
        const float y1{ cloud->points[(*iterator)].y };
        const float z1{ cloud->points[(*iterator)].z };

        ++iterator;

        const float x2{ cloud->points[(*iterator)].x };
        const float y2{ cloud->points[(*iterator)].y };
        const float z2{ cloud->points[(*iterator)].z };

        ++iterator;

        const float x3{ cloud->points[(*iterator)].x };
        const float y3{ cloud->points[(*iterator)].y };
        const float z3{ cloud->points[(*iterator)].z };

        const float i{ static_cast<float>(((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1))) };
        const float j{ static_cast<float>(((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1))) };
        const float k{ static_cast<float>(((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1))) };

        // yeah, it's stupid, but it helps with following the equations
        const float a{ i }, b{ j }, c{ k };

        const float d{ static_cast<float>(-((i * x1) + (j * y1) + (k * z1))) };

        const float euclideanDist{ std::sqrt((a * a) + (b * b) + (c * c)) };

        for (int ptIdx{ 0 }; ptIdx < cloud->points.size(); ptIdx++) {
            const float plane{ std::fabs((a * cloud->points.at(ptIdx).x) + (b * cloud->points.at(ptIdx).y) +
                                         (c * cloud->points.at(ptIdx).z) + d) };

            if ((plane / euclideanDist) <= distanceTol) { inliers.insert(ptIdx); }
        }

        if (inliers.size() > inliersResult.size()) { inliersResult = inliers; }
    }

    return inliersResult;
}

int main() {

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer{ initScene() };

    // Create data
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ CreateData() };
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ CreateData3D() };

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    // std::unordered_set<int> inliers { Ransac(cloud, 10, 1.0) };
    std::unordered_set<int> inliers{ RansacPlane(cloud, 30, 0.25) };

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index{ 0 }; index < cloud->points.size(); index++) {
        pcl::PointXYZ point{ cloud->points.at(index) };

        if (inliers.count(index)) {
            cloudInliers->points.push_back(point);
        } else {
            cloudOutliers->points.push_back(point);
        }
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) { viewer->spinOnce(); }
}
