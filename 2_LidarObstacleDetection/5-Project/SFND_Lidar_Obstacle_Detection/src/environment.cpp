/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene{ false };
    std::vector<Car> cars{ initHighway(renderScene, viewer) };

    // TODO:: Create lidar sensor
    Lidar *lidar{ new Lidar(cars, 0) };

    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud{ lidar->scan() };

    // renderRays(viewer, lidar->position, pointCloud);
    renderPointCloud(viewer, pointCloud, std::string("Point Cloud"));

    // TODO:: Create point processor

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud{
        pointProcessor.SegmentPlane(pointCloud, 100, 0.2)
    };

    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters{ pointProcessor.Clustering(
        segmentCloud.first, 1.0, 3, 30) };

    int clusterId{ 0 };
    std::vector<Color> colors{ Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1) };

    bool renderBoundingBox{ true };

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterId));

        if (renderBoundingBox) {
            Box box(pointProcessor.BoundingBox(cluster));
            renderBox(viewer, box, clusterId);
        }

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {

    ProcessPointClouds<pcl::PointXYZI> *pointProcessor{ new ProcessPointClouds<pcl::PointXYZI>() };

    constexpr float X{ 30.0 }, Y{ 6.5 }, Z{ 2.5 };

    const pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud{ pointProcessor->FilterCloud(
        inputCloud, 0.1f, Eigen::Vector4f(-(X / 2), -Y, -Z, 1), Eigen::Vector4f(X, Y, Z, 1)) };

    renderPointCloud(viewer, filteredCloud, "filteredCloud");

    const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud{
        //     pointProcessor->SegmentPlane(filteredCloud, 100, 0.2)
        pointProcessor->RansacPlane(filteredCloud, 20, 0.2)
    };

    if (segmentCloud.first->empty() || segmentCloud.second->empty()) { return; }

    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0, 1, 0));

    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters{ pointProcessor->Clustering(
        segmentCloud.first, 0.35, 30, 1775) };

    if (cloudClusters.empty()) { return; }

    int clusterId{ 0 }, colorIndex{ 0 };

    const std::vector<Color> colors{ Color(1, 0, 1), Color(0, 1, 1), Color(1, 1, 0) };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(colorIndex));

        Box box(pointProcessor->BoundingBox(cluster));
        renderBox(viewer, box, clusterId);

        ++clusterId;
        ++colorIndex;

        colorIndex %= colors.size();
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance{ 16 };

    switch (setAngle) {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS) viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    CameraAngle setAngle{ XY };
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI{ new ProcessPointClouds<pcl::PointXYZI>() };

    std::vector<boost::filesystem::path> stream{ pointProcessorI->streamPcd(
        "../src/sensors/data/pcd/data_1") };

    auto streamIterator{ stream.begin() };

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // cityBlock(viewer);

    while (!viewer->wasStopped()) {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;

        if (streamIterator == stream.end()) { streamIterator = stream.begin(); }

        viewer->spinOnce();
    }
}
