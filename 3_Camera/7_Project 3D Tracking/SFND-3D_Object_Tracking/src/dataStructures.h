
#ifndef dataStructures_h
#define dataStructures_h

#include <map>
#include <opencv2/core.hpp>
#include <vector>

struct LidarPoint {    // single lidar point in space
    double x, y, z, r; // x,y,z in [m], r is point reflectivity
};

struct BoundingBox { // bounding box around a classified object (contains both 2D and 3D data)
    int boxID;   // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs

    cv::Rect roi;      // 2D region-of-interest in image coordinates
    int classID;       // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches;  // keypoint matches enclosed by 2D roi
};

struct DataFrame { // represents the available sensor information at the same time instance
    cv::Mat cameraImg; // camera image

    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors;                 // keypoint descriptors
    std::vector<cv::DMatch> kptMatches;  // keypoint matches between previous and current frame
    std::vector<LidarPoint> lidarPoints;

    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int, int> bbMatches;           // bounding box matches between previous and current frame
};

struct TimingInfo {
    const std::string detectorType, descriptorType, matcherType, selectorType;

    std::array<int, 20> ptsPerFrame, matchedPts;
    std::array<double, 20> ttcLidar, ttcCamera;
    std::array<bool, 20> enoughLidarOrCameraPointsDetected;

    // constructors
    TimingInfo() {}

    TimingInfo(const std::string detType, const std::string descType, const std::string matchType, const std::string selType)
        : detectorType(detType), descriptorType(descType), matcherType(matchType), selectorType(selType) {}
};

struct DetectionData {
    int numKeyPoints;
    double elapsedTime;

    // constructors
    DetectionData() : numKeyPoints(0), elapsedTime(0.0) {}

    DetectionData(int points, double time) : numKeyPoints(points), elapsedTime(time) {}
};

#endif /* dataStructures_h */
