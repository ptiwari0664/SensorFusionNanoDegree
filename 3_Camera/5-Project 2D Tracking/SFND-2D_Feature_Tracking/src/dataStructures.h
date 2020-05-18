#ifndef dataStructures_h
#define dataStructures_h

#include <opencv2/core.hpp>
#include <vector>
#include <array>

struct DataFrame { // represents the available sensor information at the same time instance
    cv::Mat cameraImg;                   // camera image
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors;                 // keypoint descriptors
    std::vector<cv::DMatch> kptMatches;  // keypoint matches between previous and current frame
};

struct TimingInfo {
    const std::string detectorType, descriptorType, matcherType, selectorType;

    std::array<int, 10> ptsPerFrame, pointsLeftOnImg, matchedPts;
    std::array<double, 10> detElapsedTime, descElapsedTime, matchElapsedTime;

    // constructors
    TimingInfo() {}

    TimingInfo(const std::string detType, const std::string descType, const std::string matchType, const std::string selType)
        : detectorType(detType), descriptorType(descType), matcherType(matchType), selectorType(selType) {
    }
};

struct DetectionData {
    int numKeyPoints;
    double elapsedTime;

    // constructors
    DetectionData() : numKeyPoints(0), elapsedTime(0.0) {}

    DetectionData(int points, double time) : numKeyPoints(points), elapsedTime(time) {}
};

#endif /* dataStructures_h */
