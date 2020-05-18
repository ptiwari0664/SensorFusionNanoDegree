#ifndef matching2D_hpp
#define matching2D_hpp

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"

inline const double secondsToMilliseconds(const double seconds);
inline const bool isValidDescriptorDetectorCombo(const std::string descriptor, const std::string detector);

std::vector<TimingInfo> initializeTimingInfoVector(void);

DetectionData detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const bool bVis = false);
DetectionData detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const bool bVis = false);
DetectionData detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const std::string detectorType, const bool bVis = false);
DetectionData descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, const std::string descriptorType);

DetectionData matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches,
                      const std::string descriptorFamily, const std::string matcherType,
                      const std::string selectorType);

void createCSVOutputFile(std::vector<TimingInfo> &info, const int numberOfImages);

#endif /* matching2D_hpp */
