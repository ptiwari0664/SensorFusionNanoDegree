
#ifndef camFusion_hpp
#define camFusion_hpp

#include "dataStructures.h"
#include <opencv2/core.hpp>
#include <cstdio>
#include <vector>

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes,
                         const std::vector<LidarPoint> &lidarPoints,
                         const float shrinkFactor,
                         const cv::Mat &P_rect_xx,
                         const cv::Mat &R_rect_xx,
                         const cv::Mat &RT);

void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              const std::vector<cv::KeyPoint> &kptsPrev,
                              const std::vector<cv::KeyPoint> &kptsCurr,
                              const std::vector<cv::DMatch> &kptMatches);

void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bbBestMatches,
                        DataFrame &prevFrame,
                        DataFrame &currFrame);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes,
                   const cv::Size worldSize,
                   const cv::Size imageSize,
                   const bool bWait = true);

void computeTTCCamera(const std::vector<cv::KeyPoint> &kptsPrev,
                      const std::vector<cv::KeyPoint> &kptsCurr,
                      const std::vector<cv::DMatch> kptMatches,
                      const double frameRate,
                      double &TTC,
                      cv::Mat *visImg = nullptr);

void computeTTCLidar(const std::vector<LidarPoint> &lidarPointsPrev,
                     const std::vector<LidarPoint> &lidarPointsCurr,
                     const double frameRate,
                     double &TTC);

inline const int getMaximumIndex(const std::vector<int> &inputData);
inline const bool matchPointToBoundingBox(const DataFrame &frame, const int index, int &matchingBoxId);

#endif /* camFusion_hpp */
