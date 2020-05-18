
#ifndef lidarData_hpp
#define lidarData_hpp

#include <cstdio>
#include <fstream>
#include <string>

#include "dataStructures.h"

inline const bool withinBoundary(const double value, const double lowerBound, const double upperBound);

void cropLidarPoints(std::vector<LidarPoint> &lidarPoints, const float minX, const float maxX, const float maxY, const float minZ, const float maxZ, const float minR);

void loadLidarFromFile(std::vector<LidarPoint> &lidarPoints, const std::string filename);

void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait=true);
void showLidarImgOverlay(cv::Mat &img, std::vector<LidarPoint> &lidarPoints, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT, cv::Mat *extVisImg=nullptr);
#endif /* lidarData_hpp */
