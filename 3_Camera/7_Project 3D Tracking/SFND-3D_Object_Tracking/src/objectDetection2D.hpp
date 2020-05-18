
#ifndef objectDetection2D_hpp
#define objectDetection2D_hpp

#include <cstdio>
#include <opencv2/core.hpp>

#include "dataStructures.h"

void detectObjects(cv::Mat &img, std::vector<BoundingBox> &bBoxes, const float confThreshold,
                   const float nmsThreshold, const std::string basePath, const std::string classesFile,
                   const std::string modelConfiguration, const std::string modelWeights, const bool bVis);

#endif /* objectDetection2D_hpp */
