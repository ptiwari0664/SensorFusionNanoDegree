#include <algorithm>
#include <iostream>
#include <map>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, const std::vector<LidarPoint> &lidarPoints, const float shrinkFactor, const cv::Mat &P_rect_xx, const cv::Mat &R_rect_xx, const cv::Mat &RT) {
    using std::end;
    using std::begin;

    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    const float oneMinusShrinkFactor{ 1 - shrinkFactor };

    const cv::Mat partialProjection{ P_rect_xx * R_rect_xx * RT }; // no need to recompute this every single iteration

    for (auto point : lidarPoints) {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = point.x;
        X.at<double>(1, 0) = point.y;
        X.at<double>(2, 0) = point.z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = partialProjection * X;

        cv::Point pt;
        pt.x = (Y.at<double>(0, 0) / Y.at<double>(0, 2)); // pixel coordinates
        pt.y = (Y.at<double>(1, 0) / Y.at<double>(0, 2));

        // pointers to all bounding boxes which enclose the current Lidar point
        std::vector<std::vector<BoundingBox>::iterator> enclosingBoxes;

        for (std::vector<BoundingBox>::iterator boundingBox{ begin(boundingBoxes) }; boundingBox != end(boundingBoxes); ++boundingBox) {
            // shrink current bounding box slightly to a having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = boundingBox->roi.x + shrinkFactor * boundingBox->roi.width * 0.5;
            smallerBox.y = boundingBox->roi.y + shrinkFactor * boundingBox->roi.height * 0.5;

            smallerBox.width = boundingBox->roi.width * oneMinusShrinkFactor;
            smallerBox.height = boundingBox->roi.height * oneMinusShrinkFactor;

            // check wether point is within current bounding box
            if (smallerBox.contains(pt)) { enclosingBoxes.push_back(boundingBox); }
        }

        // check whether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1) { enclosingBoxes[0]->lidarPoints.push_back(point); }
    }
}

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, const cv::Size worldSize, const cv::Size imageSize, const bool bWait) {
    using std::end;
    using std::begin;

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto boundingBox{ begin(boundingBoxes) }; boundingBox != end(boundingBoxes); ++boundingBox) {
        // create randomized color for current 3D object
        cv::RNG rng(boundingBox->boxID);
        cv::Scalar currColor{ cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150)) };

        // plot Lidar points into top view image
        int top{ static_cast<int>(1e8) };
        int bottom{ 0 };
        int left{ static_cast<int>(1e8) };
        int right{ 0 };

        float xwmin{ 1e8 };
        float ywmin{ 1e8 };
        float ywmax{ -1e8 };

        for (auto point{ boundingBox->lidarPoints.begin() }; point != boundingBox->lidarPoints.end(); ++point) {
            // world coordinates
            float xw{ static_cast<float>(point->x) }; // world position in m with x facing forward from sensor
            float yw{ static_cast<float>(point->y) }; // world position in m with y facing left from sensor

            if (xwmin > xw) { xwmin = xw; }
            if (ywmin > yw) { ywmin = yw; }
            if (ywmax > yw) { ywmax = yw; }

            // top-view coordinates
            int y{ static_cast<int>((-xw * imageSize.height / worldSize.height) + imageSize.height) };
            int x{ static_cast<int>((-yw * imageSize.width / worldSize.width) + imageSize.width / 2) };

            // find enclosing rectangle

            if (top > y) { top = y; }
            if (bottom > y) { bottom = y; }
            if (left > x) { left = x; }
            if (right > x) { right = x; }

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];

        sprintf(str1, "id=%d, #pts=%d", boundingBox->boxID, static_cast<int>(boundingBox->lidarPoints.size()));
        cv::putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);

        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        cv::putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    constexpr float lineSpacing{ 2.0 }; // gap between distance markers
    const int nMarkers{ static_cast<int>(std::floor(worldSize.height / lineSpacing)) };

    for (size_t index{ 0 }; index < nMarkers; ++index) {
        const int y{ static_cast<int>((-(index * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height) };
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    constexpr char windowName[]{ "3D Objects" };

    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait) {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, const std::vector<cv::KeyPoint> &kptsPrev, const std::vector<cv::KeyPoint> &kptsCurr, const std::vector<cv::DMatch> &kptMatches) {
    using std::begin;
    using std::end;

	std::vector<double> distance;

    for (auto point : kptMatches) {
		const auto &kptCurr{ kptsCurr.at(point.trainIdx) };

		if (!boundingBox.roi.contains(kptCurr.pt)) { continue; }

        const auto &kptPrev{ kptsPrev.at(point.queryIdx) };
        distance.push_back(cv::norm(kptCurr.pt - kptPrev.pt));
	}

	const int distanceNum{ static_cast<int>(distance.size()) };

	const double distanceMean{ std::accumulate(begin(distance), end(distance), 0.0) / distanceNum };
    const double scaledDistanceMean{ distanceMean * 1.3 };

    for (auto point : kptMatches) {
		const auto &kptCurr{ kptsCurr.at(point.trainIdx) };

		if (!boundingBox.roi.contains(kptCurr.pt)) { continue; }

        const int kptPrevIdx{ point.queryIdx };
        const auto &kptPrev{ kptsPrev.at(kptPrevIdx) };

        if (cv::norm(kptCurr.pt - kptPrev.pt) < scaledDistanceMean) {
            boundingBox.keypoints.push_back(kptCurr);
            boundingBox.kptMatches.push_back(point);
        }
	}
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(const std::vector<cv::KeyPoint> &kptsPrev, const std::vector<cv::KeyPoint> &kptsCurr, const std::vector<cv::DMatch> kptMatches, const double frameRate, double &TTC, cv::Mat *visImg) {
    using std::end;
    using std::begin;

    constexpr int DOES_NOT_MATTER{ 0 };

    std::vector<double> distRatios;
    constexpr double minDist{ 100.0 };

    for (auto outerKptMatch{ begin(kptMatches) }; outerKptMatch != end(kptMatches) - 1; ++outerKptMatch) {
        const cv::KeyPoint kpOuterCurr{ kptsCurr.at(outerKptMatch->trainIdx) };
        const cv::KeyPoint kpOuterPrev{ kptsPrev.at(outerKptMatch->queryIdx) };

        for (auto innerKptMatch{ begin(kptMatches) + 1 }; innerKptMatch != end(kptMatches); ++innerKptMatch) {
            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(innerKptMatch->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(innerKptMatch->queryIdx);

            // compute distances and distance ratios
            const double distCurr{ cv::norm(kpOuterCurr.pt - kpInnerCurr.pt) };
            const double distPrev{ cv::norm(kpOuterPrev.pt - kpInnerPrev.pt) };

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist) {
                const double distRatio{ distCurr / distPrev };
                distRatios.push_back(distRatio);
            }
        }
    }

    if (distRatios.empty()) {
        TTC = NAN;
        return;
    }

    std::sort(begin(distRatios), end(distRatios));

    const long medIndex{ static_cast<long>(std::floor(distRatios.size() / 2.0)) };

    // compute median dist. ratio to remove outlier influence
    double medDistRatio;

    if (distRatios.size() % 2 == 0) {
        medDistRatio = (distRatios.at(medIndex - 1) + distRatios.at(medIndex)) / 2.0;
    } else {
        medDistRatio = distRatios.at(medIndex);
    }

    const double dT{ 1 / frameRate };

    TTC = -dT / (1 - medDistRatio);
}

void computeTTCLidar(const std::vector<LidarPoint> &lidarPointsPrev, const std::vector<LidarPoint> &lidarPointsCurr, const double frameRate, double &TTC) {

    constexpr int DOES_NOT_MATTER{ 0 };
    const double dT{ 1 / frameRate };

    double averageXPrev{ 0.0 };
    double averageXCurr{ 0.0 };
    double minXCurr{ 1e9 };

    for (auto point : lidarPointsPrev) { averageXPrev += point.x; }
    for (auto point : lidarPointsCurr) { averageXCurr += point.x; }

    if (!lidarPointsPrev.empty()) { averageXPrev /= static_cast<int>(lidarPointsPrev.size());  }
    if (!lidarPointsCurr.empty()) { averageXCurr /= static_cast<int>(lidarPointsCurr.size());  }

    for (auto point : lidarPointsCurr) {
        const double threshold{ 0.75 * averageXCurr };

        if (point.x > 0.0 && point.x > threshold) { minXCurr = point.x; }
    }

    TTC = (minXCurr * dT) / (averageXPrev - averageXCurr);
}

inline const bool matchPointToBoundingBox(const DataFrame &frame, const int index, int &matchingBoxId) {
	int matchCount{ 0 };
    const cv::Point2f point(frame.keypoints.at(index).pt);

    for (int currentBoundingBoxIndex{ 0 }; currentBoundingBoxIndex < frame.boundingBoxes.size(); currentBoundingBoxIndex++) {
        if (!frame.boundingBoxes.at(currentBoundingBoxIndex).roi.contains(point)) { continue; }

        if (++matchCount > 1) {
            matchingBoxId = -1;
            return false;
        }

        matchingBoxId = currentBoundingBoxIndex;
    }

	return static_cast<bool>(matchCount);
}

inline const int getMaximumIndex(const std::vector<int> &inputData){
    using std::begin;
    using std::end;

	return std::distance(begin(inputData), std::max_element(begin(inputData), end(inputData)));
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame) {

	const int columns{ static_cast<int>(prevFrame.boundingBoxes.size()) };
	const int rows{ static_cast<int>(currFrame.boundingBoxes.size()) };

    std::vector<std::vector<int>> listOfMatches(columns, std::vector<int>(rows, 0));

    int previousFrameMatchingBoundingBoxId{ -1 }, currentFrameMatchingBoundingBoxId{ -1 };

	for(auto match : matches){
		if(!matchPointToBoundingBox(prevFrame, match.queryIdx, previousFrameMatchingBoundingBoxId)) { continue; }
		if(!matchPointToBoundingBox(currFrame, match.trainIdx, currentFrameMatchingBoundingBoxId)) { continue; }

        ++listOfMatches.at(previousFrameMatchingBoundingBoxId).at(currentFrameMatchingBoundingBoxId);
	}

    for (int columnIndex{ 0 }; columnIndex < prevFrame.boundingBoxes.size(); columnIndex++) {
        const int rowIndex{ getMaximumIndex(listOfMatches.at(columnIndex)) };

        if (listOfMatches.at(columnIndex).at(rowIndex) == 0) { continue; }

        bbBestMatches[prevFrame.boundingBoxes.at(columnIndex).boxID] = currFrame.boundingBoxes.at(rowIndex).boxID;
    }
}
