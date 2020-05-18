#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>
#include <iostream>

#include "dataStructures.h"
#include "matching2D.hpp"

inline const bool isValidDescriptorDetectorCombo(const std::string descriptor, const std::string detector) {
    return !((descriptor.compare("AKAZE") == 0 && detector.compare("AKAZE") != 0) ||
            (descriptor.compare("ORB") == 0 && detector.compare("SIFT") == 0));
}

std::vector<TimingInfo> initializeTimingInfoVector(void) {
    const std::vector<std::string> detectorTypes{ "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE",  "SIFT" };
    const std::vector<std::string> descriptorTypes{ "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" };
    const std::vector<std::string> matcherTypes{ "MAT_BF" };
    const std::vector<std::string> selectorTypes{ "SEL_KNN" };

    std::vector<TimingInfo> info;

    for (auto detectorType : detectorTypes) {
        for (auto descriptorType : descriptorTypes) {
            for (auto matcherType : matcherTypes) {
                for (auto selectorType : selectorTypes) {

                    if (!isValidDescriptorDetectorCombo(descriptorType, detectorType)) { continue; }

                    info.push_back(TimingInfo(detectorType, descriptorType, matcherType, selectorType));
                }
            }
        }
    }

    return info;
}

void createCSVOutputFile(std::vector<TimingInfo> &timingInfo) {
    constexpr char COMMA[]{ ", " };
    constexpr char csvName[]{ "../report/Brandon_Marlowe_Midterm_Project.csv" };

    std::cout << "Writing output file: " << csvName << std::endl;
    std::ofstream csvStream{ csvName };

    csvStream << "Name: Brandon Marlowe" << std::endl << "Date: 2019-09-16" << std::endl << std::endl;

    csvStream << "IMAGE NO." << COMMA;
    csvStream << "DETECTOR TYPE" << COMMA;
    csvStream << "DESCRIPTOR TYPE" << COMMA;
    csvStream << "TOTAL KEYPOINTS" << COMMA;
    csvStream << "KEYPOINTS ON VEHICLE" << COMMA;
    csvStream << "DETECTOR ELAPSED TIME" << COMMA;
    csvStream << "DESCRIPTOR ELAPSED TIME" << COMMA;
    csvStream << "MATCHED KEYPOINTS" << COMMA;
    csvStream << "MATCHER ELAPSED TIME";
    csvStream << std::endl;

    for (auto &info : timingInfo) {
        for (int index{ 0 }; index < 10; index++) {
            csvStream << index << COMMA;
            csvStream << info.detectorType << COMMA;
            csvStream << info.descriptorType << COMMA;
            csvStream << info.ptsPerFrame.at(index) << COMMA;
            csvStream << info.pointsLeftOnImg.at(index) << COMMA;
            csvStream << info.detElapsedTime.at(index) << COMMA;
            csvStream << info.descElapsedTime.at(index) << COMMA;
            csvStream << info.matchedPts.at(index) << COMMA;
            csvStream << info.matchElapsedTime.at(index) << std::endl;
        }

        csvStream << std::endl;
    }

    csvStream.close();
}

/* MAIN PROGRAM */
int main(const int argc, const char *argv[]) {
    std::vector<TimingInfo> timingInfo{ initializeTimingInfoVector() };

    /* INIT VARIABLES AND DATA STRUCTURES */
    const std::string dataPath{ "../" };                                     // data location
    const std::string imgBasePath{ dataPath + "images/" };                  // camera
    const std::string imgPrefix{ "KITTI/2011_09_26/image_00/data/000000" }; // left camera, color
    const std::string imgFileType{ ".png" };                                // filetype

    // first file index to load (assumes Lidar and camera names have identical naming convention)
    constexpr int imgStartIndex{ 0 };
    constexpr int imgEndIndex{ 9 };  // last file index to load
    constexpr int imgFillWidth{ 4 }; // no. of digits which make up the file index (e.g. img-0001.png)
    constexpr int totalImages{ imgEndIndex - imgStartIndex };

    // no. of images which are held in memory (ring buffer) at the same time
    constexpr int dataBufferSize{ 2 };
    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

    bool bVis{ false };               // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (auto &info : timingInfo) {
        dataBuffer.clear();

        for (size_t imgIndex{ 0 }; imgIndex <= totalImages; imgIndex++) {

            DetectionData data;

            std::cout << "===== DETAILS FOR IMAGE: " << (imgIndex + 1) << " =====\n" << std::endl;
            std::cout << "DETECTOR Type := " << info.detectorType << std::endl;
            std::cout << "DESCRIPTOR Type := " << info.descriptorType << std::endl;
            std::cout << "MATCHER Type := " << info.matcherType << std::endl;
            std::cout << "SELECTOR Type := " << info.selectorType << std::endl;
            std::cout << std::endl;

            // assemble filenames for current index
            std::ostringstream imgNumber;
            imgNumber << std::setfill('0') << std::setw(imgFillWidth) << (imgStartIndex + imgIndex);

            const std::string imgFullFilename{ imgBasePath + imgPrefix + imgNumber.str() + imgFileType };

            const cv::Mat img{ cv::imread(imgFullFilename) }; // load image from file and convert to grayscale

            cv::Mat imgGray;
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

            //// STUDENT ASSIGNMENT
            //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = imgGray;

            if (dataBuffer.size() == dataBufferSize) { dataBuffer.erase(std::begin(dataBuffer)); }

            dataBuffer.push_back(frame);

            //// EOF STUDENT ASSIGNMENT
            std::cout << "Step #1 : LOAD IMAGE INTO BUFFER done" << std::endl;

            /* DETECT IMAGE KEYPOINTS */

            // extract 2D keypoints from current image
            std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

            //// STUDENT ASSIGNMENT
            //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable
            /// string-based selection based on detectorType / -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

            if (info.detectorType.compare("SHITOMASI") == 0) {
                data = detKeypointsShiTomasi(keypoints, imgGray, false);

            } else if (info.detectorType.compare("HARRIS") == 0) {
                data = detKeypointsHarris(keypoints, imgGray, false);

            } else {
                data = detKeypointsModern(keypoints, imgGray, info.detectorType, false);
            }

            info.ptsPerFrame.at(imgIndex) = data.numKeyPoints;
            info.detElapsedTime.at(imgIndex) = data.elapsedTime;

            //// EOF STUDENT ASSIGNMENT

            //// STUDENT ASSIGNMENT
            //// TASK MP.3 -> only keep keypoints on the preceding vehicle

            // only keep keypoints on the preceding vehicle
            constexpr bool bFocusOnVehicle{ true };

            if (bFocusOnVehicle) {
                const cv::Rect vehicleRect(535, 180, 180, 150);
                std::vector<cv::KeyPoint> retainedPoints;

                for (auto point : keypoints) {
                    if (vehicleRect.contains(cv::Point2f(point.pt))) { retainedPoints.push_back(point); }
                }

                keypoints = retainedPoints;

                info.pointsLeftOnImg.at(imgIndex) = keypoints.size();
                std::cout << std::endl;
            }

            //// EOF STUDENT ASSIGNMENT

            // optional : limit number of keypoints (helpful for debugging and learning)
            constexpr bool bLimitKpts{ false };

            if (bLimitKpts) {
                constexpr int maxKeypoints{ 50 };

                // there is no response info, so keep the first 50 as they are sorted in descending quality order
                if (info.detectorType.compare("SHITOMASI") == 0) {
                    keypoints.erase(std::begin(keypoints) + maxKeypoints, std::end(keypoints));
                }

                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                std::cout << "NOTE: Keypoints have been limited!" << std::endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (std::end(dataBuffer) - 1)->keypoints = keypoints;

            std::cout << "Step #2 : DETECT KEYPOINTS done" << std::endl;

            /* EXTRACT KEYPOINT DESCRIPTORS */

            //// STUDENT ASSIGNMENT
            //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based
            /// selection based on descriptorType / -> BRIEF, ORB, FREAK, AKAZE, SIFT

            cv::Mat descriptors;

            data = descKeypoints((std::end(dataBuffer) - 1)->keypoints,
                                 (std::end(dataBuffer) - 1)->cameraImg,
                                 descriptors,
                                 info.descriptorType);

            info.descElapsedTime.at(imgIndex) = data.elapsedTime;

            //// EOF STUDENT ASSIGNMENT

            // push descriptors for current frame to end of data buffer
            (std::end(dataBuffer) - 1)->descriptors = descriptors;

            std::cout << "Step #3 : EXTRACT DESCRIPTORS done" << std::endl;

            // wait until at least two images have been processed
            if (dataBuffer.size() > 1) {
                /* MATCH KEYPOINT DESCRIPTORS */
                std::vector<cv::DMatch> matches;
                const std::string descriptorFamily{ (info.descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY" };

                //// STUDENT ASSIGNMENT
                //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with
                /// t = 0.8 in file matching2D.cpp

                data = matchDescriptors((std::end(dataBuffer) - 2)->keypoints,
                                        (std::end(dataBuffer) - 1)->keypoints,
                                        (std::end(dataBuffer) - 2)->descriptors,
                                        (std::end(dataBuffer) - 1)->descriptors,
                                        matches,
                                        descriptorFamily,
                                        info.matcherType,
                                        info.selectorType);

                info.matchedPts.at(imgIndex) = data.numKeyPoints;
                info.matchElapsedTime.at(imgIndex) = data.elapsedTime;

                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                (std::end(dataBuffer) - 1)->kptMatches = matches;

                std::cout << "Step #4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

                // visualize matches between current and previous image
                cv::Mat matchImg{ ((std::end(dataBuffer) - 1)->cameraImg).clone() };

                if (bVis) {
                    cv::drawMatches((std::end(dataBuffer) - 2)->cameraImg,
                                    (std::end(dataBuffer) - 2)->keypoints,
                                    (std::end(dataBuffer) - 1)->cameraImg,
                                    (std::end(dataBuffer) - 1)->keypoints,
                                    matches,
                                    matchImg,
                                    cv::Scalar::all(-1),
                                    cv::Scalar::all(-1),
                                    std::vector<char>(),
                                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                    constexpr char windowName[]{ "Matching keypoints between two camera images" };

                    cv::namedWindow(windowName, 7);
                    cv::imshow(windowName, matchImg);

                    std::cout << "Press key to continue to next image" << std::endl;
                    cv::waitKey(0); // wait for key to be pressed

                }

            } else {
                info.matchedPts.at(imgIndex) = info.matchElapsedTime.at(imgIndex) = 0;

            }

            std::cout << "\n================================\n" << std::endl;
            std::cout << "................................\n" << std::endl;
        } // eof loop over all images
    }

    createCSVOutputFile(timingInfo);

    return 0;
}
