
/* INCLUDES FOR THIS PROJECT */
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

#include "camFusion.hpp"
#include "dataStructures.h"
#include "lidarData.hpp"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"

using std::end;
using std::begin;

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

void createCSVOutputFile(std::vector<TimingInfo> &timingInfo, const int numberOfImages) {
    constexpr char COMMA[]{ ", " };
    constexpr char csvName[]{ "../report/Brandon_Marlowe_Final_3D_Camera_Project.csv" };

    std::cout << "Writing output file: " << csvName << std::endl;
    std::ofstream csvStream{ csvName };

    csvStream << "Name: Brandon Marlowe" << std::endl << "Date: 2019-10-08" << std::endl << std::endl;

    csvStream << "IMAGE NO." << COMMA;
    csvStream << "DETECTOR TYPE" << COMMA;
    csvStream << "DESCRIPTOR TYPE" << COMMA;
    csvStream << "TOTAL KEYPOINTS" << COMMA;
    csvStream << "TTC LIDAR" << COMMA;
    csvStream << "TTC CAMERA" << COMMA;
    csvStream << std::endl;

    for (auto &info : timingInfo) {
        for (int index{ 0 }; index < numberOfImages; index++) {
            if (!info.enoughLidarOrCameraPointsDetected.at(index)) { continue; }

            csvStream << index << COMMA;
            csvStream << info.detectorType << COMMA;
            csvStream << info.descriptorType << COMMA;
            csvStream << info.ptsPerFrame.at(index) << COMMA;
            csvStream << info.ttcLidar.at(index) << COMMA;
            csvStream << info.ttcCamera.at(index) << std::endl;
        }
        csvStream << std::endl;
    }

    csvStream.close();
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
    std::vector<TimingInfo> timingInfo{ initializeTimingInfoVector() };

    /* INIT VARIABLES AND DATA STRUCTURES */
    // data location
    const std::string dataPath{ "../" };

    // camera
    const std::string imgBasePath{ dataPath + "images/" };
    const std::string imgPrefix{ "KITTI/2011_09_26/image_02/data/000000" }; // left camera, color
    const std::string imgFileType{ ".png" };

    // first file index to load (assumes Lidar and camera names have identical naming convention)
    constexpr int imgStartIndex{ 0 };
    constexpr int imgEndIndex{ 18 }; // last file index to load
    constexpr int imgStepWidth{ 1 };
    constexpr int imgFillWidth{ 4 }; // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    const std::string yoloBasePath{ dataPath + "dat/yolo/" };
    const std::string yoloClassesFile{ yoloBasePath + "coco.names" };
    const std::string yoloModelConfiguration{ yoloBasePath + "yolov3.cfg" };
    const std::string yoloModelWeights{ yoloBasePath + "yolov3.weights" };

    // Lidar
    const std::string lidarPrefix{ "KITTI/2011_09_26/velodyne_points/data/000000" };
    const std::string lidarFileType{ ".bin" };

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification

    // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type);

    // rotation matrix and translation vector
    cv::Mat RT(4, 4, cv::DataType<double>::type);

    RT.at<double>(0, 0) = 7.533745e-03;
    RT.at<double>(0, 1) = -9.999714e-01;
    RT.at<double>(0, 2) = -6.166020e-04;
    RT.at<double>(0, 3) = -4.069766e-03;
    RT.at<double>(1, 0) = 1.480249e-02;
    RT.at<double>(1, 1) = 7.280733e-04;
    RT.at<double>(1, 2) = -9.998902e-01;
    RT.at<double>(1, 3) = -7.631618e-02;
    RT.at<double>(2, 0) = 9.998621e-01;
    RT.at<double>(2, 1) = 7.523790e-03;
    RT.at<double>(2, 2) = 1.480755e-02;
    RT.at<double>(2, 3) = -2.717806e-01;
    RT.at<double>(3, 0) = 0.0;
    RT.at<double>(3, 1) = 0.0;
    RT.at<double>(3, 2) = 0.0;
    RT.at<double>(3, 3) = 1.0;

    R_rect_00.at<double>(0, 0) = 9.999239e-01;
    R_rect_00.at<double>(0, 1) = 9.837760e-03;
    R_rect_00.at<double>(0, 2) = -7.445048e-03;
    R_rect_00.at<double>(0, 3) = 0.0;
    R_rect_00.at<double>(1, 0) = -9.869795e-03;
    R_rect_00.at<double>(1, 1) = 9.999421e-01;
    R_rect_00.at<double>(1, 2) = -4.278459e-03;
    R_rect_00.at<double>(1, 3) = 0.0;
    R_rect_00.at<double>(2, 0) = 7.402527e-03;
    R_rect_00.at<double>(2, 1) = 4.351614e-03;
    R_rect_00.at<double>(2, 2) = 9.999631e-01;
    R_rect_00.at<double>(2, 3) = 0.0;
    R_rect_00.at<double>(3, 0) = 0;
    R_rect_00.at<double>(3, 1) = 0;
    R_rect_00.at<double>(3, 2) = 0;
    R_rect_00.at<double>(3, 3) = 1;

    P_rect_00.at<double>(0, 0) = 7.215377e+02;
    P_rect_00.at<double>(0, 1) = 0.000000e+00;
    P_rect_00.at<double>(0, 2) = 6.095593e+02;
    P_rect_00.at<double>(0, 3) = 0.000000e+00;
    P_rect_00.at<double>(1, 0) = 0.000000e+00;
    P_rect_00.at<double>(1, 1) = 7.215377e+02;
    P_rect_00.at<double>(1, 2) = 1.728540e+02;
    P_rect_00.at<double>(1, 3) = 0.000000e+00;
    P_rect_00.at<double>(2, 0) = 0.000000e+00;
    P_rect_00.at<double>(2, 1) = 0.000000e+00;
    P_rect_00.at<double>(2, 2) = 1.000000e+00;
    P_rect_00.at<double>(2, 3) = 0.000000e+00;

    // misc
    constexpr double sensorFrameRate{ 10.0 / imgStepWidth }; // frames per second for Lidar and camera

    // no. of images which are held in memory (ring buffer) at the same time
    constexpr int dataBufferSize{ 2 };

    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis{ false };                // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    constexpr size_t numImages{ imgEndIndex - imgStartIndex };

    for (auto &info : timingInfo) {
        dataBuffer.clear();

        std::cout << "===== SETTINGS: " << " =====" << std::endl;
        std::cout << "......................" << std::endl;
        std::cout << "DETECTOR Type := " << info.detectorType << std::endl;
        std::cout << "DESCRIPTOR Type := " << info.descriptorType << std::endl;
        std::cout << "MATCHER Type := " << info.matcherType << std::endl;
        std::cout << "SELECTOR Type := " << info.selectorType << std::endl;
        std::cout << "......................" << std::endl;
        std::cout << std::endl;

        for (size_t imgIndex{ 0 }; imgIndex <= numImages; imgIndex += imgStepWidth) {
            DetectionData data;

            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            std::ostringstream imgNumber;

            imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;

            const std::string imgFullFilename{ imgBasePath + imgPrefix + imgNumber.str() + imgFileType };

            cv::Mat img{ cv::imread(imgFullFilename) };

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = img;
            dataBuffer.push_back(frame);

            /* DETECT & CLASSIFY OBJECTS */

            constexpr float confThreshold{ 0.2 };
            constexpr float nmsThreshold{ 0.4 };

            detectObjects((end(dataBuffer) - 1)->cameraImg,
                          (end(dataBuffer) - 1)->boundingBoxes,
                          confThreshold,
                          nmsThreshold,
                          yoloBasePath,
                          yoloClassesFile,
                          yoloModelConfiguration,
                          yoloModelWeights,
                          bVis);

            /* CROP LIDAR POINTS */

            // load 3D Lidar points from file
            const std::string lidarFullFilename{ imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType };
            std::vector<LidarPoint> lidarPoints;

            loadLidarFromFile(lidarPoints, lidarFullFilename);

            // remove Lidar points based on distance properties
            constexpr float minZ{ -1.5 }, maxZ{ -0.9 };
            constexpr float minX{ 2.0 }, maxX{ 20.0 };
            constexpr float maxY{ 2.0 }, minR{ 0.1 }; // focus on ego lane

            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

            (end(dataBuffer) - 1)->lidarPoints = lidarPoints;

            /* CLUSTER LIDAR POINT CLOUD */

            // associate Lidar points with camera-based ROI
            // shrinks each bounding box by the given percentage to avoid 3D object
            // merging at the edges of an ROI
            constexpr float shrinkFactor{ 0.10 };

            clusterLidarWithROI((end(dataBuffer) - 1)->boundingBoxes,
                                (end(dataBuffer) - 1)->lidarPoints,
                                shrinkFactor, P_rect_00, R_rect_00, RT);

            // Visualize 3D objects
            bVis = false;

            if (bVis) {
                show3DObjects((end(dataBuffer) - 1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
            }

            bVis = false;

            /* DETECT IMAGE KEYPOINTS */

            // convert current image to grayscale
            cv::Mat imgGray;
            cv::cvtColor((end(dataBuffer) - 1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

            // extract 2D keypoints from current image
            std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

            if (info.detectorType.compare("SHITOMASI") == 0) {
                data = detKeypointsShiTomasi(keypoints, imgGray, false);

            } else if (info.detectorType.compare("HARRIS") == 0) {
                data = detKeypointsHarris(keypoints, imgGray, false);

            } else {
                data = detKeypointsModern(keypoints, imgGray, info.detectorType, false);
            }

            info.ptsPerFrame.at(imgIndex) = data.numKeyPoints;

            // optional : limit number of keypoints (helpful for debugging and learning)
            constexpr bool bLimitKpts{ false };

            if (bLimitKpts) {
                constexpr int maxKeypoints{ 50 };

                // there is no response info, so keep the first 50
                // as they are sorted in descending quality order
                if (info.detectorType.compare("SHITOMASI") == 0) {
                    keypoints.erase(begin(keypoints) + maxKeypoints, end(keypoints));
                }

                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (end(dataBuffer) - 1)->keypoints = keypoints;

            /* EXTRACT KEYPOINT DESCRIPTORS */

            cv::Mat descriptors;

            data = descKeypoints((end(dataBuffer) - 1)->keypoints,
                          (end(dataBuffer) - 1)->cameraImg,
                          descriptors,
                          info.descriptorType);

            // push descriptors for current frame to end of data buffer
            (end(dataBuffer) - 1)->descriptors = descriptors;

            // wait until at least two images have been processed
            if (dataBuffer.size() > 1) {

                /* MATCH KEYPOINT DESCRIPTORS */

                std::vector<cv::DMatch> matches;
                const std::string descriptorFamily{ (info.descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY" };

                data = matchDescriptors((end(dataBuffer) - 2)->keypoints,
                                 (end(dataBuffer) - 1)->keypoints,
                                 (end(dataBuffer) - 2)->descriptors,
                                 (end(dataBuffer) - 1)->descriptors,
                                 matches,
                                 descriptorFamily,
                                 info.matcherType,
                                 info.selectorType);

                info.matchedPts.at(imgIndex) = data.numKeyPoints;

                // store matches in current data frame
                (end(dataBuffer) - 1)->kptMatches = matches;

                /* TRACK 3D OBJECT BOUNDING BOXES */

                //// STUDENT ASSIGNMENT
                //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous
                /// frame (implement ->matchBoundingBoxes)
                std::map<int, int> bbBestMatches;

                matchBoundingBoxes(matches,
                                   bbBestMatches,
                                   *(end(dataBuffer) - 2),
                                   *(end(dataBuffer) - 1));

                // associate bounding boxes between current and previous frame using keypoint matches
                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                (end(dataBuffer) - 1)->bbMatches = bbBestMatches;

                /* COMPUTE TTC ON OBJECT IN FRONT */

                // loop over all BB match pairs

                for (auto it1{ (end(dataBuffer) - 1)->bbMatches.begin() }; it1 != (end(dataBuffer) - 1)->bbMatches.end(); ++it1) {
                    // find bounding boxes associates with current match
                    BoundingBox *prevBB, *currBB;

                    for (auto it2{ (dataBuffer.end() - 1)->boundingBoxes.begin() }; it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2) {
                        // check wether current match partner corresponds to this BB
                        if (it1->second == it2->boxID) { currBB = &(*it2); }
                    }

                    for (auto it2{ (dataBuffer.end() - 2)->boundingBoxes.begin() }; it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2) {
                        // check wether current match partner corresponds to this BB
                        if (it1->first == it2->boxID) { prevBB = &(*it2); }
                    }

                    // compute TTC for current match
                    // only compute TTC if we have Lidar points

                    if (currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0) {// only compute TTC if we have Lidar points
                        //// STUDENT ASSIGNMENT
                        //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)

                        double ttcLidar;

                        computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);

                        info.ttcLidar.at(imgIndex) = ttcLidar;
                        //// EOF STUDENT ASSIGNMENT

                        //// STUDENT ASSIGNMENT
                        //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement ->
                        /// clusterKptMatchesWithROI) / TASK FP.4 -> compute time-to-collision based on camera
                        ///(implement -> computeTTCCamera)

                        double ttcCamera;

                        clusterKptMatchesWithROI(*currBB,
                                                 (end(dataBuffer) - 2)->keypoints,
                                                 (end(dataBuffer) - 1)->keypoints,
                                                 (end(dataBuffer) - 1)->kptMatches);

                        computeTTCCamera((end(dataBuffer) - 2)->keypoints,
                                         (end(dataBuffer) - 1)->keypoints,
                                         currBB->kptMatches,
                                         sensorFrameRate,
                                         ttcCamera);

                        info.ttcCamera.at(imgIndex) = ttcCamera;
                        //// EOF STUDENT ASSIGNMENT
                        info.enoughLidarOrCameraPointsDetected.at(imgIndex) = true;

                        if (bVis) {
                            cv::Mat visImg{ (end(dataBuffer) - 1)->cameraImg.clone() };

                            showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);

                            cv::rectangle(visImg,
                                          cv::Point(currBB->roi.x, currBB->roi.y),
                                          cv::Point(currBB->roi.x + currBB->roi.width,
                                          currBB->roi.y + currBB->roi.height),
                                          cv::Scalar(0, 255, 0),
                                          2);

                            char str[200];

                            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);

                            putText(visImg,
                                    str,
                                    cv::Point2f(80, 50),
                                    cv::FONT_HERSHEY_PLAIN, 2,
                                    cv::Scalar(0, 0, 255));

                            constexpr char windowName[]{ "Final Results : TTC" };

                            cv::namedWindow(windowName, 4);
                            cv::imshow(windowName, visImg);
                            std::cout << "Press key to continue to next frame" << std::endl;
                            cv::waitKey(0);

                        }

                    } else {
                        info.ttcCamera.at(imgIndex) = info.ttcLidar.at(imgIndex) = 0.0;
                        info.enoughLidarOrCameraPointsDetected.at(imgIndex) = false;
                    }
                }
            }

            std::cout << "IMAGE PROCESSED: " << imgFullFilename << std::endl;
        }

        std::cout << "======================\n" << std::endl;
    }

    createCSVOutputFile(timingInfo, numImages);

    return 0;
}
