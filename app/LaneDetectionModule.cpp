/**
 *  Copyright 2018 Rohitkrishna Nambiar
 *  @file    LaneDetectionModule.cpp
 *  @author  rohith517)
 *  @date    10/12/2018
 *  @version 1.0
 *
 *  @brief Smart-Lane
 *
 *  @Description DESCRIPTION
 *
 *  Class member functions for LaneDetectionModule.cpp
 *
 */

#include "LaneDetectionModule.hpp"
#include "Lane.cpp"

/**
 *   @brief Default constructor for LaneDetectionModule
 *
 *
 *   @param nothing
 *   @return nothing
 */
LaneDetectionModule::LaneDetectionModule() {
  yellowMin = cv::Scalar(20, 100, 100);  // yellow lane min threshold
  yellowMax = cv::Scalar(30, 255, 255);  // yellow lane max threshold
  grayscaleMin = 200;  // white lane min threshold
  grayscaleMax = 255;  // white lane max threshold
  videoName = "xyz.mp4";  // specify video name
}

/**
 *   @brief Default destructor for LaneDetectionModule
 *
 *
 *   @param nothing
 *   @return nothing
 */
LaneDetectionModule::~LaneDetectionModule() {
}

/**
 *   @brief Method Undistortedimage for LaneDetectionModule
 *
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 *   @return nothing
 */
void LaneDetectionModule::undistortImage(const cv::Mat& src, cv::Mat& dst) {

  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << 1154.22732, 0.0, 671.627794, 0.0, 1148.18221, 386.046312, 0.0, 0.0, 1.0);

  std::vector<double> distortionCoeff { -.242565104, -0.0477893070,
      -0.00131388084, -0.0000879107779, 0.0220573263 };

  cv::undistort(src, dst, cameraMatrix, distortionCoeff);
}

/**
 *   @brief Method thresholdImageY to set
 *   		  yellow threshold image for LaneDetectionModule
 *
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 *   @return nothing
 */
void LaneDetectionModule::thresholdImageY(const cv::Mat& src, cv::Mat& dst) {
  // Convert to grayscale image
  cv::cvtColor(src, dst, cv::COLOR_BGR2HLS);

  // use white thresholding values to detect only road lanes
  cv::inRange(dst, yellowMin, yellowMax, dst);
}

/**
 *   @brief Method thresholdImageW to set
 *   		white threshold image for LaneDetectionModule
 *
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 *   @return nothing
 */
void LaneDetectionModule::thresholdImageW(const cv::Mat& src, cv::Mat& dst) {
  // Convert to grayscale image
  cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);

  // use white thresholding values to detect only road lanes
  cv::inRange(dst, grayscaleMin, grayscaleMax, dst);
}

/**
 *   @brief Method extractROI to set
 *   		  region of interest for LaneDetectionModule
 *
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 *   @return nothing
 */
void LaneDetectionModule::extractROI(const cv::Mat& src, cv::Mat& dst) {
  int width = src.cols;
  int height = src.rows;

  cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
  // Make corners for the mask
  cv::Point pts[4] = { cv::Point(0, height), cv::Point(width / 2 - 100,
                                                       height / 2 + 50),
      cv::Point(width / 2 + 100, height / 2 + 50), cv::Point(width, height) };

  // Create a polygon
  cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255));

  // And the source thresholded image and mask
  bitwise_and(src, mask, dst);
}

void LaneDetectionModule::transformPerspective(const cv::Mat& src,
                                               cv::Mat& dst,
                                               cv::Mat& Tm, cv::Mat& invTm) {
  int w = src.cols;
  int h = src.rows;

  dst = src.clone();

  // Make corners for the transform
  // br, bl, tl, tr -> order
  cv::Point2f start[4] = { cv::Point2f(w, h - 10), cv::Point2f(0, h - 10),
      cv::Point2f(546, 460), cv::Point2f(732, 460) };

  cv::Point2f end[4] = { cv::Point2f(w, h), cv::Point2f(0, h),
      cv::Point2f(0,
                                                                           0),
      cv::Point2f(w, 0) };

  Tm = getPerspectiveTransform(start, end);
  invTm = getPerspectiveTransform(end, start);

  cv::warpPerspective(src, dst, Tm, cv::Size(w, h));

//  circle(dst, end[0], 5, Scalar(0, 0, 125), -1);
//  circle(dst, end[1], 5, Scalar(0, 0, 125), -1);
//  circle(dst, end[2], 5, Scalar(0, 0, 125), -1);
//  circle(dst, end[3], 5, Scalar(0, 0, 125), -1);
}

/**
 *   @brief Method extractLanes to calculate
 *   		  parameters of lines and its characteristics
 *   		  for LaneDetectionModule.
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 *   @param lane1 object of class lane to store line characteristic.
 *   @param lane2 object of class lane to store line characteristic
 *   @param curveFlag to set degree of curve
 *   @return nothing
 */
void LaneDetectionModule::extractLanes(const cv::Mat& src, Lane& lane1,
                                       Lane& lane2, int curveFlag) {
  int w = src.cols;
  int h = src.rows;

  int bottomHeight = h - 0;

  //  std::cout << "Image size to extract with col: " << w << " row: " << h
  //            << std::endl;

  cv::Rect roi(0, h / 2, w, h / 2);
  cv::Mat croppedIm = src(roi);

  // Reduce the input matrix to a single row
  std::vector<double> histogram;
  cv::reduce(croppedIm, histogram, 0, CV_REDUCE_SUM);

  //  std::cout << "Size: " << histogram.size();
  //  for (int i = 0; i < histogram.size(); i++) {
  //    std::cout << "Index: " << i << " Value: " << histogram[i] << std::endl;
  //  }

  // Split the two vectors for left and right lane
  std::size_t const halfSize = histogram.size() / 2;
  std::vector<double> leftHist(histogram.begin(), histogram.begin() + halfSize);
  std::vector<double> rightHist(histogram.begin() + halfSize, histogram.end());

  //  for (int i = 0; i < leftHist.size(); i++) {
  //    std::cout << "Left Index: " << i << " Value: " << leftHist[i] << std::endl;
  //  }
  //
  //  for (int i = 0; i < rightHist.size(); i++) {
  //    std::cout << "Right Index: " << i << " Value: " << rightHist[i]
  //              << std::endl;
  //  }

  // Get the max element in each half
  int maxLeftIndex = std::max_element(leftHist.begin(), leftHist.end())
      - leftHist.begin();
  int maxLeft = *std::max_element(leftHist.begin(), leftHist.end());

  int maxRightIndex = std::max_element(rightHist.begin(), rightHist.end())
      - rightHist.begin();
  int maxRight = *std::max_element(rightHist.begin(), rightHist.end());

  int averagedRight = lane2.getStableCenter(halfSize + maxRightIndex);

//  std::cout << "Left Lane peak index: " << maxLeftIndex;
//  std::cout << "\tRight Lane peak index: " << halfSize + maxRightIndex;
//  std::cout << "\tAveraged Right Lane peak index: " << averagedRight
//            << std::endl;

  cv::Mat test;
  src.copyTo(test);

  // Convert the gray image to RGB
  cv::Mat grayTC;
  cv::cvtColor(test, grayTC, cv::COLOR_GRAY2BGR);

  cv::circle(grayTC, cv::Point(maxLeftIndex, bottomHeight), 10,
             cv::Scalar(0, 0, 125), -1);

  cv::circle(grayTC,
             cv::Point(averagedRight, bottomHeight),
             15,
             cv::Scalar(0, 0, 125), -1);

  cv::circle(grayTC, cv::Point(halfSize + maxRightIndex, bottomHeight), 10,
             cv::Scalar(125, 125, 0), -1);

  // Sliding Window approach
  int windowCount = 9;
  int windowHeight = h / 9;
  int windowWidth = windowHeight * 2;

  // Left Lane)
  std::vector<cv::Point> leftLane;
  for (int i = 0; i < windowCount; i++) {
    // Compute the top left and bottom right vertice of rectangle
    cv::Point tl(maxLeftIndex - windowWidth / 2, bottomHeight - windowHeight);
    cv::Point br(maxLeftIndex + windowWidth / 2, bottomHeight);
    cv::rectangle(grayTC, tl, br, cv::Scalar(0, 255, 204), 1);

    std::vector<int> nextCenter;
    // Get the location of the white pixels in the box
    for (int j = tl.x; j <= br.x; j++) {
      for (int k = tl.y; k <= br.y; k++) {
        if (src.at<uchar>(k, j) > 0) {
          leftLane.push_back(cv::Point(k, j));  // Push as row(y), col(x)
          grayTC.at<cv::Vec3b>(cv::Point(j, k)) = cv::Vec3b(255, 0, 0);
          nextCenter.push_back(j);
//          std::cout << "On" << "X: " << j << " Y: " << k << std::endl;
        }
      }
    }

    // Calculate average of the x co-ordinate for next box
//    std::cout << "nextcenter size: " << nextCenter.size() << std::endl;
    if (!nextCenter.empty()) {
      int average = std::accumulate(nextCenter.begin(), nextCenter.end(), 0)
          / nextCenter.size();
      maxLeftIndex = average;
    }
    bottomHeight = bottomHeight - windowHeight + 1;
  }

  // Right lane
  std::vector<cv::Point> rightLane;

  // reset bottom height
  bottomHeight = h - 0;

  // Loop through the windows
  for (int i = 0; i < windowCount; i++) {
    // Compute the top left and bottom right vertice of rectangle
    cv::Point tl(averagedRight - windowWidth / 2, bottomHeight - windowHeight);
    cv::Point br(averagedRight + windowWidth / 2, bottomHeight);
    cv::rectangle(grayTC, tl, br, cv::Scalar(0, 255, 204), 1);

    std::vector<int> nextCenter;
    // Get the location of the white pixels in the box
    for (int j = tl.x; j <= br.x; j++) {
      for (int k = tl.y; k <= br.y; k++) {
        if (src.at<uchar>(k, j) > 0) {
          rightLane.push_back(cv::Point(k, j));  // Push as row(y), col(x)
          grayTC.at<cv::Vec3b>(cv::Point(j, k)) = cv::Vec3b(255, 0, 0);
          nextCenter.push_back(j);
//          std::cout << "On" << "X: " << j << " Y: " << k << std::endl;
        }
      }
    }

    // Calculate average of the x co-ordinate for next box
//    std::cout << "nextcenter size: " << nextCenter.size() << std::endl;
    if (!nextCenter.empty()) {
      int average = std::accumulate(nextCenter.begin(), nextCenter.end(), 0)
          / nextCenter.size();
      averagedRight = average;
    }
    bottomHeight = bottomHeight - windowHeight + 1;
  }

  cv::imshow("Cropped", grayTC);

//  std::this_thread::sleep_for(std::chrono::seconds(1));

}

/**
 *   @brief Method getDriveHeading to calculate
 *   		  drive heading to be given to actuator for further action
 *   		  in LaneDetectionModule.
 *
 *   @param lane1 object of class lane to store line characteristic.
 *   @param lane2 object of class lane to store line characteristic.
 *   @return double value of drive head.
 */
double LaneDetectionModule::getDriveHeading(Lane& lane1, Lane& lane2) {
  return 0.0;
}

/**
 *   @brief Method displayOutput to calculate
 *   		  to display of the system
 *   		  for LaneDetectionModule.
 *
 *   @param src is a Matrix of source of image
 *   @param lane1 object of class lane to store line characteristic.
 *   @param lane2 object of class lane to store line characteristic
 *   @param heading to get Drive heading for output
 *   @return nothing
 */
void LaneDetectionModule::displayOutput(const cv::Mat& src, Lane& lane1,
 Lane& lane2, double heading) {
}

/**
 *   @brief Method detectLane check if program is successfully running
 *   		  gives bool output for LaneDetectionModule
 *
 *   @param videoName is video of source
 *   @return bool for code working.
 */
bool LaneDetectionModule::detectLane(std::string videoName) {
  cv::VideoCapture cap(videoName);  // open the default camera
  if (!cap.isOpened())  // check if we succeeded
    return -1;

  cv::namedWindow("Lane Detection", 1);
  cv::namedWindow("Cropped", 1);
  cv::moveWindow("Cropped", 20, 20);

  Lane leftLane(2, "red", 10), rightLane(2, "blue", 10);
  int frameNumber = 0;

  for (;;) {
    std::cout << "Frame number: " << frameNumber << std::endl;
    cv::Mat frame, whiteThreshold, yellowThreshold, combinedThreshold,
        gaussianBlurImage, ROIImage, warpedImage, undistortedImage;
    cap >> frame;

    if (frame.empty()) {
      return true;
    }

    // Step 1: Undistort the input image given camera params
    undistortImage(frame, undistortedImage);

    // Step 2: Get white lane
    thresholdImageW(undistortedImage, whiteThreshold);

    // Step 2: Get Yellow lane
    thresholdImageY(undistortedImage, yellowThreshold);

    // Step 2: Combine both the masks
    bitwise_or(whiteThreshold, yellowThreshold, combinedThreshold);

    // Combine mask with grayscale image. Do we need this?
    cv::Mat grayscaleImage, grayscaleMask;
    cv::cvtColor(undistortedImage, grayscaleImage, cv::COLOR_BGR2GRAY);
    bitwise_and(grayscaleImage, combinedThreshold, grayscaleMask);

    // Step 3: Apply Gaussian filter to smooth thresholded image
    cv::GaussianBlur(combinedThreshold, gaussianBlurImage, cv::Size(5, 5), 0,
                     0);

    // Step 4: Extract the region of interest
    extractROI(gaussianBlurImage, ROIImage);

    // Step 5: Transform perspective
    cv::Mat transformMatrix, invtransformMatrix;
    transformPerspective(ROIImage, warpedImage, transformMatrix,
                         invtransformMatrix);

    // Step 6: Get the lane parameters
    // curveFlag = 1: Straight Line
    // curveFlag = 2: 2nd order polynomial fit
//    extractLanes(warpedImage, leftLane, rightLane, 2);
//    extractLanes(warpedImage, 2);

    cv::Mat combined;
    hconcat(grayscaleImage, warpedImage, combined);
    imshow("Lane Detection", combined);

    frameNumber++;
    if (cv::waitKey(30) >= 0)
      break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}

/**
 *   @brief Method getYellowMax is to use get max value of yellow
 *   for LaneDetectionModule
 *
 *   @param nothing
 *   @return Scalar of RGB set values.
 */
cv::Scalar LaneDetectionModule::getYellowMax() {
  return yellowMax;
}

/**
 *   @brief Method getYellowMin is to use get min value of yellow
 *   for LaneDetectionModule
 *
 *   @param nothing
 *   @return Scalar of RGB set values.
 */
cv::Scalar LaneDetectionModule::getYellowMin() {
  return yellowMin;
}

/**
 *   @brief Method setYellowMax is to use set max value of yellow
 *   for LaneDetectionModule
 *
 *   @param  Scalar of RGB values
 *   @return nothing.
 */
void LaneDetectionModule::setYellowMax(cv::Scalar value) {
  yellowMax = value;
}

/**
 *   @brief Method setYellowMin is to use set min value of yellow
 *   for LaneDetectionModule
 *
 *   @param  Scalar of RGB values
 *   @return nothing.
 */
void LaneDetectionModule::setYellowMin(cv::Scalar value) {
  yellowMin = value;
}

/**
 *   @brief Method setGrayScaleMax is to use set max value of Gray scale
 *   value for LaneDetectionModule
 *
 *   @param  int of GrayScale values
 *   @return nothing.
 */
void LaneDetectionModule::setGrayScaleMax(int value) {
  grayscaleMax = value;
}

/**
 *   @brief Method setGrayScaleMin is to use set min value of Gray scale
 *   value for LaneDetectionModule
 *
 *   @param  int of GrayScale values
 *   @return nothing.
 */
void LaneDetectionModule::setGrayScaleMin(int value) {
  grayscaleMin = value;
}

/**
 *   @brief Method getGrayScaleMin is to use get min value of GrayScale
 *   for LaneDetectionModule
 *
 *   @param nothing
 *   @return int of GrayScale values
 */
int LaneDetectionModule::getGrayScaleMin() {
  return grayscaleMin;
}

/**
 *   @brief Method getGrayScaleMax is to use get max value of GrayScale
 *   for LaneDetectionModule
 *
 *   @param nothing
 *   @return int of GrayScale values
 */
int LaneDetectionModule::getGrayScaleMax() {
  return grayscaleMax;
}
