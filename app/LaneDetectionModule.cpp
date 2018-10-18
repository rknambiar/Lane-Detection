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

/**
 *   @brief Default constructor for LaneDetectionModule
 */
LaneDetectionModule::LaneDetectionModule() {
  yellowMin = cv::Scalar(20, 100, 100);  // yellow lane min threshold
  yellowMax = cv::Scalar(30, 255, 255);  // yellow lane max threshold
  grayscaleMin = 200;  // white lane min threshold
  grayscaleMax = 255;  // white lane max threshold
  videoName = "xyz.mp4";  // specify default video name
}

/**
 *   @brief Default destructor for LaneDetectionModule
 */
LaneDetectionModule::~LaneDetectionModule() {
}

/**
 *   @brief Method Undistortedimage for LaneDetectionModule
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
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
 *          yellow threshold image for LaneDetectionModule
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 */
void LaneDetectionModule::thresholdImageY(const cv::Mat& src, cv::Mat& dst) {
  // Image to HLS type image for decreasing effect of light intensity.
  cv::cvtColor(src, dst, cv::COLOR_BGR2HLS);

  // use white thresholding values to detect only road lanes
  cv::inRange(dst, yellowMin, yellowMax, dst);
}

/**
 *   @brief Method thresholdImageW to set
 *          white threshold image for LaneDetectionModule
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 */
void LaneDetectionModule::thresholdImageW(const cv::Mat& src, cv::Mat& dst) {
  // Convert to grayscale image
  cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);

  // use white thresholding values to detect only road lanes
  cv::inRange(dst, grayscaleMin, grayscaleMax, dst);
}

/**
 *   @brief Method extractROI to set
 *          region of interest for LaneDetectionModule
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 */
void LaneDetectionModule::extractROI(const cv::Mat& src, cv::Mat& dst) {
  int width = src.cols;  // width of recieved frame.
  int height = src.rows;  // height of recieved frame.

//  mask matrix initialized as zero Matrix of Height and Width of frame matrix.
  cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
  // Make corners points for the mask.
  cv::Point pts[4] = { cv::Point(0, height), cv::Point(width / 2 - 100,
                                                       height / 2 + 50),
      cv::Point(width / 2 + 100, height / 2 + 50), cv::Point(width, height) };

  // Create a polygon
  cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255));

  // And the source thresholded image and mask
  bitwise_and(src, mask, dst);
}

/**
 *   @brief Method transforming perspective of lane image
 *
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 *   @param Tm is the transformation matrix for perspective
 *   @param invTm is the inverse of the transformation matrix
 */
void LaneDetectionModule::transformPerspective(const cv::Mat& src, cv::Mat& dst,
                                               cv::Mat& Tm, cv::Mat& invTm) {
  int w = src.cols;
  int h = src.rows;

  dst = src.clone();

  // Make corners for the transform
  // br, bl, tl, tr -> order
  cv::Point2f start[4] = { cv::Point2f(w, h - 10), cv::Point2f(0, h - 10),
      cv::Point2f(546, 460), cv::Point2f(732, 460) };

  cv::Point2f end[4] = { cv::Point2f(w, h), cv::Point2f(0, h), cv::Point2f(0,
                                                                           0),
      cv::Point2f(w, 0) };

  Tm = getPerspectiveTransform(start, end);
  invTm = getPerspectiveTransform(end, start);

  cv::warpPerspective(src, dst, Tm, cv::Size(w, h));
}

/**
 *   @brief Method extractLanes to calculate
 *          parameters of lines and its characteristics
 *          for LaneDetectionModule.
 *
 *   @param src is a Matrix of source of image
 *   @param dst is a Matrix of destination of image
 *   @param lane1 object of class lane to store line characteristic.
 *   @param lane2 object of class lane to store line characteristic
 *   @param curveFlag to set degree of curve
 */
void LaneDetectionModule::extractLanes(const cv::Mat& src, cv::Mat& colorLane,
                                       Lane& lane1, Lane& lane2,
                                       int curveFlag) {
  int w = src.cols;
  int h = src.rows;

//  std::cout << "Image size to extract with col: " << w << " row: " << h
//            << std::endl;

  // Height to start the sliding windows
  int padding = 10;
  int bottomHeight = h - padding;
  int bottomWidth = w - padding;

  // Convert the gray src image to bgr for plotting colors(3-channel)
//  cv::Mat colorLane;
  cv::cvtColor(src, colorLane, CV_GRAY2BGR);

  // Create a mask for bottom half to get the histograms
  cv::Rect roi(0, h / 2, w, h / 2);
  cv::Mat croppedIm = src(roi);

  // Reduce the input matrix to a single row
  std::vector<double> histogram;
  cv::reduce(croppedIm, histogram, 0, CV_REDUCE_SUM);

  // Split the two vectors for left and right lane
  std::size_t const halfSize = histogram.size() / 2;
  std::vector<double> leftHist(histogram.begin(), histogram.begin() + halfSize);
  std::vector<double> rightHist(histogram.begin() + halfSize, histogram.end());

  // Get the max element in left half
  int maxLeftIndex = std::max_element(leftHist.begin(), leftHist.end())
      - leftHist.begin();

  // Get the max element in left half
  int maxRightIndex = std::max_element(rightHist.begin(), rightHist.end())
      - rightHist.begin();

  maxRightIndex = maxRightIndex + halfSize;

  // Normalize the right lane as there are breaks in the
  // lane-Optional for stability
  maxRightIndex = lane2.getStableCenter(maxRightIndex);

  // Declare vectors of Point for storing lane points
  std::vector<cv::Point> leftLane;
  std::vector<cv::Point> rightLane;

  // Sliding Window approach
  int windowCount = 9;
  int windowHeight = (h - padding) / windowCount;
  int windowWidth = windowHeight * 2;
//  std::cout << "Sliding window parameters: count: " << windowCount
//            << " height: " << windowHeight << " width: " << windowWidth
//            << std::endl;

  // Left Lane ***********************************************************
  int currentHeight = bottomHeight;
  // Assign start coordinates
  lane1.setStartCoordinate(cv::Point(maxLeftIndex, currentHeight));
  for (int i = 0; i < windowCount; i++) {
    // Get the top left and bottom right point to make a rectangle
    int tlX = maxLeftIndex - windowWidth / 2;
    int tlY = currentHeight - windowHeight;
    int brX = maxLeftIndex + windowWidth / 2;
    int brY = currentHeight;

    // boundary checks
    tlX = (tlX < 0) ? padding : tlX;
    tlY = (tlY < 0) ? padding : tlY;

    brX = (brX > w) ? bottomWidth : brX;
    brY = (brY > h) ? bottomHeight : brY;

    cv::Point tl(tlX, tlY);
    cv::Point br(brX, brY);

    cv::rectangle(colorLane, tl, br, cv::Scalar(0, 255, 204), 3);

    // Create a temporary vector to store the x's for next box
    std::vector<int> nextX;
    // Get the location of the white pixels to store in vector
    int pointFlag = 0;
    for (int j = tlX; j <= brX; j++) {
      for (int k = tlY; k <= brY; k++) {
        if (src.at<uchar>(k, j) > 0) {
          colorLane.at<cv::Vec3b>(cv::Point(j, k)) = cv::Vec3b(0, 0, 255);
          if (pointFlag % 10 == 0) {
            leftLane.push_back(cv::Point(k, j));  // Push as row(y), col(x)
          }
          pointFlag++;
          nextX.push_back(j);
        }
      }
    }

    // Get the center of the next bounding box
    if (!nextX.empty()) {
      maxLeftIndex = std::accumulate(nextX.begin(), nextX.end(), 0)
          / nextX.size();
    }
    currentHeight = currentHeight - windowHeight;
  }

  // Right Lane ****************************************************
  currentHeight = bottomHeight;
  // Assign start coordinates
  lane2.setStartCoordinate(cv::Point(maxRightIndex, currentHeight));
  for (int i = 0; i < windowCount; i++) {
    // Get the top left and bottom right point to make a rectangle
    int tlX = maxRightIndex - windowWidth / 2;
    int tlY = currentHeight - windowHeight;
    int brX = maxRightIndex + windowWidth / 2;
    int brY = currentHeight;

    // boundary checks
    tlX = (tlX < 0) ? padding : tlX;
    tlY = (tlY < 0) ? padding : tlY;

    brX = (brX > w) ? bottomWidth : brX;
    brY = (brY > h) ? bottomHeight : brY;

    cv::Point tl(tlX, tlY);
    cv::Point br(brX, brY);

    cv::rectangle(colorLane, tl, br, cv::Scalar(0, 255, 204), 3);

    // Create a temporary vector to store the x's for next box
    std::vector<int> nextX;
    // Get the location of the white pixels to store in vector
    int pointFlag = 0;
    for (int j = tlX; j <= brX; j++) {
      for (int k = tlY; k <= brY; k++) {
        if (src.at<uchar>(k, j) > 0) {
          colorLane.at<cv::Vec3b>(cv::Point(j, k)) = cv::Vec3b(0, 255, 0);
          if (pointFlag % 10 == 0) {
            rightLane.push_back(cv::Point(k, j));  // Push as row(y), col(x)
          }
          pointFlag++;
          nextX.push_back(j);
        }
      }
    }

    // Get the center of the next bounding box
    if (!nextX.empty()) {
      maxRightIndex = std::accumulate(nextX.begin(), nextX.end(), 0)
          / nextX.size();
    }
    currentHeight = currentHeight - windowHeight;
  }

  // Mat objects for lane parameters
  cv::Mat leftLaneParams = cv::Mat::zeros(curveFlag + 1, 1, CV_64F);
  cv::Mat rightLaneParams = cv::Mat::zeros(curveFlag + 1, 1, CV_64F);

//  std::cout << "Right params: " << rightLane.size() << "Left params: "
//            << leftLane.size() << std::endl;

  // Call the fitpoly function
  fitPoly(leftLane, leftLaneParams, curveFlag);
  fitPoly(rightLane, rightLaneParams, curveFlag);

  // Assign to the lane object and set params
  lane1.setPolyCoeff(leftLaneParams);
  lane2.setPolyCoeff(rightLaneParams);

  // Assign status
  lane1.setStatus(true);
  lane2.setStatus(true);

  /* All the plotting part below this line - Not necessary for program working
   * Create a copy of the input src image for plotting purposes
   */
  cv::circle(colorLane, cv::Point(maxLeftIndex, bottomHeight), 10,
             cv::Scalar(0, 0, 125), -1);

  cv::circle(colorLane, cv::Point(maxRightIndex, bottomHeight), 10,
             cv::Scalar(0, 0, 125), -1);

//  cv::imshow("Lane center", colorLane);
}

/**
 *   @brief Method fitPoly fits a 2nd order polynomial to the points
 *   on the lane
 *
 *   @param src is the input image from previous step
 *   @param dst is the destination Mat to store the coefficients of the
 *          polynomial
 *   @param order is the order of polynomial
 */
void LaneDetectionModule::fitPoly(const std::vector<cv::Point>& src,
                                  cv::Mat& dst, int order) {
  cv::Mat x = cv::Mat(src.size(), 1, CV_32F);
  cv::Mat y = cv::Mat(src.size(), 1, CV_32F);

  // Add all the points to the mat
  for (size_t i = 0; i < src.size(); i++) {
    x.at<float>(i, 0) = static_cast<float>(src[i].x);
    y.at<float>(i, 0) = static_cast<float>(src[i].y);
  }

  int npoints = x.checkVector(1);
  int nypoints = y.checkVector(1);
  CV_Assert(npoints == nypoints && npoints >= order + 1);
  cv::Mat_<double> srcX(x), srcY(y);
  cv::Mat_<double> A = cv::Mat_<double>::ones(npoints, order + 1);
  // build A matrix
  for (int y = 0; y < npoints; ++y) {
    for (int x = 1; x < A.cols; ++x)
      A.at<double>(y, x) = srcX.at<double>(y) * A.at<double>(y, x - 1);
  }
  cv::Mat w;
  cv::solve(A, srcY, w, cv::DECOMP_SVD);
  w.convertTo(dst,
              ((x.depth() == CV_64F || y.depth() == CV_64F) ? CV_64F : CV_32F));
}

/**
 *   @brief Method getDriveHeading to calculate
 *          drive heading to be given to actuator for further action
 *          in LaneDetectionModule.
 *
 *   @param lane1 object of class lane to store line characteristic.
 *   @param lane2 object of class lane to store line characteristic.
 *   @param direction is the string object to hold direction left etc.
 *
 *   @return value of drive head.
 */
double LaneDetectionModule::getDriveHeading(Lane& lane1, Lane& lane2,
                                            std::string& direction) {
  double modifiedSlope = 0;
  if (lane1.getStatus() && lane2.getStatus()) {
    // Get lane 1
    std::vector<float> laneOneCoeff = lane1.getPolyCoeff();

    // Variables to take the slope of lane 1
    cv::Point2d top, bottom;
    top.y = 20;
    bottom.y = 700;

    // Get top x co-ordinate for y = 20
    top.x = pow(top.y, 2) * laneOneCoeff[2] + pow(top.y, 1) * laneOneCoeff[1]
        + laneOneCoeff[0];

    // Get bottom x co-ordinate for y = 700;
    bottom.x = pow(bottom.y, 2) * laneOneCoeff[2]
        + pow(bottom.y, 1) * laneOneCoeff[1] + laneOneCoeff[0];

    // Calculate slope
    double slope = atan((top.y - bottom.y) / (top.x - bottom.x)) * 180
        / 3.14159265;

    // We get negative slope to the right and positive slope to the left
    // Thats because of opencv coordinate system
    if (slope > -85 && slope < 0) {
      // Negative case ie turn right. Slope changed to positive for right
      modifiedSlope = 90 + slope;
      direction = "Turn right";
    } else if (slope < 85 && slope > 0) {
      // Positive case ie turn left. Slope changed to negative for left
      modifiedSlope = slope - 90;
      direction = "Turn left";
    } else {
      modifiedSlope = modifiedSlope / 0.5;
      direction = "Head straight";
    }
  }

  return modifiedSlope;
}

/**
 *   @brief Method displayOutput to calculate
 *        to display of the system
 *        for LaneDetectionModule.
 *
 *   @param src is a Matrix of source of image
 *   @param src2 is the source color image
 *   @param lane1 object of class lane to store line characteristic.
 *   @param lane2 object of class lane to store line characteristic
 *   @param inv is the inverse perspective transformation matrix
 */
void LaneDetectionModule::displayOutput(const cv::Mat& src, cv::Mat& src2,
                                        cv::Mat& dst,
                                        Lane& lane1, Lane& lane2, cv::Mat inv) {
  std::vector<int> yaxis = { 15, 50, 100, 150, 200, 250, 300, 350, 400, 450,
      500, 550, 600, 650, 715 };

  cv::Mat dispOutput;
  src.copyTo(dispOutput);
  int w = src.cols;
  int h = src.rows;

  // **** Lane 1 **** //

  // Find the start coordinate
//  cv::Point laneOneSC = lane1.getStartCoordinate();
//  std::cout << "Left lane start coordinate: " << laneOneSC << std::endl;

  // Find the points for drawing polynomial
  std::vector<float> laneOneCoeff = lane1.getPolyCoeff();
//  std::cout << "Left lane params: " << laneOneCoeff[0] << " "
//              << laneOneCoeff[1]
//            << " " << laneOneCoeff[2] << std::endl;

  // Put in the first point of the lane i.e. starting coordinate
  std::vector<cv::Point> laneOnePoints = { };

  // Iterate through all the points
  for (size_t i = 0; i < yaxis.size(); i++) {
    // Calculate the x values for the y's
    int x = pow(yaxis[i], 2) * laneOneCoeff[2]
        + pow(yaxis[i], 1) * laneOneCoeff[1] + laneOneCoeff[0];
    laneOnePoints.push_back(cv::Point(x, yaxis[i]));
    cv::circle(dispOutput, cv::Point(x, yaxis[i]), 10, cv::Scalar(125, 0, 125),
               -1);
    cv::arrowedLine(dispOutput, cv::Point(x + 400, yaxis[i] + 30),
                    cv::Point(x + 400, yaxis[i]), cv::Scalar(255, 255, 0), 6, 8,
                    0, 0.8);
  }

//  std::cout << "Lane left points: \n";
//  for (size_t i = 0; i < laneOnePoints.size(); i++) {
//    std::cout << laneOnePoints[i] << " ";
//  }
//  std::cout << std::endl;

  // PLot the curve
  const cv::Point *pts1 = (const cv::Point*) cv::Mat(laneOnePoints).data;
  int npts1 = cv::Mat(laneOnePoints).rows;
//  std::cout << "Number of left line vertices: " << npts1 << std::endl;
  cv::polylines(dispOutput, &pts1, &npts1, 1, false, cv::Scalar(153, 255, 153),
                10);
  //  delete pts1;

  // **** Lane 2 **** //

  // Find the start coordinate
//  cv::Point laneTwoSC = lane2.getStartCoordinate();
//  std::cout << "Right lane start coordinate: " << laneTwoSC << std::endl;

  // Find the points for drawing polynomial
  std::vector<float> laneTwoCoeff = lane2.getPolyCoeff();
//  std::cout << "Right lane params: " << laneTwoCoeff[0] << " "
//            << laneTwoCoeff[1] << " " << laneTwoCoeff[2] << std::endl;

  // Put in the first point of the lane i.e. starting coordinate
  std::vector<cv::Point> laneTwoPoints = { };

  // Iterate through all the points
  for (size_t i = 0; i < yaxis.size(); i++) {
    // Calculate the x values for the y's
    int x = pow(yaxis[i], 2) * laneTwoCoeff[2]
        + pow(yaxis[i], 1) * laneTwoCoeff[1] + laneTwoCoeff[0];
    laneTwoPoints.push_back(cv::Point(x, yaxis[i]));
    cv::circle(dispOutput, cv::Point(x, yaxis[i]), 10, cv::Scalar(125, 0, 125),
               -1);
  }

//  std::cout << "Lane right points: \n";
//  for (size_t i = 0; i < laneTwoPoints.size(); i++) {
//    std::cout << laneTwoPoints[i] << " ";
//  }
//  std::cout << std::endl;

  // PLot the curve
  const cv::Point *pts2 = (const cv::Point*) cv::Mat(laneTwoPoints).data;
  int npts2 = cv::Mat(laneTwoPoints).rows;
//  std::cout << "Number of right line vertices: " << npts2 << std::endl;
  cv::polylines(dispOutput, &pts2, &npts2, 1, false, cv::Scalar(153, 255, 153),
                10);
  //  delete pts2;

  cv::Mat unwarpedOutput, unwarpedColor;
  cv::warpPerspective(dispOutput, unwarpedOutput, inv, cv::Size(w, h));
  unwarpedOutput.copyTo(unwarpedColor);

  //  std::cout << "Size: " << unwarpedOutput.size() << std::endl;

  // Combine both the output images (Opencv has a method?)
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      if ((unwarpedColor.at<cv::Vec3b>(j, i)[0] == 0)
          && (unwarpedColor.at<cv::Vec3b>(j, i)[1] == 0)
          && (unwarpedColor.at<cv::Vec3b>(j, i)[2] == 0)) {
        unwarpedColor.at<cv::Vec3b>(j, i)[0] = src2.at<cv::Vec3b>(j, i)[0];
        unwarpedColor.at<cv::Vec3b>(j, i)[1] = src2.at<cv::Vec3b>(j, i)[1];
        unwarpedColor.at<cv::Vec3b>(j, i)[2] = src2.at<cv::Vec3b>(j, i)[2];
      }
    }
  }

  // Get drive heading
  std::string direction;
  double heading = getDriveHeading(lane1, lane2, direction);

  // Setting precision to two decimals
  heading = static_cast<int>(100 * heading) / 100.0;
  std::string headStr = std::to_string(heading);
  for (std::string::size_type s = headStr.length() - 1; s > 0; --s) {
    if (headStr[s] == '0')
      headStr.erase(s, 1);
    else
      break;
  }
  std::string result = "Drive angle: " + headStr + " degrees.";
  std::cout << result << " Action: " << direction << std::endl;
  cv::putText(unwarpedColor, result, cv::Point(500, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(unwarpedColor, direction, cv::Point(550, 100),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(unwarpedColor, "Press C to exit.", cv::Point(1000, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
  cv::putText(unwarpedColor, "Lane Detection", cv::Point(75, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(153, 51, 255), 2);

  unwarpedColor.copyTo(dst);
//  imshow("Lane Detection1", unwarpedColor);
}

/**
 *   @brief Method detectLane check if program is successfully running
 *          gives bool output for LaneDetectionModule
 *
 *   @param videoName is video of source
 *
 *   @return Status of lane detection.
 */
bool LaneDetectionModule::detectLane(std::string videoName) {
  cv::VideoCapture cap(videoName);  // open the default camera
  cv::VideoWriter video("LaneDetection.mp4", CV_FOURCC('M', 'J', 'P', 'G'), 20,
                        cv::Size(1280, 720));
  if (!cap.isOpened())  // check if we succeeded
    return 0;

  std::cout << "Starting Smartlane..." << std::endl;
  cv::namedWindow("Lane Detection", 1);

  Lane leftLane(2, "red", 10), rightLane(2, "green", 10);
  int frameNumber = 0;
  bool notlast = true;

  for (;;) {
    cv::Mat frame, whiteThreshold, yellowThreshold, combinedThreshold,
        distColor, gaussianBlurImage, ROIImage, warpedImage, undistortedImage,
        laneColor, finalOutput;
    cap >> frame;

    if (frame.empty()) {
      return true;
    }

    std::cout << "Frame number: " << frameNumber << " : ";
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
    extractLanes(warpedImage, laneColor, leftLane, rightLane, 2);

    // Adding inverse transform to show in final video
    cv::Mat unwarpedOutput;
    cv::warpPerspective(laneColor, unwarpedOutput, invtransformMatrix,
                        cv::Size(1280, 720));

    // Step 7: Display the output
    displayOutput(laneColor, frame, finalOutput, leftLane, rightLane,
                  invtransformMatrix);

    // Display routine
    cv::Mat combined;
//    hconcat(grayscaleImage, ROIImage, combined);
    int looper = (frameNumber / 100);
//    frame.copyTo(combined);
    if (looper == 0 && notlast) {
      frame.copyTo(combined);
      cv::putText(combined, "Step 0. Input Image", cv::Point(350, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(90, 28, 226), 2);
    } else if (looper == 1 && notlast) {
      whiteThreshold.copyTo(combined);
      cv::cvtColor(combined, combined, CV_GRAY2BGR);
      cv::putText(combined, "Step 1. White Thresholding", cv::Point(350, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(90, 28, 226), 2);
      cv::putText(combined, "Apply white threshold to detect right lane.",
                  cv::Point(350, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(90, 28, 226), 2);
    } else if (looper == 2 && notlast) {
      yellowThreshold.copyTo(combined);
      cv::cvtColor(combined, combined, CV_GRAY2BGR);
      cv::putText(combined, "Step 2. Yellow Thresholding in HSL Space",
                  cv::Point(350, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(23, 226, 199), 2);
      cv::putText(combined,
                  "Apply yellow threshold in HSL to detect left lane.",
                  cv::Point(350, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(23, 226, 199), 2);
    } else if (looper == 3 && notlast) {
      combinedThreshold.copyTo(combined);
      cv::cvtColor(combined, combined, CV_GRAY2BGR);
      cv::putText(combined, "Step 3. Combine thresholding",
                  cv::Point(350, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(200, 226, 72), 2);
      cv::putText(combined,
                  "Combine the yellow and white thresholding.",
                  cv::Point(350, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(200, 226, 72), 2);
    } else if (looper == 4 && notlast) {
      gaussianBlurImage.copyTo(combined);
      cv::cvtColor(combined, combined, CV_GRAY2BGR);
      cv::putText(combined, "Step 4. Apply Gaussian Blur", cv::Point(350, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(226, 111, 172), 2);
    } else if (looper == 5 && notlast) {
      ROIImage.copyTo(combined);
      cv::cvtColor(combined, combined, CV_GRAY2BGR);
      cv::putText(combined, "Step 5. Region of Interest Mask",
                  cv::Point(350, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(213, 226, 183), 2);
      cv::putText(combined,
                  "Apply region of interest(ROI) mask on the lower half.",
                  cv::Point(350, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(213, 226, 183), 2);
    } else if (looper == 6 && notlast) {
      warpedImage.copyTo(combined);
      cv::cvtColor(combined, combined, CV_GRAY2BGR);
      cv::putText(combined, "Step 6. Perspective Transform", cv::Point(350, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(79, 226, 84), 2);
      cv::putText(combined, "Apply perspective transform to see a top-view.",
                  cv::Point(350, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(79, 226, 84), 2);
    }
    else if (looper == 7 && notlast) {
      laneColor.copyTo(combined);
      cv::putText(combined, "Step 7. Sliding Window",
                  cv::Point(350, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(255, 255, 255), 2);
      cv::putText(combined,
                  "Apply sliding window approach for lane detection.",
                  cv::Point(350, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(255, 255, 255), 2);
    }
    else if (looper == 8 && notlast) {
      unwarpedOutput.copyTo(combined);
      cv::putText(combined, "Step 8. Project lanes back.", cv::Point(350, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(226, 75, 80), 2);
      cv::putText(combined,
                  "Undo the perspective transform using the inverse matrix",
                  cv::Point(350, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(226, 75, 80), 2);
    }
    else {
      notlast = false;
      finalOutput.copyTo(combined);
    }

    imshow("Lane Detection", combined);
    video.write(combined);
    frameNumber++;
    if (cv::waitKey(30) >= 0)
      break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor. If
  // Everything works without error, return true.
  return true;
}

/**
 *   @brief Method getYellowMax is to use get HSL max value of yellow
 *          for LaneDetectionModule
 *
 *   @return HSL values for yellow lane.
 */
cv::Scalar LaneDetectionModule::getYellowMax() {
  return yellowMax;
}

/**
 *   @brief Method getYellowMin is to use get HSL min value of yellow
 *          for LaneDetectionModule
 *
 *   @return HSL values for yellow lane.
 */
cv::Scalar LaneDetectionModule::getYellowMin() {
  return yellowMin;
}

/**
 *   @brief Method setYellowMax is to use set HSL max value of yellow
 *          for LaneDetectionModule
 *
 *   @param  HSL values for yellow lane
 */
void LaneDetectionModule::setYellowMax(cv::Scalar value) {
  yellowMax = value;
}

/**
 *   @brief Method setYellowMin is to use set min value of yellow
 *          for LaneDetectionModule
 *
 *   @param  HSL values for yellow lane.
 */
void LaneDetectionModule::setYellowMin(cv::Scalar value) {
  yellowMin = value;
}

/**
 *   @brief Method setGrayScaleMax is to use set max value of Gray scale
 *          value for LaneDetectionModule
 *
 *   @param  int of max GrayScale value.
 */
void LaneDetectionModule::setGrayScaleMax(int value) {
  grayscaleMax = value;
}

/**
 *   @brief Method setGrayScaleMin is to use set min value of Gray scale
 *   value for LaneDetectionModule
 *
 *   @param  int of min GrayScale value
 */
void LaneDetectionModule::setGrayScaleMin(int value) {
  grayscaleMin = value;
}

/**
 *   @brief Method getGrayScaleMin is to use get min value of GrayScale
 *          for LaneDetectionModule
 *
 *   @return int of min GrayScale value
 */
int LaneDetectionModule::getGrayScaleMin() {
  return grayscaleMin;
}

/**
 *   @brief Method getGrayScaleMax is to use get max value of GrayScale
 *   for LaneDetectionModule
 *
 *   @return int of max GrayScale values
 */
int LaneDetectionModule::getGrayScaleMax() {
  return grayscaleMax;
}
