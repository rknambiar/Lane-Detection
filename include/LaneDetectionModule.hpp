/**
 *  Copyright 2018 Rohitkrishna Nambiar
 *  @file    LaneDetectionModule.cpp
 *  @author  rohith517
 *  @date    10/12/2018
 *  @version 1.0
 *
 *  @brief Lane Detection
 *
 *  @Description DESCRIPTION
 *
 *  Class member functions for LaneDetectionModule.h
 *
 */

#ifndef INCLUDE_LANEDETECTIONMODULE_HPP_
#define INCLUDE_LANEDETECTIONMODULE_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "Lane.hpp"

class LaneDetectionModule {
 public:
  /**
   *   @brief Default constructor for LaneDetectionModule
   *
   */
  LaneDetectionModule();

  /**
   *   @brief Default destructor for LaneDetectionModule
   *
   */
  ~LaneDetectionModule();

  /**
   *   @brief Method Undistortedimage for LaneDetectionModule
   *
   *   @param src is a Matrix of source of image
   *   @param dst is a Matrix of destination of image
   */
  void undistortImage(const cv::Mat& src, cv::Mat& dst);

  /**
   *   @brief Method thresholdImageY to set
   *          yellow threshold image for LaneDetectionModule
   *
   *   @param src is a Matrix of source of image
   *   @param dst is a Matrix of destination of imageg
   */
  void thresholdImageY(const cv::Mat& src, cv::Mat& dst);

  /**
   *   @brief Method thresholdImageW to set
   *      white threshold image for LaneDetectionModule
   *
   *   @param src is a Matrix of source of image
   *   @param dst is a Matrix of destination of image
   */
  void thresholdImageW(const cv::Mat& src, cv::Mat& dst);

  /**
   *   @brief Method extractROI to set
   *          region of interest for LaneDetectionModule
   *
   *   @param src is a Matrix of source of image
   *   @param dst is a Matrix of destination of image
   */
  void extractROI(const cv::Mat& src, cv::Mat& dst);

  /**
   *   @brief Method transforming perspective of lane image
   *
   *
   *   @param src is a Matrix of source of image
   *   @param dst is a Matrix of destination of image
   *   @param Tm is the transformation matrix for perspective
   *   @param invTm is the inverse of the transformation matrix
   */
  void transformPerspective(const cv::Mat& src, cv::Mat& dst, cv::Mat& Tm,
                            cv::Mat& invTm);

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
  void extractLanes(const cv::Mat& src, cv::Mat& dst, Lane& lane1, Lane& lane2,
                    int curveFlag);

  /**
   *   @brief Method fitPoly fits a 2nd order polynomial to the points
   *   on the lane
   *
   *   @param src is the input image from previous step
   *   @param dst is the destination Mat to store the coefficients of the
   *          polynomial
   *   @param order is the order of polynomial
   */
  void fitPoly(const std::vector<cv::Point>& src, cv::Mat& dst, int order);

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
  double getDriveHeading(Lane& lane1, Lane& lane2, std::string& direction);

  /**
   *   @brief Method displayOutput to calculate
   *        to display of the system
   *        for LaneDetectionModule.
   *
   *   @param src is a Matrix of source of image
   *   @param src2 is the source color image
   *   @param dst is the output image
   *   @param lane1 object of class lane to store line characteristic.
   *   @param lane2 object of class lane to store line characteristic
   *   @param inv is the inverse perspective transformation matrix
   */
  void displayOutput(const cv::Mat& src, cv::Mat& src2, cv::Mat& dst,
                     Lane& lane1,
                     Lane& lane2, cv::Mat inv);

  /**
   *   @brief Method detectLane check if program is successfully running
   *          gives bool output for LaneDetectionModule
   *
   *   @param videoName is video of source
   *
   *   @return Status of lane detection.
   */
  bool detectLane(std::string videoName);

  /**
   *   @brief Method getYellowMax is to use get HSL max value of yellow
   *          for LaneDetectionModule
   *
   *   @return HSL values for yellow lane.
   */
  cv::Scalar getYellowMax();

  /**
   *   @brief Method getYellowMin is to use get HSL min value of yellow
   *          for LaneDetectionModule
   *
   *   @return HSL values for yellow lane.
   */
  cv::Scalar getYellowMin();

  /**
   *   @brief Method setYellowMax is to use set HSL max value of yellow
   *          for LaneDetectionModule
   *
   *   @param  HSL values for yellow lane
   */
  void setYellowMax(cv::Scalar value);

  /**
   *   @brief Method setYellowMin is to use set min value of yellow
   *          for LaneDetectionModule
   *
   *   @param  HSL values for yellow lane.
   */
  void setYellowMin(cv::Scalar value);

  /**
   *   @brief Method setGrayScaleMax is to use set max value of Gray scale
   *          value for LaneDetectionModule
   *
   *   @param  int of max GrayScale value.
   */
  void setGrayScaleMax(int value);

  /**
   *   @brief Method setGrayScaleMin is to use set min value of Gray scale
   *   value for LaneDetectionModule
   *
   *   @param  int of min GrayScale value
   */
  void setGrayScaleMin(int value);

  /**
   *   @brief Method getGrayScaleMin is to use get min value of GrayScale
   *          for LaneDetectionModule
   *
   *   @return int of min GrayScale value
   */
  int getGrayScaleMin();

  /**
   *   @brief Method getGrayScaleMax is to use get max value of GrayScale
   *   for LaneDetectionModule
   *
   *   @return int of max GrayScale values
   */
  int getGrayScaleMax();

 private:
  cv::Scalar yellowMin;  // max possible RGB values of yellow
  cv::Scalar yellowMax;  // min possible RGB values of yellow
  int grayscaleMin;  // min possible grayscale value for white in our video
  int grayscaleMax;  // max possible grayscale value for white in our video
  std::string videoName;  // specify video name
};

#endif  // INCLUDE_LANEDETECTIONMODULE_HPP_
