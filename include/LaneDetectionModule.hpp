/**
 *  Copyright 2018 Rohitkrishna Nambiar
 *  @file    LaneDetectionModule.cpp
 *  @author  rohith517
 *  @date    10/12/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Week 5,Midterm Project.
 *
 *  @Description DESCRIPTION
 *
 *  Class member functions for LaneDetectionModule.h
 *
 */

#ifndef LANEDETECTIONMODULE_H_
#define LANEDETECTIONMODULE_H_
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "Lane.hpp"

class LaneDetectionModule {
public:
  /**
   *   @brief Default constructor for LaneDetectionModule
   *
   *
   *   @param nothing
   *   @return nothing
   */
  LaneDetectionModule();

  /**
   *   @brief Default destructor for LaneDetectionModule
   *
   *
   *   @param nothing
   *   @return nothing
   */
  ~LaneDetectionModule();

  /**
	   *   @brief Method Undistortedimage for LaneDetectionModule
	   *
	   *
	   *   @param src is a Matrix of source of image
	   *   @param dst is a Matrix of destination of image
	   *   @return nothing
	   */
	void undistortImage(const cv::Mat& src,cv::Mat& dst);

  /**
	   *   @brief Method thresholdImageY to set
	   *   		  yellow threshold image for LaneDetectionModule
	   *
	   *
	   *   @param src is a Matrix of source of image
	   *   @param dst is a Matrix of destination of image
	   *   @return nothing
	   */
	void thresholdImageY(const cv::Mat& src,cv::Mat& dst);

  /**
	   *   @brief Method thresholdImageW to set
	   *   		white threshold image for LaneDetectionModule
	   *
	   *
	   *   @param src is a Matrix of source of image
	   *   @param dst is a Matrix of destination of image
	   *   @return nothing
	   */
	void thresholdImageW(const cv::Mat& src,cv::Mat& dst);

  /**
   *   @brief Method extractROI to set
   *   		  region of interest for LaneDetectionModule
   *
   *
   *   @param src is a Matrix of source of image
   *   @param dst is a Matrix of destination of image
   *   @return nothing
   */
	void extractROI(const cv::Mat& src,cv::Mat& dst);

  /**
   *   @brief Method transforming perspective of lane image
   *
   *
   *   @param src is a Matrix of source of image
   *   @param dst is a Matrix of destination of image
   *   @return nothing
   */
  void transformPerspective(const cv::Mat& src, cv::Mat& dst, cv::Mat& Tm,
                            cv::Mat& invTm);

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
  void extractLanes(const cv::Mat& src, cv::Mat& dst, Lane& lane1, Lane& lane2,
                    int curveFlag);

  void extractLanes(const cv::Mat& src, int curveFlag);

  void fitPoly(const std::vector<cv::Point>& src, cv::Mat&, int order);

  /**
	   *   @brief Method getDriveHeading to calculate
	   *   		  drive heading to be given to actuator for further action
	   *   		  in LaneDetectionModule.
	   *
	   *   @param lane1 object of class lane to store line characteristic.
	   *   @param lane2 object of class lane to store line characteristic.
	   *   @return double value of drive head.
	   */
	double getDriveHeading(Lane& lane1,Lane& lane2);

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
	void displayOutput(const cv::Mat& src,Lane& lane1,Lane& lane2,double heading);

  /**
	   *   @brief Method detectLane check if program is successfully running
	   *   		  gives bool output for LaneDetectionModule
	   *
	   *   @param videoName is video of source
	   *   @return bool for code working.
	   */
	bool detectLane(std::string videoName);

  /**
	   *   @brief Method getYellowMax is to use get max value of yellow
	   *   for LaneDetectionModule
	   *
	   *   @param nothing
	   *   @return Scalar of RGB set values.
	   */
	cv::Scalar getYellowMax();

  /**
	   *   @brief Method getYellowMin is to use get min value of yellow
	   *   for LaneDetectionModule
	   *
	   *   @param nothing
	   *   @return Scalar of RGB set values.
	   */
	cv::Scalar getYellowMin();

  /**
	   *   @brief Method setYellowMax is to use set max value of yellow
	   *   for LaneDetectionModule
	   *
	   *   @param  Scalar of RGB values
	   *   @return nothing.
	   */
  void setYellowMax(cv::Scalar value);

  /**
	   *   @brief Method setYellowMin is to use set min value of yellow
	   *   for LaneDetectionModule
	   *
	   *   @param  Scalar of RGB values
	   *   @return nothing.
	   */
  void setYellowMin(cv::Scalar value);

  /**
	   *   @brief Method setGrayScaleMax is to use set max value of Gray scale
	   *   value for LaneDetectionModule
	   *
	   *   @param  int of GrayScale values
	   *   @return nothing.
	   */
	void setGrayScaleMax(int value);

  /**
	   *   @brief Method setGrayScaleMin is to use set min value of Gray scale
	   *   value for LaneDetectionModule
	   *
	   *   @param  int of GrayScale values
	   *   @return nothing.
	   */
	void setGrayScaleMin(int value);

  /**
	   *   @brief Method getGrayScaleMin is to use get min value of GrayScale
	   *   for LaneDetectionModule
	   *
	   *   @param nothing
	   *   @return int of GrayScale values
	   */
	int getGrayScaleMin();

  /**
	   *   @brief Method getGrayScaleMax is to use get max value of GrayScale
	   *   for LaneDetectionModule
	   *
	   *   @param nothing
	   *   @return int of GrayScale values
	   */
	int getGrayScaleMax();

private:
	cv::Scalar yellowMin;  // max possible RGB values of yellow
	cv::Scalar yellowMax;  // min possible RGB values of yellow
	int grayscaleMin;  // min possible grayscale value for white in our video
	int grayscaleMax;  // max possible grayscale value for white in our video
	std::string videoName;  // specify video name
};

#endif /* LANEDETECTIONMODULE_H_ */
