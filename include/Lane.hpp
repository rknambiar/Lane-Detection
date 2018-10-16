/**
 *  Copyright 2018 Rohitkrishna Nambiar
 *  @file    Lane.hpp
 *  @author  rohit517
 *  @date    10/13/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Week 6, Midterm Project
 *
 *  @section DESCRIPTION
 *
 *  Implementation to lane detection system when a video is provided
 *  it gives output of Drive heading and lane on video.
 *
 */

#ifndef TRAFFIC_LANE_H_
#define TRAFFIC_LANE_H_

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>

class Lane {
 private:
  int polyOrder;  // declare integer for order of line.
  std::string colour;  //set RGB values for colour.
  std::vector<float> polyCoeff;  // Coefficients for equation
  cv::Point startCoordinates;  //Reference coordinates for line.

  // Average center to prevent jumps for entire run
  std::vector<int> averagingCenter;
  int averagingCount;
  int currentAveragingIndex;
  bool status;  //for status for program.

 public:
  /**
   *   @brief Default constructor for Lane
   *          with ployorder,colour,polyCoeff,startCoordinate,status
   *          random values
   *
   *   @param nothing
   *   @return nothing
   */
  Lane();

  /**
   *   @brief Default constructor for Lane
   *          with ployorder,colour,polyCoeff,startCoordinate,status
   *          random values
   *
   *   @param polyOrder is order of fitting polynomial
   *   @param color is the color of lane
   *   @param averagingCount number of values to average
   *   @return nothing
   */
  Lane(int polyOrder, std::string color, int averagingCount);

  /**
   *   @brief Default destructor for Lane
   *   @param nothing
   *   @return nothing
   */
  ~Lane();

  int getStableCenter(int coordinate);

  void setStartCoordinate(cv::Point point);

  cv::Point getStartCoordinate();

  void setStatus(bool flag);

  bool getStatus();

  void setPolyCoeff(cv::Mat coeff);

  std::vector<float> getPolyCoeff();
};

#endif /* TRAFFIC_LANE_H_ */
