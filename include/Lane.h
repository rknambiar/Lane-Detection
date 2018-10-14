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
 public:
  int polyOrder;  // declare integer for order of line.
  std::string colour;  //set RGB values for colour.
  std::vector<double> polyCoeff;  // Coefficients for equation
  cv::Point startCoordinates;  //Reference coordinates for line.
  bool status = false;  //for status for program.

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
   *   @brief Default destructor for Lane
   *   @param nothing
   *   @return nothing
   */
  ~Lane();
};

#endif /* TRAFFIC_LANE_H_ */
