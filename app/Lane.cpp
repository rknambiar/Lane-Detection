/**
 *  Copyright 2018 Rohitkrishna Nambiar
 *  @file    Lane.cpp
 *  @author  rohith517
 *  @date    10/12/2018
 *  @version 1.0
 *
 *  @brief Smart-Lane
 *
 *  @section DESCRIPTION
 *
 *  Class member functions for Lane.cpp
 *
 */

#include "Lane.hpp"

/**
 *   @brief Default constructor for Lane
 *          with ployorder,colour,polyCoeff,startCoordinate,status
 *          random values
 *
 *   @param nothing
 *   @return nothing
 */
Lane::Lane() {
//  std::cout << "Default constructor called" << std::endl;
  polyOrder = 1;
  colour = "red";
  averagingCount = 10;
  currentAveragingIndex = 0;
  status = false;
}

/**
 *   @brief Default constructor for Lane
 *          with ployorder,colour,polyCoeff,startCoordinate,status
 *          random values
 *
 *   @param polyOrder is order of fitting polynomial
 *   @param color is the color of lane
 *   @param averagingCount number of values to average
 */
Lane::Lane(int polyOrder, std::string color, int averagingCount) {
//  std::cout << "Lane created with the following values: Polyorder: "
//            << polyOrder << " Color: " << color << " averaging count: "
//            << averagingCount << std::endl;
  this->polyOrder = polyOrder;
  this->colour = color;
  this->averagingCount = averagingCount;
  currentAveragingIndex = 0;
  status = false;
}

/**
 *   @brief Default destructor for Lane
 *
 *   @param nothing
 *   @return nothing
 */
Lane::~Lane() {
}

/*
 *  @brief Calculates the center x-coordinate averageof the first sliding
 *        window
 *
 *  @param coordinate is current x-center co-ordinate
 *
 *  @return averaged x-coordinate
 */
int Lane::getStableCenter(int coordinate) {
  if (currentAveragingIndex < averagingCount) {
    averagingCenter.push_back(coordinate);
    currentAveragingIndex++;
  } else {
    averagingCenter.erase(averagingCenter.begin());
    averagingCenter.push_back(coordinate);
  }

  int average = std::accumulate(averagingCenter.begin(), averagingCenter.end(),
                                0) / averagingCenter.size();
  return average;
}

/*
 *  @brief Sets the center coordinate of the first sliding window
 *
 *  @param point is current center co-ordinate
 *
 */
void Lane::setStartCoordinate(cv::Point point) {
  startCoordinates = point;
}

/*
 *  @brief Sets the status of lane
 *
 *  @param flag is the status of the lane if polynomial found
 */
void Lane::setStatus(bool flag) {
  status = flag;
}

/*
 *  @brief Sets the polynomial coeff of lane
 *
 *  @param coeff is a Mat(1x3) object containing coefficients
 */
void Lane::setPolyCoeff(cv::Mat laneCoeff) {
  polyCoeff.clear();
  if (polyCoeff.empty()) {
    polyCoeff.push_back(laneCoeff.at<float>(0, 0));
    polyCoeff.push_back(laneCoeff.at<float>(1, 0));
    polyCoeff.push_back(laneCoeff.at<float>(2, 0));
  } else {
    std::cout << "Could not insert new elements!" << std::endl;
  }
}

/*
 *  @brief Gets the center coordinate of the first sliding window
 *
 *  @return Center coordinate of the first sliding window(lane)
 */
cv::Point Lane::getStartCoordinate() {
  return startCoordinates;
}

/*
 *  @brief Gets the status of lane
 *
 *  @return status of the lane
 */
bool Lane::getStatus() {
  return status;
}

/*
 *  @brief Gets the polynomial coeff of lane
 *
 *  @return vector containing coefficients
 */
std::vector<float> Lane::getPolyCoeff() {
  return polyCoeff;
}

/*
 *  @brief Sets the polyOrder of lane
 *
 *  @param value is the polyorder of lane
 */
void Lane::setPolyOrder(int value) {
  polyOrder = value;
}

/*
 *  @brief Gets the polyOrder of lane
 *
 *  @return value is the polyorder of lane
 */
int Lane::getPolyOrder() {
  return polyOrder;
}
