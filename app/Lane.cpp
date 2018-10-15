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

#include <iostream>
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
  std::cout << "Default constructor" << std::endl;
  polyOrder = 1;
  colour = "red";
  polyCoeff = {0};
  startCoordinates = cv::Point(0, 0);
  averagingCount = 10;
  currentAveragingIndex = 0;
  status = false;
}

Lane::Lane(int polyOrder, std::string color, int averagingCount) {
  this->polyOrder = polyOrder;
  this->colour = color;
  this->averagingCount = averagingCount;
  status = false;
  currentAveragingIndex = 0;
}


/**
 *   @brief Default destructor for Lane
 *
 *   @param nothing
 *   @return nothing
 */
  Lane::~Lane() {
}

int Lane::getStableCenter(int coordinate) {

//  std::cout << "Current val: " << coordinate << "\tavi: "
//            << currentAveragingIndex << "\tavc: " << averagingCount;

  if (currentAveragingIndex < averagingCount) {
    averagingCenter.push_back(coordinate);
    currentAveragingIndex++;
  }
  else {
    averagingCenter.erase(averagingCenter.begin());
    averagingCenter.push_back(coordinate);
  }

  int average = std::accumulate(averagingCenter.begin(), averagingCenter.end(),
                                0)
      / averagingCenter.size();

//  std::cout << "\taverage: " << average << std::endl;

  return average;
}
