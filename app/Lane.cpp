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
  std::cout << "Default constructor called" << std::endl;
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
 *   @return nothing
 */
Lane::Lane(int polyOrder, std::string color, int averagingCount) {
  std::cout << "Lane created with the following values: Polyorder: "
            << polyOrder << " Color: " << color << " averaging count: "
            << averagingCount << std::endl;
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

int Lane::getStableCenter(int coordinate) {
  if (currentAveragingIndex < averagingCount) {
    averagingCenter.push_back(coordinate);
    currentAveragingIndex++;
  }
  else {
    averagingCenter.erase(averagingCenter.begin());
    averagingCenter.push_back(coordinate);
  }

  int average = std::accumulate(averagingCenter.begin(), averagingCenter.end(),
                                0) / averagingCenter.size();
  return average;
}
