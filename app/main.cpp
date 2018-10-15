/**
 *  Copyright 2018 Rohitkrishna Nambiar
 *  @file    Lane.cpp
 *  @author  rohith517
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

#include <iostream>
#include "LaneDetectionModule.cpp"
//#include "Lane.h"

int main(int argc, char* argv[]) {
  LaneDetectionModule lm;

  std::cout << "Starting lane detection with video: " << argv[1] << std::endl;
  lm.detectLane(argv[1]);
  return 0;
}
