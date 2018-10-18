/**
 *  Copyright 2018 Rohitkrishna Nambiar
 *  @file    Lane.cpp
 *  @author  rohith517
 *  @date    10/12/2018
 *  @version 1.0
 *
 *  @brief Lane Detection
 *
 *  @Description DESCRIPTION
 *
 *  Class member functions for LaneDetectionModule.cpp
 *
 */

#include "LaneDetectionModule.hpp"

int main(int argc, char* argv[]) {
  LaneDetectionModule lm;

  if (argc != 2)
    std::cout
        << "Did not receive video location as argument. "
              "Format: ./app/shell-app ../input/project-video.mp4"
        << std::endl;
  std::cout << "Starting lane detection with video: " << argv[1] << std::endl;
  bool status = lm.detectLane(argv[1]);
  std::cout << "Smartlane executed with status: " << status << std::endl;
  return 0;
}
