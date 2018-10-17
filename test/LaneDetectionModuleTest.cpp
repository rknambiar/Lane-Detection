/************************************************************************
 MIT License

 Copyright (c) 2018 Harsh Kakashaniya,Rohitkrishna Nambiar

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *************************************************************************/

/**
 * @file    LaneDetectionModuleTest.cpp
 * @author  Rohitkrishna Nambiar (rohit517)
 * @date    10/10/2018
 * @version 1.0
 *
 * @brief   Program to test LaneDetectionModule class.
 *
 * @section DESCRIPTION
 *
 * This is a program that tests LaneDetectionModule class.
 */

#include <gtest/gtest.h>
#include "LaneDetectionModule.hpp"
#include "opencv2/opencv.hpp"

// LaneDetectionModule module object
LaneDetectionModule laneModule;

/**
 *@brief Test get grey scale min threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param getGrayScaleMin  Name of the unit test
 */
TEST(GetSetTest, GetGrayScaleMin) {
  int lowTheshold = laneModule.getGrayScaleMin();
  ASSERT_EQ(lowTheshold, 200);
}

/**
 *@brief Test get grey scale max threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param getGrayScaleMax  Name of the unit test
 */
TEST(GetSetTest, GetGrayScaleMax) {
  int highTheshold = laneModule.getGrayScaleMax();
  ASSERT_EQ(highTheshold, 255);
}

/**
 *@brief Test get yellow min threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param getYellowMin     Name of the unit test
 */
TEST(GetSetTest, GetYellowMin) {
  cv::Scalar lowTheshold = laneModule.getYellowMin();
  ASSERT_EQ(lowTheshold, cv::Scalar(20, 100, 100));
}

/**
 *@brief Test get yellow max threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param getYellowMax     Name of the unit test
 */
TEST(GetSetTest, GetYellowMax) {
  cv::Scalar highTheshold = laneModule.getYellowMax();
  ASSERT_EQ(highTheshold, cv::Scalar(30, 255, 255));
}

/**
 *@brief Test set grey min threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param setGrayScaleMin  Name of the unit test
 */
TEST(GetSetTest, SetGrayScaleMin) {
  laneModule.setGrayScaleMin(150);
  int lowTheshold = laneModule.getGrayScaleMin();
  ASSERT_EQ(lowTheshold, 150);
}

/**
 *@brief Test set grey max threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param SetGrayScaleMax  Name of the unit test
 */
TEST(GetSetTest, SetGrayScaleMax) {
  laneModule.setGrayScaleMax(210);
  int highTheshold = laneModule.getGrayScaleMax();
  ASSERT_EQ(highTheshold, 210);
}

/**
 *@brief Test set yellow min threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param setYellowMin     Name of the unit test
 */
TEST(GetSetTest, SetYellowMin) {
  laneModule.setYellowMin(cv::Scalar(50, 50, 50));
  cv::Scalar lowTheshold = laneModule.getYellowMin();
  ASSERT_EQ(lowTheshold, cv::Scalar(50, 50, 50));
}

/**
 *@brief Test set yellow max threshold function.
 *
 *@param GetSetTest       Get set function test
 *@param SetYellowMax     Name of the unit test
 */
TEST(GetSetTest, SetYellowMax) {
  laneModule.setYellowMax(cv::Scalar(150, 150, 150));
  cv::Scalar highTheshold = laneModule.getYellowMax();
  ASSERT_EQ(highTheshold, cv::Scalar(150, 150, 150));
}

/**
 *@brief Test for single image execution..
 *
 *@param FunctionalTest   Get set function test
 *@param TestImage        Name of the unit test
 */
TEST(FunctionalTest, TestImage) {
  bool status = laneModule.detectLane("../input/ColorImage.jpg");
  ASSERT_EQ(status, true);
}

/**
 *@brief Test for false image path.
 *
 *@param FunctionalTest     Get set function test
 *@param TestFalseImagePath Name of the unit test
 */
TEST(FunctionalTest, TestFalseImagePath) {
  bool status = laneModule.detectLane("../input/ColorImage1.jpg");
  ASSERT_EQ(status, false);
}
