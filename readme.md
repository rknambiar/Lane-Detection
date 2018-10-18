<h1 align="center"> Lane Detection using OpenCV and C++ </h1>
<p align="center">
<a href="https://travis-ci.org/rohit517/Lane-Detection">
<img src="https://travis-ci.org/rohit517/Lane-Detection.svg?branch=master">
</a>
 <a href='https://coveralls.io/github/rohit517/Lane-Detection?branch=master'><img src='https://coveralls.io/repos/github/rohit517/Lane-Detection/badge.svg?branch=master'/></a>
<a href='https://github.com/rohit517/Lane-Detection/blob/master/LICENSE'><img src='https://img.shields.io/badge/license-MIT-blue.svg'/></a>
</p>
---

<p align="center">
<img src="https://github.com/rohit517/Lane-Detection/blob/master/output/FinalOutput.jpg" width="640" height="360">
</p>

## Overview

Lane detection is an important aspect of self driving vehicles. This project aims to build a lane detection system using OpenCV and C++. 

Algorithm overview:
- Filter and threshold for white and yellow lane in gray and HLS color space respectively.
- Apply region of interest (ROI) mask on the lower half of the image.
- Apply perspective transform to get a top-view of the lane.
- Use sliding window approach to get the left and right lane.
- Undo the perspective transform and apply the lane markings. 

A step by step video demonstrating the above steps can be found [here](https://www.youtube.com/watch?v=7M99dovhx8M). 
The algorithm applied on the complete video can be found [here](https://www.youtube.com/watch?v=zJXv4z-9pBo).

## Dependencies
Lane Detection:

- OpenCV
- cmake
- googletest

## OpenCV installation

Update packages
```
sudo apt-get update
sudo apt-get upgrade
```
Install required dependencies
``` 
sudo apt-get install build-essential checkinstall cmake pkg-config yasm
sudo apt-get install git gfortran
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
 ```
If you are using Ubuntu 14.04
```
sudo apt-get install libtiff4-dev
```
If you are using Ubuntu 16.04
```
sudo apt-get install libtiff5-dev
```

```
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install qt5-default libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
 ```
Optional dependencies
```
sudo apt-get install libprotobuf-dev protobuf-compiler
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```
Clone OpenCV and OpenCV_contrib
```
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.1 
cd ..

git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.1
cd ..
```
Make build directory
```
cd opencv
mkdir build
cd build
```
Run Cmake
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
```
Find out number of CPU cores in your machine
```
nproc

# substitute 4 by output of nproc
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```
For installation related issues.

A complete OpenCV installation guide in Ubuntu can be found [here](http://www.codebind.com/cpp-tutorial/install-opencv-ubuntu-cpp/). 

## Standard install via command-line
```
git clone --recursive https://github.com/rohit517/Lane-Detection.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/smartLane-test
Run program: ./app/shell-app ../input/project_video.mp4
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## For more info

For more details on the algorithm/approach please feel free to reach out to me at rohit517@terpmail.umd.edu.

## License
```
MIT License

Copyright (c) 2018 Rohit

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
```
