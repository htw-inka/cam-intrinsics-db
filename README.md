# Automatic camera intrinsics finder and database

This repository contains a simple [OpenCV](http://opencv.org)-based automatic camera calibration tool and a database containing XML files with camera matrix and distortion coefficients for different devices that have so far been determined using this tool.

A camera's matrix and distortion coefficients is especially important in Augmented Reality applications, where real a object's 3D pose needs to be approximated from 2D camera images.

For more information, see the excellent [OpenCV tutorial on camera calibration](http://docs.opencv.org/trunk/doc/tutorials/calib3d/camera_calibration/camera_calibration.html).

OpenCV also contains [source code for a camera calibration tool](http://docs.opencv.org/trunk/_downloads/camera_calibration.cpp), which is also the inspiration for this tool (and parts of it's code have been directly used here, too). However, for most cases it is rather bloated and more difficult to use.

## Dependencies

* [OpenCV 2.x](http://opencv.org) headers and library files need to be installed

## Compile

## Contribute

## TODO

* comments
* add board size as optional program argument
* add number of frames to use from videos as optional program argument
