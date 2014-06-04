# Automatic camera intrinsics finder and database

*Markus Konrad <konrad@htw-berlin.de>, June 2014*

*INKA Research Group, HTW Berlin - http://inka.htw-berlin.de*

This repository contains a simple [OpenCV](http://opencv.org)-based automatic camera calibration tool and a database containing XML files with camera matrix and distortion coefficients for different devices that have so far been determined using this tool.

A camera's matrix and distortion coefficients is especially important in Augmented Reality applications, where real a object's 3D pose needs to be approximated from 2D camera images.

For more information, see the excellent [OpenCV tutorial on camera calibration](http://docs.opencv.org/trunk/doc/tutorials/calib3d/camera_calibration/camera_calibration.html).

OpenCV also contains [source code for a camera calibration tool](http://docs.opencv.org/trunk/_downloads/camera_calibration.cpp), which is also the inspiration for this tool (and parts of it's code have been directly used here, too). However, for most cases it is rather bloated and more difficult to use, because it requires to write a XML file with settings first. The presented tool here can be controlled completely with program arguments (see "Usage" below). It can also run "headless", meaning without graphical output, which speeds up the whole process.

## Tool usage

The program can be called in the following way:

```
cam_intrinsics-db [-g|i] <square size in meters> [device]
```

* `square size in meters` is necessary and specifies the side length of the squares in the chessboard grid that was used for calibration
* use `-g` for graphical output (shows original and undistorted first frame)
* use `-i` for *interactive* graphical output (step through all frames)
* optionally specify a 'device' for which calibration photos or videos exist in the 'device_data/' folder; if no device is specified, the program will calibrate *all* devices in the *device_data* folder (with the same square size setting)

## Calibration data

Calibration data, which means images and/or videos with that depict a chessboard grid from different angles, resides for each device in the folder *device_data*. You can download and print out such a grid from the *assets* folder.

## Database

The *database* folder contains an XML file for each camera that was calibrated with this tool so far. The XML files use the [*opencv_storage* format](http://docs.opencv.org/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html) to store the following calibration information:

* `Camera_Matrix` contains the 3x3 camera matrix
* `Distortion_Coefficients` contains the 5 distortion coefficients
* `Avg_Reprojection_Error` is the reported average reprojection error that resulted from the above calibration information

These XML files can be loaded and used in your (AR) program.

## Dependencies

* [OpenCV 2.x](http://opencv.org) headers and library files need to be installed

## Compile

This program can be compiled with any standard C++ compiler, like e.g. *g++*.

To compile you should at first make sure the settings for the header search path and library search path are correct. If necessary, edit the `CFLAGS` and `LDFLAGS` parameters in the Makefile.

Run `make` to compile the program.

You can run `make clean` to remove the object file and the executable.

## Contribute

If you have a camera that you calibrated with this tool, please submit your data set from the *device_data* folder (calibration images / video) and the resulting XML file. You can for example do so by sending a pull request.

Note that you can only use chessboard grids for calibration. You can download and print out such a grid from the *assets* folder.

## TODO

* add the following as optional program arguments:
** board size
** number of frames to use from videos
** calibrateCamera parameters