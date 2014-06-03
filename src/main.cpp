/**
 * Automatic camera intrinsics finder.
 *
 * Markus Konrad <post (*AT*) mkonrad (*DOT*) net>, June 2014.
 *
 * Contains many code parts from
 * opencv/samples/tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
 * See http://docs.opencv.org/trunk/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
 */

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


/** DEFINES **/

#define DEVICE_STRLEN        256
#define DEVICE_PATH_STRLEN   512


/** GLOBAL VARS **/

float square_size = 0.0f;
char device[DEVICE_STRLEN];
bool all_devices = true;

bool graphical_disp = false;
bool interactive = false;

Size board_size(9,6);   // number of *inner* corners per a chessboard row and column

bool status_ok = true;

vector<vector<Point2f> > imagePoints;
Mat cameraMatrix, distCoeffs;
Size imageSize;


/** FUNCTIONS **/

void printHelp() {
    cout << "usage:" << endl;
    cout << "cam_intrinsics-db [-g|i] <square size in meters> [device]" << endl;
    cout << " use '-g' for graphical output" << endl;
    cout << " use '-i' for *interactive* graphical output" << endl;
    cout << " optionally specify a 'device' for which calibration photos or videos exist in the 'device_data/' folder" << endl;
}

void err(const char *msg) {
    cerr << "ERROR: " << msg << endl;
    status_ok = false;
}

bool calibrate_with_img(Mat& img) {
    vector<Point2f> pointBuf;

    bool found = findChessboardCorners(img, board_size, pointBuf,
                                       CV_CALIB_CB_ADAPTIVE_THRESH
                                       | CV_CALIB_CB_FAST_CHECK
                                       | CV_CALIB_CB_NORMALIZE_IMAGE);
                                       
    if (!found) {
        return false;
    }
    
    Mat imgGray;
    cvtColor(img, imgGray, COLOR_BGR2GRAY);
    cornerSubPix(imgGray, pointBuf, Size(11,11),
                 Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

    if (graphical_disp) {
        drawChessboardCorners(img, board_size, Mat(pointBuf), found);
    }
    
    return true;
}

void calibrate_device(const char *device) {
    cout << "calibrating device '" << device << "'..." << endl;
    
    char path[DEVICE_PATH_STRLEN];
    sprintf(path, "./device_data/%s", device);
    
    DIR *dir = opendir(path);
    if (dir == NULL) {
        err("could not open the device's photos directory");
        return;
    }
    
    struct dirent *dir_item;
    
    while ((dir_item = readdir(dir)) != NULL) {
        if (!dir_item->d_name
         || dir_item->d_name[0] == '.'
         || dir_item->d_type != DT_REG)
            continue;
        
        char file[DEVICE_PATH_STRLEN];
        sprintf(file, "%s/%s", path, dir_item->d_name);
        
        cout << "> working with file '" << file << "'" << endl;
        
        Mat img = imread(file, CV_LOAD_IMAGE_COLOR);
        
        if (!img.data || img.rows == 0 || img.cols == 0) {
            err("image could not be loaded");
            return;
        }
        
        bool res = calibrate_with_img(img);
        
        cout << ">> calibration: " << (res ? "ok" : "failed") << endl;
        
        if (res && graphical_disp) {
            imshow("image view", img);
            if (interactive) waitKey(0);
        }
    }
}

void calibrate_all() {
    DIR *dir = opendir("./device_data");
    if (dir == NULL) {
        err("could not open photos directory");
        return;
    }
    
    struct dirent *dir_item;
    
    while (status_ok && ((dir_item = readdir(dir)) != NULL)) {
        if (!dir_item->d_name
         || dir_item->d_name[0] == '.'
         || dir_item->d_type != DT_DIR)
            continue;
    
        calibrate_device(dir_item->d_name);
//        cout << "file: " << dir_item->d_name << endl;
//        cout << " is folder: " << (dir_item->d_type == DT_DIR) << endl;
    }
    
    closedir(dir);
}

void write_output() {

}

int main(int argc, char *argv[]) {
    if (argc < 2 || strlen(argv[1]) <= 1) {
        printHelp();
        return 1;
    }
    
    int params_idx = 1;
        
    if (strlen(argv[params_idx]) == 2 && argv[params_idx][0] == '-') {    // optional flag parameter
        if (argv[params_idx][1] == 'g') {
            graphical_disp = true;
        } else if (argv[params_idx][1] == 'i') {
            graphical_disp = true;
            interactive = true;
        }
        
        if (argc <= params_idx + 1) {
            printHelp();
            return 1;
        }
        
        params_idx++;
    }
    
    square_size = (float)strtod(argv[params_idx], NULL);
    params_idx++;
    
    if (square_size <= 0.0f) {
        err("unable to parse first argument as square size");
        printHelp();
        
        return 2;
    }
    
    if (argc > params_idx) {
        strncpy(device, argv[params_idx], DEVICE_STRLEN);
        all_devices = false;
    }
    
    cout << "using square size of " << square_size << " meters" << endl;
    cout << "generating camera intrinsics for ";
    if (all_devices) {
        cout << "all devices" << endl;
        
        calibrate_all();
    } else {
        cout << "device '" << device << "'" << endl;
        
        calibrate_device(device);
    }

    if (status_ok) {
        write_output();
    } else {
        cerr << "calibration failed" << endl;
    
        return 2;
    }

    return 0;
}