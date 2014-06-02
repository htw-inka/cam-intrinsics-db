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

#define DEVICE_STRLEN        256
#define DEVICE_PATH_STRLEN   512

float square_size = 0.0f;
char device[DEVICE_STRLEN];
bool all_devices = true;

bool status_ok = true;

vector<vector<Point2f> > imagePoints;
Mat cameraMatrix, distCoeffs;
Size imageSize;


void printHelp() {
    cout << "required arguments:" << endl;
    cout << "- square size in meters" << endl;
    cout << endl;
    cout << "optional arguments:" << endl;
    cout << "- device" << endl;
}

void err(const char *msg) {
    cerr << "ERROR: " << msg << endl;
    status_ok = false;
}

void calibrate_device(const char *device) {
    cout << "calibrating device '" << device << "'..." << endl;
    
    char path[DEVICE_PATH_STRLEN];
    sprintf(path, "./device_photos/%s", device);
    
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
    }
}

void calibrate_all() {
    DIR *dir = opendir("./device_photos");
    if (dir == NULL) {
        err("could not open photos directory");
        return;
    }
    
    struct dirent *dir_item;
    
    while ((dir_item = readdir(dir)) != NULL) {
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
    if (argc < 2) {
        printHelp();
        
        return 1;
    }
    
    square_size = (float)strtod(argv[1], NULL);
    
    if (square_size <= 0.0f) {
        err("unable to parse first argument as square size");
        printHelp();
        
        return 2;
    }
    
    if (argc >= 3) {
        strncpy(device, argv[2], DEVICE_STRLEN);
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
    
        return 3;
    }

    return 0;
}