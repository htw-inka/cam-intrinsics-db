#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

#define DEVICE_STRLEN   255

float square_size = 0.0f;
char device[DEVICE_STRLEN];
bool all_devices = true;

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

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printHelp();
        
        return 1;
    }
    
    square_size = (float)strtod(argv[1], NULL);
    
    if (square_size <= 0.0f) {
        cout << "unable to parse first argument as square size" << endl;
        printHelp();
        
        return 2;
    }
    
    if (argc >= 3) {
        strncpy(device, argv[2], DEVICE_STRLEN);
        all_devices = false;
    }
    
    cout << "using square size of " << square_size << " meters" << endl;
    cout << "generating camera intrinsics for ";
    if (all_devices)
        cout << "all devices" << endl;
    else
        cout << "device '" << device << "'" << endl;

    return 0;
}