#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static void printHelp() {
    cout << "required arguments:" << endl;
    cout << "- square size in meters" << endl;
    cout << endl;
    cout << "optional arguments:" << endl;
    cout << "- device" << endl;
}

int main(int argc, char *argv[]) {
    cout << argc << endl;

    if (argc < 1) {
        printHelp();
        
        return 1;
    }
    
    

    return 0;
}