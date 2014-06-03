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
#include <cstdlib>
#include <cstring>
#include <string>
#include <algorithm>
#include <sys/types.h>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


/** DEFINES AND CONSTANTS **/

#define VID_USE_NUM_FRAMES      10
#define VID_BAD_FRAME_ATTEMPTS  5
#define VID_BAD_FRAME_SKIP      5

#define DEVICE_STRLEN           256
#define DEVICE_PATH_STRLEN      512

const char ESC_KEY = 27;

enum file_type {
    UNKNOWN,
    PIC,
    VID
};

/** GLOBAL VARS **/

float square_size = 0.0f;
char device[DEVICE_STRLEN];
bool all_devices = true;

VideoCapture vid_cap;

bool graphical_disp = false;
bool interactive = false;

Size board_size(9,6);   // number of *inner* corners per a chessboard row and column

bool status_ok = true;

vector<vector<Point2f> > img_pts;
vector<Point3f> std_obj_pts;

vector<Mat> undistort_imgs;

Mat cam_mat, dist_mat;
double avg_reproj_err = 0.0;
Size img_size(0, 0);

vector<string> file_ext_pic;
vector<string> file_ext_vid;



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

void print_mat(const Mat &m) {
    for (int y = 0; y < m.rows; y++) {
        for (int x = 0; x < m.cols; x++) {
            cout << m.at<double>(y, x);
            
            if (x < m.cols - 1) cout << " ";
        }
        
        cout << endl;
    }
}

enum file_type guess_type(const char *file) {
    // find extension
    const char *last_dot = strrchr(file, '.');
    const int ext_len = strlen(file) - (last_dot - file + 1);
                
    char ext[5];
    strncpy(ext, last_dot + 1, ext_len);
    ext[ext_len] = '\0';
    
    // make lower case
    for (int i = 0; i < ext_len; ++i) ext[i] = tolower(ext[i]);

    // try to find in extension vectors
    vector<string>::iterator found_type = std::find(file_ext_pic.begin(), file_ext_pic.end(), string(ext));
    
    if (found_type != file_ext_pic.end()) return PIC;
    
    found_type = std::find(file_ext_vid.begin(), file_ext_vid.end(), string(ext));
    
    if (found_type != file_ext_vid.end()) return VID;
    
    return UNKNOWN;
}

bool write_output(const char *file) {
    FileStorage fs(file, FileStorage::WRITE);
    
    bool ok = false;
    
    if (fs.isOpened()) {
        fs << "Camera_Matrix" << cam_mat;
        fs << "Distortion_Coefficients" << dist_mat;
        fs << "Avg_Reprojection_Error" << avg_reproj_err;
        
        ok = true;
    }
    
    fs.release();
    
    return ok;
}

bool find_corners_in_img(Mat& img) {
    vector<Point2f> point_buf;

    bool found = findChessboardCorners(img, board_size, point_buf,
                                       CV_CALIB_CB_ADAPTIVE_THRESH
                                       | CV_CALIB_CB_FAST_CHECK
                                       | CV_CALIB_CB_NORMALIZE_IMAGE);
                                       
    if (!found) {
        return false;
    }
    
    Mat img_gray;
    cvtColor(img, img_gray, COLOR_BGR2GRAY);
    cornerSubPix(img_gray, point_buf, Size(11,11),
                 Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

    if (graphical_disp) {
        drawChessboardCorners(img, board_size, Mat(point_buf), found);
    }
    
    Size new_img_size = img.size();
    
    if (img_size.width > 0 && img_size.height > 0
     && img_size.width != new_img_size.width
     && img_size.height != new_img_size.height)
    {
        err("all images of one device must have the same dimension");
        return false;
    } else if (img_size.width == 0 && img_size.height == 0) {
        img_size = new_img_size;
    }
    
    img_pts.push_back(point_buf);
    
    return true;
}

bool run_calibration_with_data(vector<vector<Point2f> > pts, double *reproj_err) {
    assert(reproj_err);

    // initialize
    cam_mat = Mat::eye(3, 3, CV_64F);
    dist_mat = Mat::zeros(8, 1, CV_64F);
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    
    // fill obj_pts with std_obj_pts elements so that it has the same size as img_pts
    vector<vector<Point3f> > obj_pts;
    obj_pts.resize(img_pts.size(), std_obj_pts);
    
    //Find intrinsic and extrinsic camera parameters
    *reproj_err = calibrateCamera(obj_pts, img_pts, img_size, cam_mat,
                                  dist_mat, rvecs, tvecs, CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    
    return checkRange(cam_mat) && checkRange(dist_mat);
}

bool process_img(Mat &img) {
    if (!img.data || img.rows == 0 || img.cols == 0) return false;
        
    if (find_corners_in_img(img)) {
        undistort_imgs.push_back(img);
        
        if (graphical_disp) {
            imshow("image view", img);
            if (interactive) waitKey(0);
        }
    } else {
        return false;
    }
    
    return true;
}

bool process_vid(const char *file, int *num_img, int *num_img_ok) {
    vid_cap.open(file);

    if (!vid_cap.isOpened()) return false;
    
    const int num_frames = (int)vid_cap.get(CV_CAP_PROP_FRAME_COUNT);
    
    if (num_frames < VID_USE_NUM_FRAMES) {
        cout << "video does not contain enough frames" << endl;
        return false;
    }
    
    cout << ">>> number of frames in the video: " << num_frames << endl;
    
    const int skip_frames = num_frames / VID_USE_NUM_FRAMES;
    
    int bad_frame_attempt = 0;
    *num_img = VID_USE_NUM_FRAMES;
    *num_img_ok = 0;
    for (int frame_step = 0; frame_step < VID_USE_NUM_FRAMES; ) {
        int frame_pos = frame_step * skip_frames + bad_frame_attempt * VID_BAD_FRAME_SKIP;
        vid_cap.set(CV_CAP_PROP_POS_FRAMES, (double)(frame_pos));
        
        Mat img;
        vid_cap >> img;
        
        if (img.empty()) continue;
        
        cout << ">>> got video frame at frame pos " << frame_pos  << endl;
        
        bool ok = process_img(img);
        
        if (!ok && bad_frame_attempt < VID_BAD_FRAME_ATTEMPTS) {
            bad_frame_attempt++;
            cout << ">>> bad frame, retrying with other frame" << endl;
        } else {
            bad_frame_attempt = 0;  // reset
            frame_step++;
        }
        
        if (ok) (*num_img_ok)++;
    }

    return true;
}

void calibrate_device(const char *device) {
    // initialize
    img_size = Size(0, 0);
    undistort_imgs.clear();

    cout << "calibrating device '" << device << "'..." << endl;
    
    char path[DEVICE_PATH_STRLEN];
    sprintf(path, "./device_data/%s", device);
    
    DIR *dir = opendir(path);
    if (dir == NULL) {
        err("could not open the device's photos directory");
        return;
    }
    
    struct dirent *dir_item;
    
    int num_img = 0;
    int num_img_ok = 0;
    
    while (status_ok && (dir_item = readdir(dir)) != NULL) {
        if (!dir_item->d_name
         || dir_item->d_name[0] == '.'
         || dir_item->d_type != DT_REG)
            continue;
        
        char file[DEVICE_PATH_STRLEN];
        sprintf(file, "%s/%s", path, dir_item->d_name);
        
        cout << "> working with file '" << file << "'" << endl;

        if (guess_type(file) == PIC) {
            cout << ">> image file" << endl;
            
            num_img++;
        
            Mat img = imread(file, CV_LOAD_IMAGE_COLOR);
            bool res = process_img(img);
        
            cout << ">> finding chessboard corners: " << (res ? "ok" : "failed") << endl;
        
            if (res) num_img_ok++;
        } else if (guess_type(file) == VID) {
            cout << ">> video file" << endl;
            
            if (!process_vid(file, &num_img, &num_img_ok)) {
                cout << ">> error processing video file" << endl;
            }
        } else {
            cout << ">> unknown file type" << endl;
            continue;
        }
    }
        
    cout << "using " << num_img_ok << " out of " << num_img << " images for calibration" << endl;
    cout << "image size: " << img_size.width << "x" << img_size.height << " pixels" << endl;
    
    if (num_img_ok <= 0 || img_pts.size() <= 0) {
        err("no chessboard corners found for calibration");
        return;
    }
    
    
    cout << "calibrating device '" << device << "' using data set of " << img_pts.size() << endl;
    
    double reproj_err;
    bool calib_ok = run_calibration_with_data(img_pts, &reproj_err);
    
    if (!calib_ok) {
        cout << "calibration failed";
        return;
    }
        
    cout << "calibration succeeded with reprojection error " << reproj_err << endl;
    
    cout << "camera matrix:" << endl;
    print_mat(cam_mat);
    
    cout << "distortion coefficients:" << endl;
    print_mat(dist_mat);
    
    if (graphical_disp) {
        cout << "showing undistorted images" << endl;
    
        Mat uimg, map1, map2;
        initUndistortRectifyMap(cam_mat, dist_mat, Mat(),
            getOptimalNewCameraMatrix(cam_mat, dist_mat, img_size, 1, img_size, 0),
            img_size, CV_16SC2, map1, map2);
    
        int uimg_nr = 0;
        
        for (vector<Mat>::iterator it = undistort_imgs.begin();
             it != undistort_imgs.end();
             ++it)
        {
            cout << "image #" << uimg_nr << endl;
            
            remap(*it, uimg, map1, map2, INTER_LINEAR);
            imshow("image view (undistorted)", uimg);
            
            if (interactive) {
                char c = (char)waitKey();
                if (c  == ESC_KEY || c == 'q' || c == 'Q')
                    break;
            }
            
            uimg_nr++;
        }
    }
    
    avg_reproj_err = reproj_err;
    
    char file_out[DEVICE_PATH_STRLEN];
    sprintf(file_out, "./database/%s.xml", device);
    cout << "writing output to " << file_out << endl;
    
    if (!write_output(file_out)) {
        err("the result could not be written to the output file");
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
    }
    
    closedir(dir);
}

void init() {
    std_obj_pts.clear();
    
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            std_obj_pts.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
        }
    }
    
    file_ext_pic.clear();
    file_ext_pic.push_back("jpg");
    file_ext_pic.push_back("jpeg");
    file_ext_pic.push_back("png");
    
    file_ext_vid.clear();
    file_ext_vid.push_back("mgp");
    file_ext_vid.push_back("mpeg");
    file_ext_vid.push_back("avi");
    file_ext_vid.push_back("mov");
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
    
    init();
    
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
        cout << "done" << endl;
    } else {
        cerr << "calibration failed" << endl;
    
        return 3;
    }

    return 0;
}