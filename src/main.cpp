/**
 * Automatic camera intrinsics finder.
 *
 * The function printHelp() contains information about the usage of this program.
 *
 * Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * Contains many code parts from
 * http://docs.opencv.org/trunk/_downloads/camera_calibration.cpp
 *
 * See also: http://docs.opencv.org/trunk/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
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

#define VID_USE_NUM_FRAMES      25  // number of frames to use from videos
#define VID_BAD_FRAME_ATTEMPTS  5   // number of attempts when no chessboard could be detected in a video frame
#define VID_BAD_FRAME_SKIP      5   // number of frames to skip in the above event

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

bool disp_first_frame = false;  // program argument -g
bool interactive = false;       // program argument -i

bool flip_hori = false;
bool flip_vert = false;
bool fix_principal_pt = false;
bool fix_aspect_ratio = false;
bool zero_tangent_dist = false;

Size board_size(9,6);   // number of *inner* corners per a chessboard row and column

bool status_ok = true;  // program will shutdown when this is false

vector<vector<Point2f> > img_pts;   // found chessboard corners in images
vector<Point3f> std_obj_pts;        // "ideal" chessboard corners in 3D space

vector<Mat> undistort_imgs;         // vector that contains images that will be undistorted later

Mat cam_mat, dist_mat;              // result matrices
double avg_reproj_err = 0.0;
Size img_size(0, 0);

vector<string> file_ext_pic;
vector<string> file_ext_vid;



/** HELPER FUNCTIONS **/

void printHelp() {
    cout << "usage:" << endl;
    cout << "cam_intrinsics-db [-(g|i)pazhv] <square size in meters> [device]" << endl;
    cout << " optional flags:" << endl;
    cout << "  '-g' for graphical output (shows original and undistorted first frame)" << endl;
    cout << "  '-i' for *interactive* graphical output (step through all frames)" << endl;
    cout << "  'p' to fix principal point during calibration" << endl;
    cout << "  'a' to fix aspect ratio during calibration" << endl;
    cout << "  'z' to assume zero tangential distortion during calibration" << endl;
    cout << "  'h' to flip image horizontally" << endl;
    cout << "  'v' to flip image vertically" << endl;
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

/** PROGRAM FUNCTIONS **/

/**
 * Write the two result matrices and the reprojection error
 * to the XML file in <file>.
 */
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

/**
 * Find the chessboard corners in <img>.
 * Returns false if no chessboard corners could be found, otherwise true.
 */
bool find_corners_in_img(Mat& img, bool is_first_frame) {
    vector<Point2f> point_buf;  // will contain the coords. of the found chessboard corners

    bool found = findChessboardCorners(img, board_size, point_buf,
                                       CV_CALIB_CB_ADAPTIVE_THRESH
                                       | CV_CALIB_CB_FAST_CHECK
                                       | CV_CALIB_CB_NORMALIZE_IMAGE);
                                       
    if (!found) {
        return false;
    }
    
    // refine the corners
    Mat img_gray;
    cvtColor(img, img_gray, COLOR_BGR2GRAY);
    cornerSubPix(img_gray, point_buf, Size(11,11),
                 Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

    // draw the corners if necessary
    if ((disp_first_frame && is_first_frame) || interactive) {
        drawChessboardCorners(img, board_size, Mat(point_buf), found);
    }
    
    // set the size if necessary
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
    
    // add to the result to the overall points vector
    img_pts.push_back(point_buf);
    
    return true;
}

/**
 * Run the calibration with the provided points in <pts>.
 * Saves the reported reprojection error in <reproj_err>.
 * Return false if calibration failed, otherwise true.
 */
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
    int calib_flags = CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5;
    if (fix_aspect_ratio) calib_flags |= CV_CALIB_FIX_ASPECT_RATIO;
    if (fix_principal_pt) calib_flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
    if (zero_tangent_dist) calib_flags |= CV_CALIB_ZERO_TANGENT_DIST;
    *reproj_err = calibrateCamera(obj_pts, img_pts, img_size,
                                  cam_mat, dist_mat,
                                  rvecs, tvecs,
                                  calib_flags);
    
    // matrices must be ok
    return checkRange(cam_mat) && checkRange(dist_mat);
}

/**
 * Process a single image <img>.
 * Find its chessboard corners and display the result if necessary.
 * Returns false if the processing failed (invalid image; no chessboard
 * corners found), otherwise true.
 */
bool process_img(Mat &img, bool is_first_frame) {
    if (!img.data || img.rows == 0 || img.cols == 0) return false;  // invalid image

    if (flip_vert || flip_hori) {
        int flip_mode;
        if (flip_vert && !flip_hori) {
            flip_mode = 0;
        } else if (flip_hori && !flip_vert) {
            flip_mode = 1;
        } else {
            flip_mode = -1;
        }
        
        flip(img, img, flip_mode);
    }
    
    // find chessboard corners in the image
    if (find_corners_in_img(img, is_first_frame)) {
        if ((disp_first_frame && is_first_frame) || interactive) {
            undistort_imgs.push_back(img);  // save it for later
            
            // show the image with the chessboard corners
            char win_title[DEVICE_PATH_STRLEN];
            sprintf(win_title, "image view - %s", device);
            imshow(win_title, img);
            if (interactive) waitKey(0);
        }
    } else {
        return false;
    }
    
    return true;
}

/**
 * Process a video in <file>. Will select <VID_USE_NUM_FRAMES> frames in the video,
 * equally distributed on the whole video.
 * Save the number of selected and sucessfully processed frames in <num_img> and
 * <num_img_ok>, respectively.
 * Returns false if processing the video failed, otherwise true.
 */
bool process_vid(const char *file, int *num_img, int *num_img_ok) {
    // open the video file
    vid_cap.open(file);

    // check on fail
    if (!vid_cap.isOpened()) return false;
    
    // get the number of frames in the video
    const int num_frames = (int)vid_cap.get(CV_CAP_PROP_FRAME_COUNT);
    
    if (num_frames < VID_USE_NUM_FRAMES) {
        cout << "video does not contain enough frames" << endl;
        return false;
    }
    
    cout << ">>> number of frames in the video: " << num_frames << endl;
    
    // set how many frames will be skipped for processing
    const int skip_frames = num_frames / VID_USE_NUM_FRAMES;
    
    int bad_frame_attempt = 0;  // will remember the number of bad frames
    *num_img = VID_USE_NUM_FRAMES;
    *num_img_ok = 0;
    for (int frame_step = 0; frame_step < VID_USE_NUM_FRAMES; ) {   // select and process single video frames
        // calculate the position of the frame which will be grabbed
        int frame_pos = frame_step * skip_frames + bad_frame_attempt * VID_BAD_FRAME_SKIP;
        vid_cap.set(CV_CAP_PROP_POS_FRAMES, (double)(frame_pos));
        
        // read the frame
        Mat img;
        vid_cap >> img;
        
        if (img.empty()) continue;  // this shouldn't happen
        
        cout << ">>> got video frame at frame pos " << frame_pos  << endl;
        
        // process the frame
        bool ok = process_img(img, *num_img_ok == 0);
        
        if (!ok && bad_frame_attempt < VID_BAD_FRAME_ATTEMPTS) {    // remember a bad frame
            bad_frame_attempt++;
            cout << ">>> bad frame, retrying with other frame" << endl;
        } else {    // frame was ok
            bad_frame_attempt = 0;  // reset
            frame_step++;
        }
        
        if (ok) (*num_img_ok)++;
    }

    return true;
}

/**
 * Start the calibration process for <dev>.
 */
void calibrate_device(const char *dev) {
    if (device != dev) {
        strncpy(device, dev, DEVICE_STRLEN);
    }
    
    // initialize
    img_size = Size(0, 0);
    undistort_imgs.clear();
    img_pts.clear();

    cout << "calibrating device '" << device << "'..." << endl;
    
    // STEP 1: collect image points for the chessboard images
    
    // path to the device data
    char path[DEVICE_PATH_STRLEN];
    sprintf(path, "./device_data/%s", device);
    
    // open the directory
    DIR *dir = opendir(path);
    if (dir == NULL) {
        err("could not open the device's photos directory");
        return;
    }
    
    struct dirent *dir_item;
    int num_img = 0;
    int num_img_ok = 0;
    
    // go through the items of the directory
    while (status_ok && (dir_item = readdir(dir)) != NULL) {
        // only handle regular files
        if (!dir_item->d_name
         || dir_item->d_name[0] == '.'
         || dir_item->d_type != DT_REG)
            continue;
        
        // full file path
        char file[DEVICE_PATH_STRLEN];
        sprintf(file, "%s/%s", path, dir_item->d_name);
        
        cout << "> working with file '" << file << "'" << endl;

        // check the file type according to the file extension
        if (guess_type(file) == PIC) {
            // process an image file
            cout << ">> image file" << endl;
            
            Mat img = imread(file, CV_LOAD_IMAGE_COLOR);
            bool res = process_img(img, num_img_ok == 0);
        
            cout << ">> finding chessboard corners: " << (res ? "ok" : "failed") << endl;
        
            num_img++;
            if (res) num_img_ok++;
        } else if (guess_type(file) == VID) {
            // process a vide file
            cout << ">> video file" << endl;
            
            if (!process_vid(file, &num_img, &num_img_ok)) {
                cout << ">> error processing video file" << endl;
            }
        } else {
            cout << ">> unknown file type" << endl;
            continue;
        }
    }
    
    closedir(dir);

    // STEP 2: start the calibration process
    
    cout << "using " << num_img_ok << " out of " << num_img << " images for calibration" << endl;
    cout << "image size: " << img_size.width << "x" << img_size.height << " pixels" << endl;
    
    if (num_img_ok <= 0 || img_pts.size() <= 0) {
        err("no chessboard corners found for calibration");
        return;
    }
    
    cout << "calibrating device '" << device << "' using data set of " << img_pts.size() << endl;
    
    // run the calibration and save the reprojection error value
    double reproj_err;
    bool calib_ok = run_calibration_with_data(img_pts, &reproj_err);
    
    if (!calib_ok) {
        cout << "calibration failed";
        return;
    }

    // print the results
    cout << "calibration succeeded with reprojection error " << reproj_err << endl;
    
    cout << "camera matrix:" << endl;
    print_mat(cam_mat);
    
    cout << "distortion coefficients:" << endl;
    print_mat(dist_mat);
    
    // show the undistorted frames if necessary
    if (disp_first_frame || interactive) {
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
            
            char win_title[DEVICE_PATH_STRLEN];
            sprintf(win_title, "image view (undist) - %s", device);
            imshow(win_title, uimg);
            
            if (disp_first_frame) break;
            
            if (interactive) {
                char c = (char)waitKey();
                if (disp_first_frame || c  == ESC_KEY || c == 'q' || c == 'Q')
                    break;
            }
            
            uimg_nr++;
        }
    }
    
    avg_reproj_err = reproj_err;
    
    // STEP 3: write the results to a file
    
    char file_out[DEVICE_PATH_STRLEN];
    sprintf(file_out, "./database/%s.xml", device);
    cout << "writing output to " << file_out << endl;
    
    if (!write_output(file_out)) {
        err("the result could not be written to the output file");
    }
}

/**
 * Start the calibration process for all devices that have images or videos
 * at "./device_data/"
 */
void calibrate_all() {
    // open the directory
    DIR *dir = opendir("./device_data");
    if (dir == NULL) {
        err("could not open photos directory");
        return;
    }
    
    // go through the items of the directory
    struct dirent *dir_item;
    while (status_ok && ((dir_item = readdir(dir)) != NULL)) {
        // only handle folders
        if (!dir_item->d_name
         || dir_item->d_name[0] == '.'
         || dir_item->d_type != DT_DIR)
            continue;
    
        // calibrate the device
        calibrate_device(dir_item->d_name);
    }
    
    closedir(dir);
}

/**
 * Initialize the program.
 */
void init() {
    // set the "ideal" chessboard points in 3D space
    std_obj_pts.clear();
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            std_obj_pts.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
        }
    }
    
    // set valid picture file extensions
    file_ext_pic.clear();
    file_ext_pic.push_back("jpg");
    file_ext_pic.push_back("jpeg");
    file_ext_pic.push_back("png");
    file_ext_pic.push_back("gif");
    file_ext_pic.push_back("bmp");
    
    // set valid video file extensions
    file_ext_vid.clear();
    file_ext_vid.push_back("mgp");
    file_ext_vid.push_back("mpeg");
    file_ext_vid.push_back("avi");
    file_ext_vid.push_back("mov");
}

/**
 * Parse flags in <arg>.
 */
void parse_flags_arg(const char *arg) {
    const int len = strlen(arg);
    assert(len > 1);
    
    for (int i = 1; i < len; ++i) {
        char c = arg[i];
        
        if (c == 'g' && !interactive) {
            disp_first_frame = true;
        } else if (c == 'i' && !disp_first_frame) {
            interactive = true;
        } else if (c == 'p') {
            fix_principal_pt = true;
        } else if (c == 'a') {
            fix_aspect_ratio = true;
        } else if (c == 'z') {
            zero_tangent_dist = true;
        } else if (c == 'h') {
            flip_hori = true;
        } else if (c == 'v') {
            flip_vert = true;
        }
    }
}

/** MAIN FUNCTION **/

int main(int argc, char *argv[]) {
    if (argc < 2 || strlen(argv[1]) <= 1) { // we require at least 1 argument (square size)
        printHelp();
        return 1;
    }
    
    int params_idx = 1;
    
    // parse optional flags
    if (strlen(argv[params_idx]) > 1 && argv[params_idx][0] == '-') {
        parse_flags_arg(argv[params_idx]);
        
        if (argc <= params_idx + 1) {   // we require at least 1 more argument
            printHelp();
            return 1;
        }
        
        params_idx++;
    }
    
    // get the square size
    square_size = (float)strtod(argv[params_idx], NULL);
    params_idx++;
    
    if (square_size <= 0.0f) {
        err("unable to parse first argument as square size");
        printHelp();
        
        return 2;
    }
    
    // optionally get the device
    if (argc > params_idx) {
        strncpy(device, argv[params_idx], DEVICE_STRLEN);
        all_devices = false;
    }
    
    // init the program
    init();
    
    // run the routines
    cout << "using square size of " << square_size << " meters" << endl;
    cout << "calibration options:" << endl;
    cout << " fix principal point: " << fix_principal_pt << endl;
    cout << " fix aspect ratio: " << fix_aspect_ratio << endl;
    cout << " assume zero tangential distortion: " << zero_tangent_dist << endl;
    cout << " horizontal flip: " << flip_hori << endl;
    cout << " vertical flip: " << flip_vert << endl;
    cout << "generating camera intrinsics for ";
    
    if (all_devices) {
        cout << "all devices" << endl;
        
        calibrate_all();
    } else {
        cout << "device '" << device << "'" << endl;
        
        calibrate_device(device);
    }

    // display result
    if (status_ok) {
        cout << "done" << endl;
    } else {
        cerr << "calibration failed" << endl;
    
        return 3;
    }
    
    if (disp_first_frame) {
        cout << "select one of the spawned windows and press a key to close" << endl;
        waitKey(0);
    }

    return 0;
}