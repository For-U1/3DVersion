

#include <map>
#include <iostream>
#include <regex>
#include <opencv2/opencv.hpp>
#include "utils.h"

using namespace std;

class Calibration
{
public:
    cv::Mat Q;
    cv::Size imageSize;

    // Load calibration data from a file
    bool calib(const string &calib_file);

    // Rectify stereo images
    bool rectify(cv::Mat &imgL_src, cv::Mat &imgR_src, cv::Mat &imgL_des, cv::Mat &imgR_des);

protected:
    cv::Mat KK_L, KK_R;                                     // Camera matrices
    cv::Mat RadialDistortion_L, RadialDistortion_R;         // Radial distortion coefficients
    cv::Mat TangentialDistortion_L, TangentialDistortion_R; // Tangential distortion coefficients
    cv::Mat R, T, E, F, error, Dist_L, Dist_R;

    cv::Mat rmap_L[2], rmap_R[2];
    cv::Mat R_L, R_R, P_L, P_R;
    cv::Rect validROI_L, validROI_R;
};


