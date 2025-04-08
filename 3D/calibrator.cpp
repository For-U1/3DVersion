#include "Calibrator.h"
#include <fstream>
#include <cmath>
#include <stdexcept>
//#include"utils.h"

bool Calibration::calib(const string &calib_file) 
{
    // Read calibration file
    ifstream in(calib_file);
    if (in.fail()) 
    {
        throw logic_error("file not existed！");
        return false;
    }
    cout << "read calib file:" << calib_file << endl;
    regex p{".*[a-zA-Z]+.*"};
    string line, key;
    int row, col;
    vector<double> vec; // Data for one line
    vector<vector<double>> matrix; // Matrix data

    while (getline(in, line) && in.good()) {

        if (regex_match(line, p)) 
        {
            key = line;
            matrix.clear();
        } else 
        {
            if (split(line, vec, "\t")) 
            {
                matrix.emplace_back(vec);
            } else {
                row = matrix.size();
                col = matrix[0].size();
                cv::Mat M = cv::Mat_<double>(row, col);
                for (int r = 0; r < row; ++r) {
                    for (int c = 0; c < col; ++c) {
                        M.at<double>(r, c) = matrix[r][c];
                    }
                }

                cout << "###################" << endl;
                if (key == "imageSize") {
                    imageSize = cv::Size2l(int(M.at<double>(1)), int(M.at<double>(0)));
                } else if (key == "KK_L") {
                    KK_L = M.t();
                    cout << "KK_L:" << endl << KK_L << endl;
                } else if (key == "KK_R") {
                    KK_R = M.t();
                    cout << "KK_R:" << endl << KK_R << endl;
                } else if (key == "RadialDistortion_L") {
                    RadialDistortion_L = M;
                    cout << "RadialDistortion_L:" << endl << RadialDistortion_L << endl;
                } else if (key == "RadialDistortion_R") {
                    RadialDistortion_R = M;
                    cout << "RadialDistortion_R:" << endl << RadialDistortion_R << endl;
                } else if (key == "TangentialDistortion_L") {
                    TangentialDistortion_L = M;
                    cout << "TangentialDistortion_L:" << endl << TangentialDistortion_L << endl;
                } else if (key == "TangentialDistortion_R") {
                    TangentialDistortion_R = M;
                    cout << "TangentialDistortion_R:" << endl << TangentialDistortion_R << endl;
                } else if (key == "R") {
                    R = M.t();
                    cout << "R:" << endl << R << endl;
                } else if (key == "T") {
                    T = M.t();
                    cout << "T:" << endl << T << endl;
                }
                else if (key == "E"){
                    //                    E = M.t();
                    //                    cout << "E:" << endl;
                    //                    cout << E << endl;
                }
                else if (key == "F"){
                    //                    F = M.t();
                    //                    cout << "F:" << endl;
                    //                    cout << F << endl;
                }
                else if (key == "error") {
                    error = M;
                    cout << "error:" << endl << error << endl;
                } else {
                    cerr << key << endl << "error key" << endl;
                    return false;
                }
            }
        }
    }

    // Distortion coefficients for OpenCV
    Dist_L = (cv::Mat_<double>(5, 1) <<
                  RadialDistortion_L.at<double>(0),
              RadialDistortion_L.at<double>(1),
              TangentialDistortion_L.at<double>(0),
              TangentialDistortion_L.at<double>(1),
              0.);
    Dist_R = (cv::Mat_<double>(5, 1) <<
                  RadialDistortion_R.at<double>(0),
              RadialDistortion_R.at<double>(1),
              TangentialDistortion_R.at<double>(0),
              TangentialDistortion_R.at<double>(1),
              0.);

    // Stereo calibration
    cv::stereoRectify(
        KK_L, Dist_L,
        KK_R, Dist_R,
        imageSize, R, T,
        R_L, R_R, P_L, P_R, Q,
        cv::CALIB_ZERO_DISPARITY,
        1, imageSize,
        &validROI_L, &validROI_R
        );

    // Compute undistortion and rectification maps
    cv::initUndistortRectifyMap(KK_L, Dist_L, R_L, P_L, imageSize, CV_16SC2, rmap_L[0], rmap_L[1]);
    cv::initUndistortRectifyMap(KK_R, Dist_R, R_R, P_R, imageSize, CV_16SC2, rmap_R[0], rmap_R[1]);

    cout << "finish calib!" << endl;
    return true;
}

bool Calibration::rectify(cv::Mat &imgL_src, cv::Mat &imgR_src, cv::Mat &imgL_des, cv::Mat &imgR_des)
{
    // Undistort images
    cv::undistort(imgL_src, imgL_des, KK_L, Dist_L);
    cv::undistort(imgR_src, imgR_des, KK_R, Dist_R);
    //cout<<"wodaozhile"<<endl;
    // Rectify images
    cv::remap(imgL_des, imgL_des, rmap_L[0], rmap_L[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(imgR_des, imgR_des, rmap_R[0], rmap_R[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return true;
}
