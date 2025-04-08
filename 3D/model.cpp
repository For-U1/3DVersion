#include "Model.h"
#include <cstdio>
#include <cerrno>
#include <iostream>

bool Model::init(cv::Mat &Q) {
    this->Q = Q;
    return true;
}

bool Model::calcDistance(cv::Mat &dif_map, cv::Mat &depth_map, vector<pair<cv::Point2f, cv::Point2f>> &cps) {
    // Initialize disparity map and depth map with zeros
    for (const auto &cp : cps) {
        cv::Point2f l = cp.first, r = cp.second;
        float d = l.x - r.x;
        if (d <= 0) {
            d = 0.0;
        }
        dif_map.at<float>(static_cast<int>(l.y), static_cast<int>(l.x)) = d;
    }

    // Optionally remove noise, which might reduce accuracy
    cv::medianBlur(dif_map, dif_map, 3);

    // Convert disparity map to depth map
    cv::reprojectImageTo3D(dif_map, depth_map, Q, false, CV_32FC1);
    return true;
}

bool Model::saveXYZ(const string& filename, cv::Mat& depth_map, float min_z, float max_z) {
    FILE* fp = nullptr;
    errno_t err = fopen_s(&fp, filename.c_str(), "wt");
    if (err != 0 || fp == nullptr) {
        cerr << "Error opening file: " << filename << endl;
        return false;
    }

    // Scan with a border of 5 pixels
    int edge = 5;
    for (int row = edge; row < depth_map.rows - edge; ++row) {
        for (int col = edge; col < depth_map.cols - edge; ++col) {
            float point_x = depth_map.at<cv::Vec3f>(row, col)[0];
            float point_y = depth_map.at<cv::Vec3f>(row, col)[1];
            float point_z = depth_map.at<cv::Vec3f>(row, col)[2];
            if (point_z > max_z || point_z < min_z) {
                point_x = 0.;
                point_y = 0.;
                point_z = 0.;
                depth_map.at<cv::Vec3f>(row, col)[0] = point_x;
                depth_map.at<cv::Vec3f>(row, col)[1] = point_y;
                depth_map.at<cv::Vec3f>(row, col)[2] = point_z;
            }
            fprintf(fp, "%f %f %f\n", point_x, point_y, point_z);
        }
    }
    fclose(fp);
    cout << "Write 3d point into file: \t" << filename << endl;
    return true;
}
