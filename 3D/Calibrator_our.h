#pragma once
#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_CALIBRATOR_H
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_CALIBRATOR_H


#include <map>
#include<iostream>
#include <regex>
#include <opencv2/opencv.hpp>
#include "utils.h"

//using namespace std;

class Calibrator_Registration
{
public:

    cv::Mat Q;
    cv::Size imageSize;

protected:

    cv::Mat
        KK_L, KK_R,                                     // 相机畸变
        RadialDistortion_L, RadialDistortion_R,         // 径向畸变
        TangentialDistortion_L, TangentialDistortion_R, // 切向畸变
        R, T, E, F, error, Dist_L, Dist_R;

    cv::Mat rmap_L[2], rmap_R[2];
    cv::Mat R_L, R_R, P_L, P_R;
    cv::Rect validROI_L, validROI_R;

public:
    // function：加载Matlab的标定结果
    bool calib_registration(const string& calib_file, bool use_CALIB_ZERO_DISPARITY) 
    {
        // 01 读取标定文件
        std::ifstream in(calib_file);
        if (in.fail()) 
        {
            throw std::logic_error("file not existed！ ");
            return false;
        }
        cout << "read calib file:" << calib_file << endl;
        std::regex p{ ".*[a-zA-Z]+.*" };
        string line, key;
        int row, col;
        vector<double> vec;// 一行数据
        vector<vector<double>> matrix{}; // 一个矩阵
        while (getline(in, line) && in.good()) 
        {
            // 匹配到字符串：意味着读取R
            if (std::regex_match(line, p)) 
            {
                key = line;
                matrix.clear();
            }
            // 其他情况
            else 
            {
                // 读取到数据，
                if (split(line, vec, "\t")) 
                {
                    matrix.emplace_back(vec);
                }
                // 否则为空行，意味着结束，将matrix数据转换到cv::Mat矩阵中
                else 
                {
                    // 将vector转换为Mat类型
                    row = matrix.size();
                    col = matrix[0].size();
                    cv::Mat M = cv::Mat_<double>(row, col);
                    for (int r = 0; r < row; ++r) 
                    {
                        for (int c = 0; c < col; ++c) 
                        {
                            M.at<double>(r, c) = matrix[r][c];
                        }
                    }
                    // 根据名字，将数据赋值
                    //cout << "###################" << endl;
                    if (key == "imageSize") 
                    {
                        imageSize = cv::Size2l(int(M.at<double>(1)), int(M.at<double>(0)));
                    }
                    else if (key == "KK_L") 
                    {
                        KK_L = M;
                        //cout << "KK_L:" << endl;
                        //cout << KK_L << endl;
                    }
                    else if (key == "KK_R") 
                    {
                        KK_R = M;
                        //cout << "KK_R:" << endl;
                        //cout << KK_R << endl;
                    }
                    else if (key == "RadialDistortion_L") 
                    {
                        RadialDistortion_L = M;
                        //cout << "RadialDistortion_L:" << endl;
                        //cout << RadialDistortion_L << endl;
                    }
                    else if (key == "RadialDistortion_R") 
                    {
                        RadialDistortion_R = M;
                        //cout << "RadialDistortion_R:" << endl;
                        //cout << RadialDistortion_R << endl;
                    }
                    else if (key == "TangentialDistortion_L") 
                    {
                        TangentialDistortion_L = M;
                        //cout << "TangentialDistortion_L:" << endl;
                        //cout << TangentialDistortion_L << endl;
                    }
                    else if (key == "TangentialDistortion_R") 
                    {
                        TangentialDistortion_R = M;
                        //cout << "TangentialDistortion_R:" << endl;
                        //cout << TangentialDistortion_R << endl;
                    }
                    else if (key == "R") 
                    {
                        // opencv/matlab区别
                        R = M;
                        //cout << "R" << endl;
                        //cout << R << endl;
                    }
                    else if (key == "T") 
                    {
                        T = M.t();
                        //cout << "T:" << endl;
                        //cout << T << endl;
                    }
                    // 暂时不需要处理 E、F
                    else if (key == "E") 
                    {
                        //                    E = M.t();
                        //                    cout << "E:" << endl;
                        //                    cout << E << endl;
                    }
                    else if (key == "F") 
                    {
                        //                    F = M.t();
                        //                    cout << "F:" << endl;
                        //                    cout << F << endl;
                    }
                    else if (key == "error") 
                    {
                        error = M;
                        //cout << "error:" << endl;
                        //cout << error << endl;
                    }
                    else 
                    {
                        std::cerr << key << endl;
                        std::cerr << "error key" << endl;
                        return false;
                    }
                }
            }
        }
        // 畸变系数
        // MatLab：k1, k2, k3, p1, p2
        // Opencv：k1, k2, p1, p2, k3
        // 注：对于鱼眼相机，我们采用k3
        if (RadialDistortion_L.cols == 2) 
        {
            Dist_L = (cv::Mat_<double>(5, 1) <<
                RadialDistortion_L.at<double>(0),
                RadialDistortion_L.at<double>(1),
                TangentialDistortion_L.at<double>(0),
                TangentialDistortion_L.at<double>(1),
                0.
                );
            Dist_R = (cv::Mat_<double>(5, 1) <<
                RadialDistortion_R.at<double>(0),
                RadialDistortion_R.at<double>(1),
                TangentialDistortion_R.at<double>(0),
                TangentialDistortion_R.at<double>(1),
                0.
                );
        }
        else
        {
            Dist_L = (cv::Mat_<double>(5, 1) <<
                RadialDistortion_L.at<double>(0),
                RadialDistortion_L.at<double>(1),
                TangentialDistortion_L.at<double>(0),
                TangentialDistortion_L.at<double>(1),
                RadialDistortion_L.at<double>(2)
                );
            Dist_R = (cv::Mat_<double>(5, 1) <<
                RadialDistortion_R.at<double>(0),
                RadialDistortion_R.at<double>(1),
                TangentialDistortion_R.at<double>(0),
                TangentialDistortion_R.at<double>(1),
                RadialDistortion_R.at<double>(2)
                );
        }

        // 标定
        if (use_CALIB_ZERO_DISPARITY) 
        {
            cv::stereoRectify(
                KK_L, Dist_L,
                KK_R, Dist_R,
                imageSize, R, T,
                R_L, R_R, P_L, P_R, Q,
                cv::CALIB_ZERO_DISPARITY,
                1, imageSize,
                &validROI_L, &validROI_R
            );

        }
        else
        {
            cv::stereoRectify(
                KK_L, Dist_L,
                KK_R, Dist_R,
                imageSize, R, T,
                R_L, R_R, P_L, P_R, Q,
                0,
                -1, imageSize,
                &validROI_L, &validROI_R
            );
        }


        // 为cv::undistort、cv::remap提前计算
        cv::initUndistortRectifyMap(KK_L, Dist_L, R_L, P_L, imageSize, CV_16SC2, rmap_L[0], rmap_L[1]);
        cv::initUndistortRectifyMap(KK_R, Dist_R, R_R, P_R, imageSize, CV_16SC2, rmap_R[0], rmap_R[1]);
        cout << "finish calib!" << endl;
        return true;
    }

    // function：立体校正（包含单台相机的畸变校正）
    bool rectify(cv::Mat& imgL_src, cv::Mat& imgR_src, cv::Mat& imgL_des, cv::Mat& imgR_des) 
    {
        // 单目畸变校正
        cv::undistort(imgL_src, imgL_des, KK_L, Dist_L);
        cv::undistort(imgR_src, imgR_des, KK_R, Dist_R);
        // 双目立体校正
        cv::remap(imgL_des, imgL_des, rmap_L[0], rmap_L[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(imgR_des, imgR_des, rmap_R[0], rmap_R[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        return true;
    }
    void showImagePair(const cv::Mat& imgL, const cv::Mat& imgR, const string& title, float s, bool line, bool color) 
    {
        cv::Mat img_all;
        cv::hconcat(imgL, imgR, img_all);
        cv::resize(img_all, img_all, cv::Size(), s, s);
        double min_v, max_v;
        cv::Point min_p, max_p;
        cv::minMaxLoc(img_all, &min_v, &max_v, &min_p, &max_p);
        img_all = (img_all - min_v) * 255. / (max_v - min_v);
        img_all.convertTo(img_all, CV_8UC1);
        if (color) 
        {
            cv::applyColorMap(img_all, img_all, cv::COLORMAP_JET);
        }
        // 绘制平行线
        int h = img_all.rows;
        int w = img_all.cols;
        if (line) {
            int num = 10;
            int p = h / num;
            for (int i = 0; i < num; ++i) {
                int y = i * p;
                cv::line(img_all, cv::Point(0, y), cv::Point(w, y), cv::Scalar(255, 255, 255), 1);
            }
        }
        cv::imshow(title, img_all);
        cv::waitKey(1);
    }
    // function：立体校正（包含单台相机的畸变校正）
    bool rectifySingleL(cv::Mat& imgL_src, cv::Mat& imgL_des) 
    {
        // 单目畸变校正
        cv::undistort(imgL_src, imgL_des, KK_L, Dist_L);
        // 双目立体校正
        cv::remap(imgL_des, imgL_des, rmap_L[0], rmap_L[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        return true;
    }
    int countImagesInDirectory(const std::filesystem::path& directoryPath)
    {
        // 定义图片文件的扩展名集合
        std::set<std::string> imageExtensions = { ".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tiff" };
        int imageCount = 0;
        try {
            // 遍历目录下的所有文件
            for (const auto& entry : std::filesystem::directory_iterator(directoryPath))
            {
                if (entry.is_regular_file()) 
                {
                    // 获取文件扩展名
                    std::string extension = entry.path().extension().string();
                    // 将扩展名转换为小写
                    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

                    // 检查是否是图片文件
                    if (imageExtensions.find(extension) != imageExtensions.end()) 
                    {
                        ++imageCount;
                    }
                }
            }
        }
        catch (const std::filesystem::filesystem_error& e)
        {
            std::cerr << "Filesystem error: " << e.what() << std::endl;
        }

        return imageCount;
    }

};
#endif //CH7_STRUCTURELIGHTSTEREOTRIANGLE_CALIBRATOR_H
