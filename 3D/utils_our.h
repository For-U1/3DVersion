//#pragma once
//#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
//#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
//
//#include <vector>
//#include <iostream>
//#include <string>
//#include <sstream>
//#include <fstream>
//#include <iomanip>
//#include <filesystem>
//#include <opencv2/opencv.hpp>
//
//namespace fs = std::filesystem;
//
//// 分割字符串
//bool split(const std::string& line, std::vector<double>& res, const std::string& pattern) {
//    if (line.empty())
//        return false;
//    res.clear();
//    std::stringstream ss(line);
//    std::string data;
//    double v;
//    char* ptr;
//    while (std::getline(ss, data, '\t')) {
//        v = std::strtod(data.c_str(), &ptr);
//        res.emplace_back(v);
//    }
//    return !res.empty();
//}
//
//// 生成图像路径
//bool glob(int T, int N, const std::string& folder, std::vector<std::string>& files) {
//    files.clear();
//    int num = T * N;
//    for (int i = 1; i <= num; ++i) {
//        std::stringstream ss;
//        ss << std::setw(2) << std::setfill('0') << i;
//        std::string imagename = ss.str();
//        std::string filename = folder + "/" + imagename + ".bmp";
//        files.emplace_back(filename);
//    }
//    return !files.empty();
//}
//
//// 生成左右相机的图像路径
//bool globLR(int T, int N, const std::string& folder, std::vector<std::string>& filesL, std::vector<std::string>& filesR) {
//    filesL.clear();
//    filesR.clear();
//    std::string folderL = folder + "/L";
//    std::string folderR = folder + "/R";
//    int num = T * N;
//    for (int i = 1; i <= num; ++i) {
//        std::stringstream ss;
//        int count = i - 1;
//        ss << std::setw(2) << std::setfill('0') << count;
//        std::string imagename = ss.str();
//        std::string filename_L = folderL + "/left_" + imagename + ".bmp";
//        std::string filename_R = folderR + "/right_" + imagename + ".bmp";
//        filesL.emplace_back(filename_L);
//        filesR.emplace_back(filename_R);
//    }
//    return (!filesL.empty()) && (!filesR.empty());
//}
//
//// 显示左右图像对
//void showImagePair(const cv::Mat& imgL, const cv::Mat& imgR, const std::string& title, float s, bool line, bool color) {
//    cv::Mat img_all;
//    cv::hconcat(imgL, imgR, img_all);
//    cv::resize(img_all, img_all, cv::Size(), s, s);
//    double min_v, max_v;
//    cv::minMaxLoc(img_all, &min_v, &max_v);
//    img_all = (img_all - min_v) * 255.0 / (max_v - min_v);
//    img_all.convertTo(img_all, CV_8UC1);
//    if (color) {
//        cv::applyColorMap(img_all, img_all, cv::COLORMAP_JET);
//    }
//
//    // 绘制水平参考线
//    if (line) {
//        int h = img_all.rows;
//        int w = img_all.cols;
//        int num = 10;
//        int p = h / num;
//        for (int i = 0; i < num; ++i) {
//            int y = i * p;
//            cv::line(img_all, cv::Point(0, y), cv::Point(w, y), cv::Scalar(255, 255, 255), 1);
//        }
//    }
//
//    cv::imshow(title, img_all);
//    cv::waitKey(1);
//}
//
//// 显示单张图像
//void showImage(const cv::Mat& img, const std::string& title, float s, bool color) {
//    cv::Mat img_s;
//    cv::resize(img, img_s, cv::Size(1920, 1080));
//    double min_v, max_v;
//    cv::minMaxLoc(img_s, &min_v, &max_v);
//    img_s = (img_s - min_v) * 255.0 / (max_v - min_v);
//    img_s.convertTo(img_s, CV_8UC1);
//    if (color) {
//        cv::applyColorMap(img_s, img_s, cv::COLORMAP_JET);
//    }
//    cv::imshow(title, img_s);
//    cv::waitKey(1);
//}
//
//// 保存 Mat 到文本文件
//void saveMat(const std::string& filename, cv::Mat& mat) {
//    fs::path pathObj(filename);
//    fs::path dirPath = pathObj.parent_path();
//
//    // 确保目录存在
//    if (!fs::exists(dirPath)) {
//        if (!fs::create_directories(dirPath)) {
//            std::cerr << "Failed to create directory: " << dirPath << std::endl;
//            return;
//        }
//    }
//
//    std::ofstream f(filename);
//    if (!f.is_open()) {
//        std::cerr << "Error opening file: " << filename << std::endl;
//        return;
//    }
//
//    std::cout << "Writing to file: " << filename << std::endl;
//    for (int row = 0; row < mat.rows; ++row) {
//        for (int col = 0; col < mat.cols; ++col) {
//            f << std::setprecision(32) << mat.at<double>(row, col);
//            if (col < mat.cols - 1) {
//                f << ",";
//            }
//        }
//        f << std::endl;
//    }
//    f.close();
//    std::cout << "Finished writing to file: " << filename << std::endl;
//}
//
//// 计算模运算
//double mod(double a, double m) {
//    return a - m * std::floor(a / m);
//}
//
//#endif // CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
