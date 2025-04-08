#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <functional>

using namespace std;

class CornerDetection
{
public:
    bool corner(const std::string& calib_file, bool showcornor, const std::string& savePath,
        int boardWidth, int boardHeight, double squareSize,
        cv::Mat& R, cv::Mat& T, // 添加引用参数
        std::function<void(int)> progressCallback = nullptr);

    void saveMat(std::ofstream& file, const std::string& name, const cv::Mat& mat);
    void saveScalar(std::ofstream& file, const std::string& name, double value);
    void saveSize(std::ofstream& file, const std::string& name, const cv::Size& size);
};

#endif // CALIBRATE_H
