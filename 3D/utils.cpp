#include "utils.h"
#include <fstream>
#include <iomanip>
#include <sstream>

// 分割字符串
bool split(const string &line, vector<double> &res, const string &pattern) {
    if(line.empty())
        return false;
    res.clear();
    stringstream ss(line);
    string data;
    double v;
    char *ptr;
    while (getline(ss, data, '\t')) {
        v = strtod(data.c_str(), &ptr);
        res.emplace_back(v);
    }
    return !res.empty();
}

// 获取文件路径列表
bool glob(int T, int N, const string &folder, vector<string> &files) {
    files.clear();
    int num = T * N;
    for (int i = 1; i <= num; ++i) {
        string filename = folder + "/" + to_string(i) + ".bmp";
        files.emplace_back(filename);
    }
    return !files.empty();
}

// 获取左右文件路径列表
bool globLR(int T, int N, const string &folder, vector<string> &filesL, vector<string> &filesR) {
    filesL.clear(); filesR.clear();
    string folderL = folder + "/L";
    string folderR = folder + "/R";
    int num = T * N;
    for (int i = 1; i <= num; ++i) {
        string filename_L = folderL + "/" + to_string(i) + ".bmp";
        string filename_R = folderR + "/" + to_string(i) + ".bmp";
        filesL.emplace_back(filename_L);
        filesR.emplace_back(filename_R);
    }
    return (!filesL.empty()) && (!filesR.empty());
}

// 显示图像对
void showImagePair(const cv::Mat &imgL, const cv::Mat &imgR, const string &title, float s, bool line, bool color) {
    cv::Mat img_all;
    cv::hconcat(imgL, imgR, img_all);
    cv::resize(img_all, img_all, cv::Size(), s, s);
    double min_v, max_v;
    cv::Point min_p, max_p;
    cv::minMaxLoc(img_all, &min_v, &max_v, &min_p, &max_p);
    img_all = (img_all - min_v) * 255. / (max_v - min_v);
    img_all.convertTo(img_all, CV_8UC1);
    if (color) {
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

// 显示单个图像
void showImage(const cv::Mat &img, const string &title, float s, bool color) {
    cv::Mat img_s;
    cv::resize(img, img_s, cv::Size(), s, s);
    double min_v, max_v;
    cv::Point min_p, max_p;
    cv::minMaxLoc(img_s, &min_v, &max_v, &min_p, &max_p);
    img_s = (img_s - min_v) * 255. / (max_v - min_v);
    img_s.convertTo(img_s, CV_8UC1);
    if (color) {
        cv::applyColorMap(img_s, img_s, cv::COLORMAP_JET);
    }
    cv::imshow(title, img_s);
    cv::waitKey(1);
}

// 保存矩阵到文件
void saveMat(const string &filename, cv::Mat &mat) {
    ofstream f;
    f.open(filename);
    cout << "Writing...:\t" << filename << endl;
    for (int row = 0; row < mat.rows; ++row) {
        for (int col = 0; col < mat.cols; ++col) {
            if (col == mat.cols - 1) {
                f << setprecision(32) << mat.at<double>(row, col) << endl;
            } else {
                f << mat.at<double>(row, col) << ",";
            }
        }
    }
    f.close();
    cout << "Have Written into file:" << filename << endl;
}

// 模运算
double mod(double a, double m) {
    return a - m * floor(a / m);
}
