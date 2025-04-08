#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

// 分割字符串
bool split(const string &line, vector<double> &res, const string &pattern);

// 获取文件路径列表
bool glob(int T, int N, const string &folder, vector<string> &files);

// 获取左右文件路径列表
bool globLR(int T, int N, const string &folder, vector<string> &filesL, vector<string> &filesR);

// 显示图像对
void showImagePair(const cv::Mat &imgL, const cv::Mat &imgR, const string &title, float s, bool line, bool color);

// 显示单个图像
void showImage(const cv::Mat &img, const string &title, float s, bool color);

// 保存矩阵到文件
void saveMat(const string &filename, cv::Mat &mat);

// 模运算
double mod(double a, double m);

#endif // CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
