#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_MATCHER_H
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_MATCHER_H

#include <opencv2/opencv.hpp>
#include "Spline.h"

using namespace std;

class Matcher {
public:
    bool init(int win_size, float pha_dif);
    bool phaseMatch(cv::Mat &phaL, cv::Mat &phaR, vector<pair<cv::Point2f, cv::Point2f>> &cps);

protected:
    bool is_valid_box(cv::Mat &BOX);
    int phase_search(cv::Mat &BOX_L, cv::Mat &phaR, int Y_R, int X_R_Start);
    double calc_std(cv::Mat &box1, cv::Mat &box2);
    double search_sub_pixel(cv::Mat &BOX_L, cv::Mat &phaR, int XR, int YR);

private:
    cv::Size winSize;
    int wh;
    float pha_dif;
};

#endif //CH7_STRUCTURELIGHTSTEREOTRIANGLE_MATCHER_H
