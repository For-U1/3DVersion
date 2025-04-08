#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_PHASER
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_PHASER

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "utils.h"

using namespace std;

#define pi acos(-1)

class Phaser {
public:
    void makePatterns(double A, double B, int N, int W, int H, double T1, double T2, double T3, const string &save_dir);

    double calcT123(double T1, double T2, double T3);

    void calcPhase(vector<string> &files, vector<string> &filesSimu,
                   int N, double T1, double T2, double T3,
                   cv::Mat &pha, cv::Mat &B,
                   const string &save_dir, bool write);

    void filterB(cv::Mat &pha, cv::Mat &B, double B_min);

    void filterGrad(cv::Mat &pha);

protected:
    double calcT12(double T1, double T2);

    void loadImages(int N, vector<string> &filesSimu, vector<string> &files,
                    vector<cv::Mat> &imgsSimu1, vector<cv::Mat> &imgsSimu2, vector<cv::Mat> &imgsSimu3,
                    vector<cv::Mat> &imgs1, vector<cv::Mat> &imgs2, vector<cv::Mat> &imgs3);

    void calcWrappedPhase(vector<cv::Mat> &imgs, int N, cv::Mat &pha, cv::Mat &B);

    void calcUnwrappedPhase(cv::Mat &pha1, cv::Mat &pha2, double T1, double T2, cv::Mat &pha12, double &T12);

    void toStdPhase(cv::Mat &pha, cv::Mat &phaSimu);
};

#endif // CH7_STRUCTURELIGHTSTEREOTRIANGLE_PHASER
