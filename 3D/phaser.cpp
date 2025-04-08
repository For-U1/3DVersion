#include "Phaser.h"
#include <fstream>
#include <iostream>
#include <cmath>

void Phaser::makePatterns(double A, double B, int N, int W, int H, double T1, double T2, double T3, const string &save_dir) {
    cv::Mat I = cv::Mat::zeros(cv::Size(W, 1), CV_64FC1);
    cv::Mat I_img = cv::Mat::zeros(cv::Size(W, H), CV_64FC1);
    int idx = 1;
    cout << "\nsave phase patterns into folder:\t" << save_dir << "..." << endl;
    ofstream f;
    f.open(save_dir + "/" + "write.csv", ios::out);
    for (auto T: {T1, T2, T3}) {
        for (int k = 0; k < N; ++k) {
            for (int w = 1; w <= W; ++w) {
                double I_value = A + B * cos(w / double (T) * 2. * pi + 2 * double(k) / double(N) * pi);
                I.at<double>(w - 1) = I_value;
                if (w == W) {
                    f << to_string(int(I_value)) << "\n";
                } else {
                    f << to_string(int(I_value)) << ",";
                }
            }
            cv::repeat(I, H, 1, I_img);
            string save_file = save_dir + "\\" + to_string(idx) + ".bmp";
            cv::imwrite(save_file, I_img);
            idx += 1;
        }
    }
    f.close();
}

double Phaser::calcT123(double T1, double T2, double T3) {
    double T12 = calcT12(T1, T2);
    double T23 = calcT12(T2, T3);
    return calcT12(T12, T23);
}

void Phaser::calcPhase(vector<string> &files, vector<string> &filesSimu,
                       int N, double T1, double T2, double T3,
                       cv::Mat &pha, cv::Mat &B,
                       const string &save_dir, bool write) {
    vector<cv::Mat> imgsSimu1, imgsSimu2, imgsSimu3;
    vector<cv::Mat> imgs1, imgs2, imgs3;

    loadImages(N, filesSimu, files, imgsSimu1, imgsSimu2, imgsSimu3, imgs1, imgs2, imgs3);

    cv::Mat phaSimu1, phaSimu2, phaSimu3, B_Simu;
    cv::Mat pha1, pha2, pha3, B1, B2, B3;

    calcWrappedPhase(imgsSimu1, N, phaSimu1, B_Simu);
    calcWrappedPhase(imgsSimu2, N, phaSimu2, B_Simu);
    calcWrappedPhase(imgsSimu3, N, phaSimu3, B_Simu);
    calcWrappedPhase(imgs1, N, pha1, B1);
    calcWrappedPhase(imgs2, N, pha2, B2);
    calcWrappedPhase(imgs3, N, pha3, B3);
    B = (B1 + B2 + B3) / 3.;

    cv::Mat phaSimu12, phaSimu23;
    cv::Mat pha12, pha23;
    double T12, T23;

    calcUnwrappedPhase(phaSimu1, phaSimu2, T1, T2, phaSimu12, T12);
    calcUnwrappedPhase(phaSimu2, phaSimu3, T2, T3, phaSimu23, T23);
    calcUnwrappedPhase(pha1, pha2, T1, T2, pha12, T12);
    calcUnwrappedPhase(pha2, pha3, T2, T3, pha23, T23);

    toStdPhase(pha12, phaSimu12);
    toStdPhase(pha23, phaSimu23);

    cv::Mat pha123, phaSimu123;
    double T123;
    calcUnwrappedPhase(phaSimu12, phaSimu23, T12, T23, phaSimu123, T123);
    calcUnwrappedPhase(pha12, pha23, T12, T23, pha123, T123);

    toStdPhase(pha123, phaSimu123);

    cv::Mat mask1 = (pha123 < 2 * pi) / 255.;
    cv::Mat mask2 = (pha123 > 0.0) / 255.;
    mask1.convertTo(mask1, CV_64FC1);
    mask2.convertTo(mask2, CV_64FC1);
    pha = pha123.mul(mask1).mul(mask2);

    if (write) {
        saveMat(save_dir + "/" + "B.txt", B);
        saveMat(save_dir + "/" + "phaSimu12.txt", phaSimu12);
        saveMat(save_dir + "/" + "phaSimu23.txt", phaSimu23);
        saveMat(save_dir + "/" + "pha12.txt", pha12);
        saveMat(save_dir + "/" + "pha23.txt", pha23);
        saveMat(save_dir + "/" + "phaSimu123.txt", phaSimu123);
        saveMat(save_dir + "/" + "pha123.txt", pha123);
    }
}

void Phaser::filterB(cv::Mat &pha, cv::Mat &B, double B_min) {
    cv::Mat B_mask = (B > B_min) / 255.;
    B_mask.convertTo(B_mask, CV_64FC1);
    pha = pha.mul(B_mask);
}

void Phaser::filterGrad(cv::Mat &pha) {
    cv::Mat dx, dy;
    cv::Sobel(pha, dx, pha.depth(), 1, 0, 1, 0.5, 0); // x方向
    cv::Sobel(pha, dy, pha.depth(), 0, 1, 1, 0.5, 0); // y方向

    float d_thr = 0.2;
    cv::Mat dx_mask = (cv::abs(dx) < d_thr) / 255.;
    cv::Mat dy_mask = (cv::abs(dy) < d_thr) / 255.;

    dx_mask.convertTo(dx_mask, CV_64FC1);
    dy_mask.convertTo(dy_mask, CV_64FC1);

    cv::Mat dxy_mask = dx_mask.mul(dy_mask);
    cv::erode(dxy_mask, dxy_mask, cv::Mat());

    for (int row = 0; row < dxy_mask.rows; ++row) {
        for (int col = 0; col < dxy_mask.cols; ++col) {
            if (dxy_mask.at<double>(row, col) < 0.5) {
                pha.at<double>(row, col) = 0;
            }
        }
    }
}

double Phaser::calcT12(double T1, double T2) {
    return (T1 * T2) / (T2 - T1);
}

void Phaser::loadImages(int N, vector<string> &filesSimu, vector<string> &files,
                        vector<cv::Mat> &imgsSimu1, vector<cv::Mat> &imgsSimu2, vector<cv::Mat> &imgsSimu3,
                        vector<cv::Mat> &imgs1, vector<cv::Mat> &imgs2, vector<cv::Mat> &imgs3) {
    int num = files.size();
    for (int i = 0; i < num; ++i) {
        string fileSimu = filesSimu[i];
        string fileModel = files[i];

        cv::Mat imgSimu = cv::imread(fileSimu, 0);
        if (imgSimu.empty()) {
            cerr << "unable to load image:\t" << fileSimu << endl;
            return;
        }

        cv::Mat imgModel = cv::imread(fileModel, 0);
        if (imgModel.empty()) {
            cerr << "unable to load image:\t" << fileModel << endl;
            return;
        }

        cv::GaussianBlur(imgSimu, imgSimu, cv::Size(3, 3), 1, 1);
        cv::GaussianBlur(imgModel, imgModel, cv::Size(3, 3), 1, 1);

        imgSimu.convertTo(imgSimu, CV_64FC1);
        imgModel.convertTo(imgModel, CV_64FC1);

        int r = i / N;
        if (r == 0) {
            imgsSimu1.emplace_back(imgSimu);
            imgs1.emplace_back(imgModel);
        } else if (r == 1) {
            imgsSimu2.emplace_back(imgSimu);
            imgs2.emplace_back(imgModel);
        } else {
            imgsSimu3.emplace_back(imgSimu);
            imgs3.emplace_back(imgModel);
        }
    }
}

void Phaser::calcWrappedPhase(vector<cv::Mat> &imgs, int N, cv::Mat &pha, cv::Mat &B) {
    cv::Size img_size = imgs[0].size();
    pha = cv::Mat::zeros(img_size, CV_64FC1);
    B = cv::Mat::zeros(img_size, CV_64FC1);
    cv::Mat Ik = cv::Mat::zeros(img_size, CV_64FC1);
    cv::Mat sin_sum = cv::Mat::zeros(img_size, CV_64FC1);
    cv::Mat cos_sum = cv::Mat::zeros(img_size, CV_64FC1);

    for (int k = 0; k < N; k++) {
        double pk = 2. * double(k) / double(N) * pi;
        sin_sum += imgs[k] * sin(pk);
        cos_sum += imgs[k] * cos(pk);
    }

    for (int row = 0; row < Ik.rows; ++row) {
        for (int col = 0; col < Ik.cols; ++col) {
            double s = sin_sum.at<double>(row, col);
            double c = cos_sum.at<double>(row, col);
            pha.at<double>(row, col) = -atan2(s, c);
            B.at<double>(row, col) = sqrt(s * s + c * c) * 2. / double(N);
        }
    }
}

void Phaser::calcUnwrappedPhase(cv::Mat &pha1, cv::Mat &pha2, double T1, double T2, cv::Mat &pha12, double &T12) {
    pha12 = cv::Mat::zeros(pha1.size(), CV_64FC1);
    T12 = T1 * T2 / (T2 - T1);

    for (int row = 0; row < pha1.rows; ++row) {
        for (int col = 0; col < pha1.cols; ++col) {
            double pha1_v = pha1.at<double>(row, col);
            double pha2_v = pha2.at<double>(row, col);
            double delta = mod(pha1_v - pha2_v, 2. * pi);
            pha12.at<double>(row, col) = pha1_v + round((delta * T12 / T1 - pha1_v) / (2. * pi)) * 2. * pi;
        }
    }
}

void Phaser::toStdPhase(cv::Mat &pha, cv::Mat &phaSimu) {
    double e = 0.2;
    double pha_min = 0.0, pha_max = 0.0;
    double *min_p = &pha_min, *max_p = &pha_max;
    cv::minMaxIdx(phaSimu, min_p, max_p);
    phaSimu = (phaSimu - pha_min) / (pha_max - pha_min + e) * 2. * pi;
    pha = (pha - pha_min) / (pha_max - pha_min + e) * 2. * pi;
}
