#include "Matcher.h"
#include <cmath>
//#include <iostream>
#include <QDebug>

bool Matcher::
    init(int win_size, float pha_dif) {
    winSize = cv::Size(win_size, win_size);
    wh = win_size / 2;
    this->pha_dif = pha_dif;
    return false;
}

bool Matcher::phaseMatch(cv::Mat &phaL, cv::Mat &phaR, vector<pair<cv::Point2f, cv::Point2f>> &cps)
{
    int XR;
    bool flag_find;

    qCritical() << "start phase match!" ;
    for (int Y_L = wh; Y_L < phaL.rows - wh; Y_L++)
    {
        bool flag_glob_search = true;
        for (int X_L = wh; X_L < phaL.cols - wh; X_L++)
        {
            cv::Rect roi_L(cv::Point(X_L - wh, Y_L - wh), winSize);
            cv::Mat BOX_L = phaL(roi_L);
            flag_find = false;
            if (is_valid_box(BOX_L))
            {
                if (flag_glob_search)
                {
                    XR = phase_search(BOX_L, phaR, Y_L, wh);
                    flag_find = (XR != -1);
                    if (flag_find)
                    {
                        flag_glob_search = false;
                    }
                }
                else
                {
                    int X_R_PRE = XR;
                    XR = phase_search(BOX_L, phaR, Y_L, X_R_PRE - 3);
                    flag_find = (XR != -1);
                    if (!flag_find)
                    {
                        flag_glob_search = true;
                    }
                }
            }

            if (flag_find)
            {
                double XR_new = search_sub_pixel(BOX_L, phaR, XR, Y_L);
                cv::Point2f p_L, p_R;
                p_L.x = float(X_L);
                p_L.y = float(Y_L);
                p_R.x = float(XR_new);
                p_R.y = float(Y_L);
                cps.emplace_back(p_L, p_R);
            }
        }
    }
    qCritical() << "size of matched:" << cps.size() ;
    return !cps.empty();
}

bool Matcher::is_valid_box(cv::Mat &BOX)
{
    double min_v, max_v;
    cv::Point min_pt, max_pt;
    cv::minMaxLoc(BOX, &min_v, &max_v, &min_pt, &max_pt);
    return min_v > 0;
}

int Matcher::phase_search(cv::Mat &BOX_L, cv::Mat &phaR, int Y_R, int X_R_Start)
{
    double l = BOX_L.at<double>(wh, wh);
    vector<pair<cv::Mat, int>> ps;

    for (int X_R = X_R_Start; X_R < phaR.cols - wh; X_R++)
    {
        double r = phaR.at<double>(Y_R, X_R);
        if (abs(l - r) <= pha_dif)
        {
            cv::Rect roi_R(cv::Point(X_R - wh, Y_R - wh), winSize);
            cv::Mat BOX_R = phaR(roi_R);
            if (is_valid_box(BOX_R))
            {
                ps.emplace_back(BOX_R, X_R);
            }
        }
    }

    if (ps.empty())
    {
        return -1;
    }

    double min_v = 10000;
    int XR = -1;
    for (auto &p: ps)
    {
        double v = calc_std(BOX_L, p.first);
        if (v < min_v)
        {
            min_v = v;
            XR = p.second;
        }
    }
    return XR;
}

double Matcher::calc_std(cv::Mat &box1, cv::Mat &box2)
{
    cv::Mat v_mat;
    cv::pow(box2 - box1, 2, v_mat);
    return sqrt(cv::sum(v_mat).val[0]);
}

double Matcher::search_sub_pixel(cv::Mat &BOX_L, cv::Mat &phaR, int XR, int YR)
{
    if (XR - 2 < 0 || XR + 2 > phaR.cols)
    {
        return double(XR);
    }

    double xs[5], ys[5];
    int idx = 0;
    for (int i = -2; i <= 2; ++i)
    {
        double pha_v = phaR.at<double>(YR, XR + i);
        if (pha_v == 0)
        {
            return double(XR);
        }
        xs[idx] = pha_v;
        ys[idx] = double(XR + i);
        idx += 1;
    }

    SplineSpace::SplineInterface *sp = new SplineSpace::Spline(xs, ys, 5);
    double x = BOX_L.at<double>(wh, wh);
    if (x >= *min_element(xs, xs + 5) && x <= *max_element(xs, xs + 5))
    {
        double y;
        sp->SinglePointInterp(x, y);
        delete sp; // 释放内存
        return y;
    }
    else
    {
        cv::Mat temp;
        cv::Rect ROI_R(cv::Point(XR, YR), winSize);
        cv::Mat BOX_R = phaR(ROI_R);
        cv::resize(BOX_R, temp, cv::Size(10, 9), 0, 0, cv::INTER_LINEAR);
        double min_val = 10000;
        double pos = 0;
        double sep = 3. / 10;
        for (int c = 0; c < temp.cols - 2; ++c)
        {
            cv::Rect roi(cv::Point(c, 3), winSize);
            cv::Mat BOX_R_temp = temp(roi);
            double val = calc_std(BOX_L, BOX_R_temp);
            if (val < min_val)
            {
                min_val = val;
                pos = sep + sep * c;
            }
        }
        return XR - 3. / 2 + pos;
    }
}
