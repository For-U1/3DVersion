//#pragma once
//#include <QThread>
//#include <QString>
//#include <QMutex>
//#include <QWaitCondition>
//#include "Calibrate.h"
//
//class CalibrateThread : public QThread
//{
//    Q_OBJECT
//
//public:
//    explicit CalibrateThread(const QString& calibDirPath, const QString& savePath, QObject* parent = nullptr);
//    ~CalibrateThread();
//
//    void run() override;
//    void stop();
//
//signals:
//    void calibrationFinished(bool success);
//
//    void calibrationResults(const Mat& R, const Mat& T);  // 新增信号，用于传递旋转矩阵和平移向量
//
//    void progressUpdated(int progress);  // 定义进度更新信号
//private:
//    QString calibDirPath_;
//    QString savePath_;
//    QMutex mutex_;
//    QWaitCondition waitCondition_;
//    bool stopRequested_;
//};
//
// calibratethread.h

#ifndef CALIBRATETHREAD_H
#define CALIBRATETHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <opencv2/opencv.hpp>
#include "calibrate.h"

class CalibrateThread : public QThread
{
    Q_OBJECT
public:
    CalibrateThread(const QString& calibDirPath, const QString& savePath,
        int boardWidth, int boardHeight, double squareSize, QObject* parent = nullptr);
    ~CalibrateThread();

    void run() override;
    void stop();

signals:
    void calibrationFinished(bool success);
    void calibrationResults(const QVector<double>& R_data, int R_rows, int R_cols,
        const QVector<double>& T_data, int T_rows, int T_cols);
    void progressUpdated(int value);  // 进度更新信号

private:
    QString calibDirPath_;
    QString savePath_;
    int boardWidth_;
    int boardHeight_;
    double squareSize_;
    bool stopRequested_;
    QMutex mutex_;
    QWaitCondition waitCondition_;
};

#endif // CALIBRATETHREAD_H
