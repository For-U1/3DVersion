//#include "Calibratethread.h"
//#include <QDebug>
//#include <QFileInfo>
//
//CalibrateThread::CalibrateThread(const QString& calibDirPath, const QString& savePath, QObject* parent)
//    : QThread(parent),
//    calibDirPath_(calibDirPath),
//    savePath_(savePath),
//    stopRequested_(false)
//{
//}
//
//CalibrateThread::~CalibrateThread()
//{
//    stop();
//    wait(); // 等待线程结束
//}
//
//void CalibrateThread::run()
//{
//    CornerDetection cornerDetection;
//    Mat R, T;  // 用于保存标定结果
//    qDebug() << "start calibratethread.";
//
//    while (!stopRequested_)
//    {
//        // 执行标定
//        bool success = cornerDetection.corner(calibDirPath_.toStdString(), true, savePath_.toStdString());
//
//        // 根据 corner() 方法的返回值发出标定完成信号
//        if (success)
//        {
//            emit calibrationFinished(true);
//            emit calibrationResults(R, T);  // 发射包含标定结果的信号
//        }
//        else
//        {
//            qDebug() << "Calibration finished, but some errors occurred during the process.";
//            emit calibrationFinished(false);
//        }
//        qDebug() << "cailibration finished";
//        // 标定完成后退出循环
//        break;
//    }
//}
//
//void CalibrateThread::stop()
//{
//    QMutexLocker locker(&mutex_);
//    stopRequested_ = true;
//    // 如果你在 run() 方法中有等待逻辑，这里可以通知线程
//    waitCondition_.wakeAll();
//}
//
//
// calibratethread.cpp

#include "calibratethread.h"
#include <QDebug>

CalibrateThread::CalibrateThread(const QString& calibDirPath, const QString& savePath,
    int boardWidth, int boardHeight, double squareSize, QObject* parent)
    : QThread(parent),
    calibDirPath_(calibDirPath),
    savePath_(savePath),
    boardWidth_(boardWidth),
    boardHeight_(boardHeight),
    squareSize_(squareSize),
    stopRequested_(false)
{
}

CalibrateThread::~CalibrateThread()
{
    stop();
    wait(); // 等待线程结束
}

void CalibrateThread::run()
{
    CornerDetection cornerDetection;
    cv::Mat R, T;  // 用于保存标定结果
    qDebug() << "Start calibration thread.";

    if (stopRequested_)
        return;

    auto progressCallback = [this](int value)
    {
        emit progressUpdated(value);
    };

    // 执行标定
    bool success = cornerDetection.corner(calibDirPath_.toStdString(), true, savePath_.toStdString(),
        boardWidth_, boardHeight_, squareSize_, R, T ,progressCallback);

    // 根据 corner() 方法的返回值发出标定完成信号
    if (success)
    {
        // 提取 R 矩阵的数据
        QVector<double> R_data;
        R_data.reserve(R.rows * R.cols);
        for (int i = 0; i < R.rows; ++i)
        {
            for (int j = 0; j < R.cols; ++j)
            {
                R_data.append(R.at<double>(i, j));
            }
        }

        // 提取 T 矩阵的数据
        QVector<double> T_data;
        T_data.reserve(T.rows * T.cols);
        for (int i = 0; i < T.rows; ++i)
        {
            for (int j = 0; j < T.cols; ++j)
            {
                T_data.append(T.at<double>(i, j));
            }
        }

        // 发射信号，传递数据和矩阵尺寸
        emit calibrationResults(R_data, R.rows, R.cols, T_data, T.rows, T.cols);
    }
    else
    {
        qDebug() << "Calibration finished, but some errors occurred during the process.";
        emit calibrationFinished(false);
    }
    qDebug() << "Calibration finished";
}

void CalibrateThread::stop()
{
    QMutexLocker locker(&mutex_);
    stopRequested_ = true;
    waitCondition_.wakeAll();
}
