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
//    wait(); // �ȴ��߳̽���
//}
//
//void CalibrateThread::run()
//{
//    CornerDetection cornerDetection;
//    Mat R, T;  // ���ڱ���궨���
//    qDebug() << "start calibratethread.";
//
//    while (!stopRequested_)
//    {
//        // ִ�б궨
//        bool success = cornerDetection.corner(calibDirPath_.toStdString(), true, savePath_.toStdString());
//
//        // ���� corner() �����ķ���ֵ�����궨����ź�
//        if (success)
//        {
//            emit calibrationFinished(true);
//            emit calibrationResults(R, T);  // ��������궨������ź�
//        }
//        else
//        {
//            qDebug() << "Calibration finished, but some errors occurred during the process.";
//            emit calibrationFinished(false);
//        }
//        qDebug() << "cailibration finished";
//        // �궨��ɺ��˳�ѭ��
//        break;
//    }
//}
//
//void CalibrateThread::stop()
//{
//    QMutexLocker locker(&mutex_);
//    stopRequested_ = true;
//    // ������� run() �������еȴ��߼����������֪ͨ�߳�
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
    wait(); // �ȴ��߳̽���
}

void CalibrateThread::run()
{
    CornerDetection cornerDetection;
    cv::Mat R, T;  // ���ڱ���궨���
    qDebug() << "Start calibration thread.";

    if (stopRequested_)
        return;

    auto progressCallback = [this](int value)
    {
        emit progressUpdated(value);
    };

    // ִ�б궨
    bool success = cornerDetection.corner(calibDirPath_.toStdString(), true, savePath_.toStdString(),
        boardWidth_, boardHeight_, squareSize_, R, T ,progressCallback);

    // ���� corner() �����ķ���ֵ�����궨����ź�
    if (success)
    {
        // ��ȡ R ���������
        QVector<double> R_data;
        R_data.reserve(R.rows * R.cols);
        for (int i = 0; i < R.rows; ++i)
        {
            for (int j = 0; j < R.cols; ++j)
            {
                R_data.append(R.at<double>(i, j));
            }
        }

        // ��ȡ T ���������
        QVector<double> T_data;
        T_data.reserve(T.rows * T.cols);
        for (int i = 0; i < T.rows; ++i)
        {
            for (int j = 0; j < T.cols; ++j)
            {
                T_data.append(T.at<double>(i, j));
            }
        }

        // �����źţ��������ݺ;���ߴ�
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
