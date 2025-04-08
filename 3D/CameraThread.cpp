#include "camerathread.h"
#include <QDebug>
#include <QImage>
#include <QDir>

CameraThread::CameraThread(uint32_t index, const QString& windowName, QObject* parent)
    : QThread(parent), index_(index), windowName_(windowName), stopFlag_(false), imageCount_(0) // 使用初始化列表来初始化构造函数的参数
{
}

CameraThread::~CameraThread()
{
    stop();
    wait();
    qDebug() << "Camera thread for camera" << index_ << "destroyed.";
}

void CameraThread::run()
{
    if (!cameraController_.initializeCamera(index_, windowName_))
    {
        qCritical() << "Failed to initialize camera" << index_;
        return;
    }

    if (!cameraController_.startAcquisition())
    {
        qCritical() << "Failed to start acquisition for camera" << index_;
        return;
    }

    qDebug() << "Camera" << index_ << "started successfully";

    while (!stopFlag_.load())
    {
        cv::Mat image;
        if (cameraController_.captureImage(image))
        {
            QImage qImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
            emit imageCaptured(index_, qImage.rgbSwapped());
        }
        else
        {
            qCritical() << "Failed to capture image from camera" << index_;
        }
        msleep(30);
    }

    cameraController_.stopAcquisition();
    cameraController_.releaseCamera();
}

void CameraThread::stop()
{
    if (stopFlag_.load())
    {
        return;
    }

    qDebug() << "Stopping camera thread for camera" << index_;
    stopFlag_.store(true);
    wait(); // 确保线程正确停止
    qDebug() << "Camera thread for camera" << index_ << "stopped.";
}

void CameraThread::captureAndSave(const QString& path)
{
    cv::Mat image;
    if (cameraController_.captureImage(image))
    {
        QString cameraName = (index_ == 1) ? "left" : "right";
        QString filename = path + QString("/%1_%2.bmp").arg(cameraName).arg(imageCount_ + 1); // 使用计数器生成唯一文件名
        cv::imwrite(filename.toStdString(), image);
        qDebug() << "Saved image:" << filename;
        imageCount_++; // 增加计数器
    }
    else
    {
        qCritical() << "Failed to capture image from camera" << index_;
    }
}
void CameraThread::setExposureTime(double exposureTime)
{
    cameraController_.setExposureTime(exposureTime);
}


