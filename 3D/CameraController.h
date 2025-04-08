#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#pragma once

#include <QObject>
#include <QString>
#include <QMutex>
#include <opencv2/opencv.hpp>
#include <GxIAPI.h>
#include <DxImageProc.h>

class CameraController : public QObject
{
    Q_OBJECT
public:
    explicit CameraController(QObject* parent = nullptr);
    ~CameraController();

    bool initializeCamera(uint32_t index, const QString& windowName); //成员函数
    void releaseCamera();
    bool captureImage(cv::Mat& image);
    bool startAcquisition();
    bool stopAcquisition();
    bool listDevices(); // 查找当前设备数
    bool setExposureTime(double exposureTime);
    bool setGain(double gain);

signals:
    void imageCaptured(int cameraIndex, const QImage& image);

private:
    struct Camera
    {
        GX_DEV_HANDLE handle = nullptr;
        BYTE* bufferRaw = nullptr;
        BYTE* bufferRGB = nullptr;
        int64_t imageHeight = 0;
        int64_t imageWidth = 0;
        int64_t payloadSize = 0;
        int64_t pixelColorFilter = 0;
        cv::Mat frame;
        QString windowName;
    };
    QMutex mutex_; // 添加互斥锁
    Camera camera_;
};

#endif // CAMERACONTROLLER_H


