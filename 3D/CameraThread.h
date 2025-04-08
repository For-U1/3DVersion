#pragma once
#include <QThread>
#include <QString>
#include <opencv2/opencv.hpp>
#include "CameraController.h"
#include <atomic>

class CameraThread : public QThread
{
    Q_OBJECT
public:
    explicit CameraThread(uint32_t index, const QString& windowName, QObject* parent = nullptr); //声明构造函数
    ~CameraThread(); //声明析构函数

    void run() override; //成员函数
    void stop();
    void captureAndSave(const QString& path);
    void setExposureTime(double exposureTime);

signals:
    void imageCaptured(int cameraIndex, const QImage& image);

private:
    CameraController cameraController_;//私有成员变量
    uint32_t index_;
    QString windowName_;
    std::atomic<bool> stopFlag_;
    int imageCount_;  // 添加计数器
};


