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
    explicit CameraThread(uint32_t index, const QString& windowName, QObject* parent = nullptr); //�������캯��
    ~CameraThread(); //������������

    void run() override; //��Ա����
    void stop();
    void captureAndSave(const QString& path);
    void setExposureTime(double exposureTime);

signals:
    void imageCaptured(int cameraIndex, const QImage& image);

private:
    CameraController cameraController_;//˽�г�Ա����
    uint32_t index_;
    QString windowName_;
    std::atomic<bool> stopFlag_;
    int imageCount_;  // ��Ӽ�����
};


