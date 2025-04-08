#pragma once
#include <QWidget>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include "camerathread.h"

    class Camerawidget : public QWidget
    {
        Q_OBJECT
    public:
        explicit Camerawidget(int num, QWidget* parent = nullptr);//构造函数
        ~Camerawidget();//析构函数

    private slots: //槽函数
        void openCamera();
        void captureImage();
        void closeCamera();
        void onImageCaptured(int cameraIndex, const QImage& image);
        void onExposureChanged(double exposure);  // 槽函数：曝光时间设置
        //void onGainChanged(double gain);          // 槽函数：增益设置

    private:
        CameraThread* cameraThreadLeft_;//左相机线程
        CameraThread* cameraThreadRight_;//右相机线程
        QLabel* leftCameraView_;  // 左相机画面显示
        QLabel* rightCameraView_; // 右相机画面显示
        //int index;//相机编号,未使用，暂时注销
        CameraController* cameraController_; // 添加 CameraController 成员变量
        QString savePath_;//图片保存路径
        QDoubleSpinBox* exposureSpinBox_;  // 相机曝光时间控件
        //QDoubleSpinBox* gainSpinBox_;      // 增益控件,暂时不加
    };

