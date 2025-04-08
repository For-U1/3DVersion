//// ProjectionControlThread.h
//#pragma once
//#ifndef PROJECTIONCONTROLTHREAD_H
//#define PROJECTIONCONTROLTHREAD_H
//
//#include <iostream>
//#include <mutex>
//#include <memory>
//#include <opencv2/opencv.hpp>
//#include <string>
//#include <thread>
//#include <atomic>
//#include <QObject>
//#include <QImage>
//#include "ProjectControl.h"
//
//class ProjectionControlThread : public QObject
//{
//    Q_OBJECT
//
//public:
//    ProjectionControlThread(const std::string& leftPath, const std::string& rightPath);
//    ~ProjectionControlThread();
//
//    // 禁用拷贝构造和拷贝赋值
//    ProjectionControlThread(const ProjectionControlThread&) = delete;
//    ProjectionControlThread& operator=(const ProjectionControlThread&) = delete;
//
//    // 运行控制线程
//    void run();
//
//    // 停止线程
//    void stop();
//
//signals:
//    void logMessage(QString message); // 信号用于发送日志消息
//
//private:
//    // 相机设备和流指针
//    CGXDevicePointer ObjDevicePtr1;
//    CGXStreamPointer ObjStreamPtr1;
//    CGXFeatureControlPointer ObjFeatureControlPtr1;
//
//    CGXDevicePointer ObjDevicePtr2;
//    CGXStreamPointer ObjStreamPtr2;
//    CGXFeatureControlPointer ObjFeatureControlPtr2;
//
//    // 捕获事件处理器
//    std::unique_ptr<CSampleCaptureEventHandler1> pCaptureEventHandler1;
//    std::unique_ptr<CSampleCaptureEventHandler2> pCaptureEventHandler2;
//
//    // 图像保存路径
//    std::string leftImageSavePath;
//    std::string rightImageSavePath;
//
//    // 控制线程的原子标志
//    std::atomic<bool> running;
//
//    // 内部方法
//    bool initializeProjector();
//    bool initializeCameras();
//    void startAcquisition();
//    void stopAcquisition();
//    void cleanup();
//
//    // 图像保存计数器
//    std::atomic<int> leftImageCount;
//    std::atomic<int> rightImageCount;
//};
//
//#endif // PROJECTIONCONTROLTHREAD_H
//
