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
//    // ���ÿ�������Ϳ�����ֵ
//    ProjectionControlThread(const ProjectionControlThread&) = delete;
//    ProjectionControlThread& operator=(const ProjectionControlThread&) = delete;
//
//    // ���п����߳�
//    void run();
//
//    // ֹͣ�߳�
//    void stop();
//
//signals:
//    void logMessage(QString message); // �ź����ڷ�����־��Ϣ
//
//private:
//    // ����豸����ָ��
//    CGXDevicePointer ObjDevicePtr1;
//    CGXStreamPointer ObjStreamPtr1;
//    CGXFeatureControlPointer ObjFeatureControlPtr1;
//
//    CGXDevicePointer ObjDevicePtr2;
//    CGXStreamPointer ObjStreamPtr2;
//    CGXFeatureControlPointer ObjFeatureControlPtr2;
//
//    // �����¼�������
//    std::unique_ptr<CSampleCaptureEventHandler1> pCaptureEventHandler1;
//    std::unique_ptr<CSampleCaptureEventHandler2> pCaptureEventHandler2;
//
//    // ͼ�񱣴�·��
//    std::string leftImageSavePath;
//    std::string rightImageSavePath;
//
//    // �����̵߳�ԭ�ӱ�־
//    std::atomic<bool> running;
//
//    // �ڲ�����
//    bool initializeProjector();
//    bool initializeCameras();
//    void startAcquisition();
//    void stopAcquisition();
//    void cleanup();
//
//    // ͼ�񱣴������
//    std::atomic<int> leftImageCount;
//    std::atomic<int> rightImageCount;
//};
//
//#endif // PROJECTIONCONTROLTHREAD_H
//
