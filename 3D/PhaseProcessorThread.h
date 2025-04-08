//#pragma once
//#ifndef PHASEPROCESSORTHREAD_H
//#define PHASEPROCESSORTHREAD_H
//
//#include <QThread>
//#include <QMutex>
//#include <memory>
//#include <opencv2/opencv.hpp>
//#include "Phaser.h"
//#include "Calibrator.h"
//#include "Config.h"
//#include "Matcher.h"
//#include "Model.h"
//
//// ��λ�̴߳����࣬������λ����ĺ�̨����
//class PhaseProcessorThread : public QThread
//{
//    Q_OBJECT
//
////public:
//    /*explicit PhaseProcessorThread(const std::shared_ptr<Config>& config, QObject* parent = nullptr);
//    ~PhaseProcessorThread();*/
//public:
//    explicit PhaseProcessorThread(const std::shared_ptr<Config>& config);
//    ~PhaseProcessorThread();
//
//
//    // ������Ա����������������ֹͣ����
//    void startProcessing();  // ��������
//    void stopProcessing();   // ֹͣ����
//
//signals:
//    void processingStarted();  // ����ʼ�ź�
//    void processingStopped();  // ����ֹͣ�ź�
//
//protected:
//    void run() override;  // ��д run ����
//
//private:
//    bool getStopFlag();             // ��ȡ stopFlag
//    void setStopFlag(bool value);   // ���� stopFlag
//    bool stopFlag = false;          // ���ƴ���ֹͣ�ı�־
//    QMutex mutex;                   // ���ڱ��� stopFlag �Ļ�����
//    std::shared_ptr<Config> config; // ���ö���
//    Phaser phaser;                  // ��λ�������
//    Calibrator calibrator;          // ���У׼����
//    Matcher matcher;                // ��λƥ�����
//    Model model;                    // ����ģ�Ͷ���
//    clock_t start_time, end_time;   // ���ڲ�������ʱ��
//};
//
//#endif // PHASEPROCESSORTHREAD_H

//#ifndef PHASEPROCESSORTHREAD_H
//#define PHASEPROCESSORTHREAD_H
//#include <QThread>
//#include <atomic>
//#include <memory>
//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include "Phaser.h"
//#include "Calibrator.h"
//#include "Config.h"
//#include "Matcher.h"
//#include "Model.h"
//
//class PhaseProcessorThread : public QThread
//{
//    Q_OBJECT
//
//public:
//    explicit PhaseProcessorThread(std::shared_ptr<Config> config, QObject* parent = nullptr);
//    ~PhaseProcessorThread();
//
//    // ������Ա����������������ֹͣ����
//    void startProcessing();  // ��������
//    void stopProcessing();   // ֹͣ����
//
//signals:
//    void processingStarted();           // ����ʼ�ź�
//    void processingStopped();           // ����ֹͣ�ź�
//    void progressUpdated(int percentage); // ���ȸ����ź�
//
//protected:
//    void run() override;  // ��д run ����
//
//private:
//    bool getStopFlag();             // ��ȡ stopFlag
//    void setStopFlag(bool value);   // ���� stopFlag
//    std::atomic<bool> m_stopFlag;   // ���ƴ���ֹͣ�ı�־
//    std::shared_ptr<Config> m_config; // ���ö���
//    Phaser phaser;                  // ��λ�������
//    Calibrator calibrator;          // ���У׼����
//    Matcher matcher;                // ��λƥ�����
//    Model model;                    // ����ģ�Ͷ���
//    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time; // ���ڲ�������ʱ��
//};
//
//#endif // PHASEPROCESSORTHREAD_H
#ifndef PHASEPROCESSORTHREAD_H
#define PHASEPROCESSORTHREAD_H

#include <QThread>
#include <QMutex>
#include <memory>
#include <opencv2/opencv.hpp>
#include "calibrator.h"
#include "Phaser.h"
#include "Config.h"
#include "Matcher.h"
#include "Model.h"


// ��λ�̴߳����࣬������λ����ĺ�̨����
class PhaseProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit PhaseProcessorThread(Config* config);
    ~PhaseProcessorThread();

    // ������Ա����������������ֹͣ����
    void startProcessing();  // ��������
    void stopProcessing();   // ֹͣ����

signals:
    void processingStarted();  // ����ʼ�ź�
    void processingStopped();  // ����ֹͣ�ź�
    void processingProgress(int value); // ��������ź�

protected:
    void run() override;  // ��д run ����

private:
    bool getStopFlag();             // ��ȡ stopFlag
    void setStopFlag(bool value);   // ���� stopFlag
    bool stopFlag = false;          // ���ƴ���ֹͣ�ı�־
    QMutex mutex;                   // ���ڱ��� stopFlag �Ļ�����
    Config* config; // ���ö���ָ��
    Phaser phaser;                  // ��λ�������
    Calibration calibrator;          // ���У׼����
    Matcher matcher;                // ��λƥ�����
    Model model;                    // ����ģ�Ͷ���
    clock_t start_time, end_time;   // ���ڲ�������ʱ��
};

#endif // PHASEPROCESSORTHREAD_H
