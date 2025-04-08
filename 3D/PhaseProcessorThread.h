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
//// 相位线程处理类，用于相位处理的后台任务
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
//    // 公共成员函数，用于启动和停止处理
//    void startProcessing();  // 启动处理
//    void stopProcessing();   // 停止处理
//
//signals:
//    void processingStarted();  // 处理开始信号
//    void processingStopped();  // 处理停止信号
//
//protected:
//    void run() override;  // 重写 run 函数
//
//private:
//    bool getStopFlag();             // 获取 stopFlag
//    void setStopFlag(bool value);   // 设置 stopFlag
//    bool stopFlag = false;          // 控制处理停止的标志
//    QMutex mutex;                   // 用于保护 stopFlag 的互斥锁
//    std::shared_ptr<Config> config; // 配置对象
//    Phaser phaser;                  // 相位处理对象
//    Calibrator calibrator;          // 相机校准对象
//    Matcher matcher;                // 相位匹配对象
//    Model model;                    // 点云模型对象
//    clock_t start_time, end_time;   // 用于测量处理时间
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
//    // 公共成员函数，用于启动和停止处理
//    void startProcessing();  // 启动处理
//    void stopProcessing();   // 停止处理
//
//signals:
//    void processingStarted();           // 处理开始信号
//    void processingStopped();           // 处理停止信号
//    void progressUpdated(int percentage); // 进度更新信号
//
//protected:
//    void run() override;  // 重写 run 函数
//
//private:
//    bool getStopFlag();             // 获取 stopFlag
//    void setStopFlag(bool value);   // 设置 stopFlag
//    std::atomic<bool> m_stopFlag;   // 控制处理停止的标志
//    std::shared_ptr<Config> m_config; // 配置对象
//    Phaser phaser;                  // 相位处理对象
//    Calibrator calibrator;          // 相机校准对象
//    Matcher matcher;                // 相位匹配对象
//    Model model;                    // 点云模型对象
//    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, end_time; // 用于测量处理时间
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


// 相位线程处理类，用于相位处理的后台任务
class PhaseProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit PhaseProcessorThread(Config* config);
    ~PhaseProcessorThread();

    // 公共成员函数，用于启动和停止处理
    void startProcessing();  // 启动处理
    void stopProcessing();   // 停止处理

signals:
    void processingStarted();  // 处理开始信号
    void processingStopped();  // 处理停止信号
    void processingProgress(int value); // 处理进度信号

protected:
    void run() override;  // 重写 run 函数

private:
    bool getStopFlag();             // 获取 stopFlag
    void setStopFlag(bool value);   // 设置 stopFlag
    bool stopFlag = false;          // 控制处理停止的标志
    QMutex mutex;                   // 用于保护 stopFlag 的互斥锁
    Config* config; // 配置对象指针
    Phaser phaser;                  // 相位处理对象
    Calibration calibrator;          // 相机校准对象
    Matcher matcher;                // 相位匹配对象
    Model model;                    // 点云模型对象
    clock_t start_time, end_time;   // 用于测量处理时间
};

#endif // PHASEPROCESSORTHREAD_H
