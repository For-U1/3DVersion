//
//// ProjectionControlThread.cpp 
//#include "ProjectionControlThread.h"
////#include "dlpc_common.h"
////#include "dlpc34xx.h"
////#include "dlpc347x_internal_patterns.h"
////#include "cypress_i2c.h"
//#include <filesystem>
//#include <stdexcept>
//namespace fs = std::filesystem;
//using namespace std;
//
//
//// 构造函数，接受保存路径
//ProjectionControlThread::ProjectionControlThread(const std::string& leftPath, const std::string& rightPath)
//    : ObjDevicePtr1(nullptr),
//    ObjStreamPtr1(nullptr),
//    ObjFeatureControlPtr1(nullptr),
//    ObjDevicePtr2(nullptr),
//    ObjStreamPtr2(nullptr),
//    ObjFeatureControlPtr2(nullptr),
//    pCaptureEventHandler1(nullptr),
//    pCaptureEventHandler2(nullptr),
//    leftImageSavePath(leftPath),
//    rightImageSavePath(rightPath),
//    running(true)
//{
//}
//
//// 析构函数
//ProjectionControlThread::~ProjectionControlThread()
//{
//    stop();
//    cleanup();
//}
//
//// 运行控制线程
//void ProjectionControlThread::run()
//{
//    try
//    {
//        
//        // 初始化投影仪
//        if (!initializeProjector()) 
//        {
//            emit logMessage("Projector initialization failed, terminating operation.");
//            return;
//        }
//
//        // 初始化相机
//        if (!initializeCameras()) 
//        {
//            emit logMessage("Camera initialization failed, terminating operation.");
//            cleanup();
//            return;
//        }
//
//        // 启动图像采集和投影
//        startAcquisition();
//        Sleep(100);
//        emit logMessage("Projection started");
//        // 设置操作模式
//        /*DLPC34XX_WriteOperatingModeSelect(DLPC34XX_OM_SENS_INTERNAL_PATTERN);
//        DLPC34XX_WriteInternalPatternControl(DLPC34XX_PC_START, 0x0);*/
//        // 设置操作模式为内部图案模式
//        Sleep(100);
//        uint32_t ret = DLPC34XX_WriteOperatingModeSelect(DLPC34XX_OM_SENS_INTERNAL_PATTERN);
//        if (ret != SUCCESS)
//        {
//            emit logMessage(QString("Failed to set projector to internal pattern mode. Error code: %1").arg(ret));
//            cleanup();
//            return;
//        }
//
//        // 启动内部图案投影
//        ret = DLPC34XX_WriteInternalPatternControl(DLPC34XX_PC_START, 0x0);
//        if (ret != SUCCESS)
//        {
//            emit logMessage("Failed to start internal pattern projection.");
//            cleanup();
//            return;
//        }
//        else
//        {
//            emit logMessage("Internal pattern projection started.");
//        }
//        Sleep(900);
//         //持续运行，直到接收到停止信号
//        while (running)
//        {
//#ifdef _WIN32
//            Sleep(100); // 100 毫秒
//#else
//            usleep(100 * 1000); // 100 毫秒
//#endif
//        }
//
//        // 停止采集和清理资源
//        stopAcquisition();
//        cleanup();
//
//        emit logMessage("Projection and acquisition stopped.");
//    }
//    catch (const std::exception& ex)
//    {
//        emit logMessage(QString("Exception occurred during execution: %1").arg(ex.what()));
//        cleanup();
//    }
//
//    
//}
//
//// 停止线程
//void ProjectionControlThread::stop()
//{
//    if (running.exchange(false)) {
//        emit logMessage("Stop command issued");
//    }
//}
//
//// 初始化投影仪
//bool ProjectionControlThread::initializeProjector()
//{
//    InitConnectionAndCommandLayer();
//    bool Status = CYPRESS_I2C_RequestI2CBusAccess();
//
//    if (Status != true) 
//    {
//        emit logMessage("Unable to open projector");
//        return false;
//    }
//    else
//    {
//        emit logMessage("Projector connection successfully");
//    }
//    DLPC34XX_ControllerDeviceId_e DeviceId = DLPC34XX_CDI_DLPC3479;
//    DLPC34XX_ReadControllerDeviceId(&DeviceId);
//    uint16_t PixelsPerLine, LinesPerFrame;
//    DLPC34XX_ReadInputImageSize(&PixelsPerLine, &LinesPerFrame);
//    return true;
//}
//
// //初始化相机
//bool ProjectionControlThread::initializeCameras()
//{
//    // 初始化相机工厂
//    IGXFactory::GetInstance().Init();
//
//    // 枚举设备
//    gxdeviceinfo_vector vectorDeviceInfo;
//    IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);
//    if (vectorDeviceInfo.size() < 2)
//    {
//        emit logMessage("Fewer than two devices found, cannot proceed!");
//        IGXFactory::GetInstance().Uninit();
//        return false;
//    }
//    else
//    {
//        emit logMessage(QString("Found %1 camera devices").arg(vectorDeviceInfo.size()));
//    }
//
//    // 根据相机序列号判断左右
//    if (vectorDeviceInfo[0].GetSN() < vectorDeviceInfo[1].GetSN()) 
//    {
//        std::swap(vectorDeviceInfo[0], vectorDeviceInfo[1]); // 交换使得相机1为左边
//    }
//
//    // 打开第一个相机及其第一个流
//    ObjDevicePtr1 = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
//
//    if (ObjDevicePtr1 == nullptr) // 使用显式比较
//    {
//        emit logMessage("Unable to open Camera 1");
//        return false;
//    }
//
//    ObjStreamPtr1 = ObjDevicePtr1->OpenStream(0);
//    if (ObjStreamPtr1 == nullptr) // 使用显式比较
//    {
//        emit logMessage("Unable to open Camera 1 stream");
//        return false;
//    }
//
//    // 打开第二个相机及其第一个流
//    CGXDevicePointer ObjDevicePtr2 = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[1].GetSN(), GX_ACCESS_EXCLUSIVE);
//
//    if (ObjDevicePtr2 == nullptr) // 使用显式比较
//    {
//        emit logMessage("Unable to open Camera 2");
//        return false;
//    }
//
//    ObjStreamPtr2 = ObjDevicePtr2->OpenStream(0);
//    if (ObjStreamPtr2 == nullptr) // 使用显式比较
//    {
//        emit logMessage("Unable to open Camera 2 stream");
//        return false;
//    }
//
//    // 获取第一个相机的远端设备属性控制器
//    ObjFeatureControlPtr1 = ObjDevicePtr1->GetRemoteFeatureControl();
//    if (ObjFeatureControlPtr1 == nullptr) // 使用显式比较
//    {
//        emit logMessage("Unable to get Camera 1 feature control");
//        return false;
//    }
//
//    // 设置相机1的特征
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerMode")->SetValue("On");
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerSource")->SetValue("Line3");
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");
//    ObjFeatureControlPtr1->GetFloatFeature("ExposureTime")->SetValue(10000.0); // 10毫秒（确认单位）
//
//    // 获取第二个相机的远端设备属性控制器
//    ObjFeatureControlPtr2 = ObjDevicePtr2->GetRemoteFeatureControl();
//    if (ObjFeatureControlPtr2 == nullptr) // 使用显式比较
//    {
//        emit logMessage("Unable to get Camera 2 feature control");
//        return false;
//    }
//
//    // 设置相机2的特征
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerMode")->SetValue("On");
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerSource")->SetValue("Line3");
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");
//    ObjFeatureControlPtr2->GetFloatFeature("ExposureTime")->SetValue(10000.0); // 10毫秒（确认单位）
//
//    // 注册回调采集到第一个相机
//    pCaptureEventHandler1 = std::make_unique<CSampleCaptureEventHandler1>();
//    pCaptureEventHandler1->setSavePath(leftImageSavePath);
//    ObjStreamPtr1->RegisterCaptureCallback(pCaptureEventHandler1.get(), nullptr);
//
//    // 注册回调采集到第二个相机
//    pCaptureEventHandler2 = std::make_unique<CSampleCaptureEventHandler2>();
//    pCaptureEventHandler2->setSavePath(rightImageSavePath);
//    ObjStreamPtr2->RegisterCaptureCallback(pCaptureEventHandler2.get(), nullptr);
//
//    return true;
//}
//
// //启动采集
//void ProjectionControlThread::startAcquisition()
//{
//    // 发送开采命令到第一个相机
//    ObjStreamPtr1->StartGrab();
//    ObjFeatureControlPtr1->GetCommandFeature("AcquisitionStart")->Execute();
//
//    // 发送开采命令到第二个相机
//    ObjStreamPtr2->StartGrab();
//    ObjFeatureControlPtr2->GetCommandFeature("AcquisitionStart")->Execute();
//}
//
//// 停止采集
//void ProjectionControlThread::stopAcquisition()
//{
//    // 发送停采命令到第一个相机
//    ObjFeatureControlPtr1->GetCommandFeature("AcquisitionStop")->Execute();
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerMode")->SetValue("Off");
//    ObjStreamPtr1->StopGrab();
//    ObjStreamPtr1->UnregisterCaptureCallback();
//    ObjStreamPtr1->Close();
//    ObjDevicePtr1->Close();
//
//    // 发送停采命令到第二个相机
//    ObjFeatureControlPtr2->GetCommandFeature("AcquisitionStop")->Execute();
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerMode")->SetValue("Off");
//    ObjStreamPtr2->StopGrab();
//    ObjStreamPtr2->UnregisterCaptureCallback();
//    ObjStreamPtr2->Close();
//    ObjDevicePtr2->Close();
//}
//
//// 清理资源
//void ProjectionControlThread::cleanup()
//{
//    // 销毁事件回调指针（智能指针自动处理）
//    pCaptureEventHandler1.reset();
//    pCaptureEventHandler2.reset();
//
//    // 反初始化库
//    IGXFactory::GetInstance().Uninit();
//
//    // 释放 I2C 资源
//    CYPRESS_I2C_RelinquishI2CBusAccess();
//}
//
//
