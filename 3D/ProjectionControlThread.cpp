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
//// ���캯�������ܱ���·��
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
//// ��������
//ProjectionControlThread::~ProjectionControlThread()
//{
//    stop();
//    cleanup();
//}
//
//// ���п����߳�
//void ProjectionControlThread::run()
//{
//    try
//    {
//        
//        // ��ʼ��ͶӰ��
//        if (!initializeProjector()) 
//        {
//            emit logMessage("Projector initialization failed, terminating operation.");
//            return;
//        }
//
//        // ��ʼ�����
//        if (!initializeCameras()) 
//        {
//            emit logMessage("Camera initialization failed, terminating operation.");
//            cleanup();
//            return;
//        }
//
//        // ����ͼ��ɼ���ͶӰ
//        startAcquisition();
//        Sleep(100);
//        emit logMessage("Projection started");
//        // ���ò���ģʽ
//        /*DLPC34XX_WriteOperatingModeSelect(DLPC34XX_OM_SENS_INTERNAL_PATTERN);
//        DLPC34XX_WriteInternalPatternControl(DLPC34XX_PC_START, 0x0);*/
//        // ���ò���ģʽΪ�ڲ�ͼ��ģʽ
//        Sleep(100);
//        uint32_t ret = DLPC34XX_WriteOperatingModeSelect(DLPC34XX_OM_SENS_INTERNAL_PATTERN);
//        if (ret != SUCCESS)
//        {
//            emit logMessage(QString("Failed to set projector to internal pattern mode. Error code: %1").arg(ret));
//            cleanup();
//            return;
//        }
//
//        // �����ڲ�ͼ��ͶӰ
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
//         //�������У�ֱ�����յ�ֹͣ�ź�
//        while (running)
//        {
//#ifdef _WIN32
//            Sleep(100); // 100 ����
//#else
//            usleep(100 * 1000); // 100 ����
//#endif
//        }
//
//        // ֹͣ�ɼ���������Դ
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
//// ֹͣ�߳�
//void ProjectionControlThread::stop()
//{
//    if (running.exchange(false)) {
//        emit logMessage("Stop command issued");
//    }
//}
//
//// ��ʼ��ͶӰ��
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
// //��ʼ�����
//bool ProjectionControlThread::initializeCameras()
//{
//    // ��ʼ���������
//    IGXFactory::GetInstance().Init();
//
//    // ö���豸
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
//    // ����������к��ж�����
//    if (vectorDeviceInfo[0].GetSN() < vectorDeviceInfo[1].GetSN()) 
//    {
//        std::swap(vectorDeviceInfo[0], vectorDeviceInfo[1]); // ����ʹ�����1Ϊ���
//    }
//
//    // �򿪵�һ����������һ����
//    ObjDevicePtr1 = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
//
//    if (ObjDevicePtr1 == nullptr) // ʹ����ʽ�Ƚ�
//    {
//        emit logMessage("Unable to open Camera 1");
//        return false;
//    }
//
//    ObjStreamPtr1 = ObjDevicePtr1->OpenStream(0);
//    if (ObjStreamPtr1 == nullptr) // ʹ����ʽ�Ƚ�
//    {
//        emit logMessage("Unable to open Camera 1 stream");
//        return false;
//    }
//
//    // �򿪵ڶ�����������һ����
//    CGXDevicePointer ObjDevicePtr2 = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[1].GetSN(), GX_ACCESS_EXCLUSIVE);
//
//    if (ObjDevicePtr2 == nullptr) // ʹ����ʽ�Ƚ�
//    {
//        emit logMessage("Unable to open Camera 2");
//        return false;
//    }
//
//    ObjStreamPtr2 = ObjDevicePtr2->OpenStream(0);
//    if (ObjStreamPtr2 == nullptr) // ʹ����ʽ�Ƚ�
//    {
//        emit logMessage("Unable to open Camera 2 stream");
//        return false;
//    }
//
//    // ��ȡ��һ�������Զ���豸���Կ�����
//    ObjFeatureControlPtr1 = ObjDevicePtr1->GetRemoteFeatureControl();
//    if (ObjFeatureControlPtr1 == nullptr) // ʹ����ʽ�Ƚ�
//    {
//        emit logMessage("Unable to get Camera 1 feature control");
//        return false;
//    }
//
//    // �������1������
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerMode")->SetValue("On");
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerSource")->SetValue("Line3");
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");
//    ObjFeatureControlPtr1->GetFloatFeature("ExposureTime")->SetValue(10000.0); // 10���루ȷ�ϵ�λ��
//
//    // ��ȡ�ڶ��������Զ���豸���Կ�����
//    ObjFeatureControlPtr2 = ObjDevicePtr2->GetRemoteFeatureControl();
//    if (ObjFeatureControlPtr2 == nullptr) // ʹ����ʽ�Ƚ�
//    {
//        emit logMessage("Unable to get Camera 2 feature control");
//        return false;
//    }
//
//    // �������2������
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerMode")->SetValue("On");
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerSource")->SetValue("Line3");
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");
//    ObjFeatureControlPtr2->GetFloatFeature("ExposureTime")->SetValue(10000.0); // 10���루ȷ�ϵ�λ��
//
//    // ע��ص��ɼ�����һ�����
//    pCaptureEventHandler1 = std::make_unique<CSampleCaptureEventHandler1>();
//    pCaptureEventHandler1->setSavePath(leftImageSavePath);
//    ObjStreamPtr1->RegisterCaptureCallback(pCaptureEventHandler1.get(), nullptr);
//
//    // ע��ص��ɼ����ڶ������
//    pCaptureEventHandler2 = std::make_unique<CSampleCaptureEventHandler2>();
//    pCaptureEventHandler2->setSavePath(rightImageSavePath);
//    ObjStreamPtr2->RegisterCaptureCallback(pCaptureEventHandler2.get(), nullptr);
//
//    return true;
//}
//
// //�����ɼ�
//void ProjectionControlThread::startAcquisition()
//{
//    // ���Ϳ��������һ�����
//    ObjStreamPtr1->StartGrab();
//    ObjFeatureControlPtr1->GetCommandFeature("AcquisitionStart")->Execute();
//
//    // ���Ϳ�������ڶ������
//    ObjStreamPtr2->StartGrab();
//    ObjFeatureControlPtr2->GetCommandFeature("AcquisitionStart")->Execute();
//}
//
//// ֹͣ�ɼ�
//void ProjectionControlThread::stopAcquisition()
//{
//    // ����ͣ�������һ�����
//    ObjFeatureControlPtr1->GetCommandFeature("AcquisitionStop")->Execute();
//    ObjFeatureControlPtr1->GetEnumFeature("TriggerMode")->SetValue("Off");
//    ObjStreamPtr1->StopGrab();
//    ObjStreamPtr1->UnregisterCaptureCallback();
//    ObjStreamPtr1->Close();
//    ObjDevicePtr1->Close();
//
//    // ����ͣ������ڶ������
//    ObjFeatureControlPtr2->GetCommandFeature("AcquisitionStop")->Execute();
//    ObjFeatureControlPtr2->GetEnumFeature("TriggerMode")->SetValue("Off");
//    ObjStreamPtr2->StopGrab();
//    ObjStreamPtr2->UnregisterCaptureCallback();
//    ObjStreamPtr2->Close();
//    ObjDevicePtr2->Close();
//}
//
//// ������Դ
//void ProjectionControlThread::cleanup()
//{
//    // �����¼��ص�ָ�루����ָ���Զ�����
//    pCaptureEventHandler1.reset();
//    pCaptureEventHandler2.reset();
//
//    // ����ʼ����
//    IGXFactory::GetInstance().Uninit();
//
//    // �ͷ� I2C ��Դ
//    CYPRESS_I2C_RelinquishI2CBusAccess();
//}
//
//
