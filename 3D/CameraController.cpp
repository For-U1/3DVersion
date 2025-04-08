#include "CameraController.h"
#include <QDebug>
#include <QImage>

CameraController::CameraController(QObject* parent)
    : QObject(parent)
{
    GX_STATUS emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to initialize library, Error Code:" << emStatus;
    }
}

CameraController::~CameraController()
{
    releaseCamera();
    GXCloseLib();
    qDebug() << "Camera controller destroyed.";
}

bool CameraController::initializeCamera(uint32_t index, const QString& windowName)
{
    GX_STATUS emStatus;
    Camera& camera = camera_;
    GX_OPEN_PARAM openParam;
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;
    openParam.openMode = GX_OPEN_INDEX;

    // �����������
    char pszContent[10];  // ���ӻ�������С�����ɸ����ַ�
    snprintf(pszContent, sizeof(pszContent), "%u", index);
    openParam.pszContent = pszContent;

    // ������豸
    emStatus = GXOpenDevice(&openParam, &camera.handle);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to open device with index" << index << ", Error Code:" << emStatus;
        return false;
    }
    qDebug() << "Successfully opened device with index" << index;

    //�����������
    //emStatus = GXSetFloatValue(camera.handle, "ExposureTime", 100000.0000);
    //if (emStatus != GX_STATUS_SUCCESS)
    //{
    //    qCritical() << "Failed to set ExposureTime, Error Code:" << emStatus;
    //    GXCloseDevice(camera.handle);  // �ر�����豸
    //    return false;
    //}

    emStatus = GXSetBoolValue(camera.handle, "GammaEnable", true);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to enable Gamma, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    //emStatus = GXSetFloatValue(camera.handle, "Gain", 0.0000);
    //if (emStatus != GX_STATUS_SUCCESS)
    //{
    //    qCritical() << "Failed to set Gain, Error Code:" << emStatus;
    //    GXCloseDevice(camera.handle);  // �ر�����豸
    //    return false;
    //}

    emStatus = GXSetEnumValueByString(camera.handle, "TriggerMode", "Off");
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to set TriggerMode, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    emStatus = GXSetEnumValueByString(camera.handle, "TriggerSource", "Line0");
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to set TriggerSource, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    emStatus = GXSetEnumValueByString(camera.handle, "LineSelector", "Line0");
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to set LineSelector, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    // ��ȡ���������Ϣ
    emStatus = GXGetInt(camera.handle, GX_INT_PAYLOAD_SIZE, &camera.payloadSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to get Payload Size, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    emStatus = GXGetInt(camera.handle, GX_INT_WIDTH, &camera.imageWidth);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to get Image Width, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    emStatus = GXGetInt(camera.handle, GX_INT_HEIGHT, &camera.imageHeight);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to get Image Height, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    // �������ͼ�񻺴�
    camera.frame.create(camera.imageHeight, camera.imageWidth, CV_8UC3);

    bool bColorFilter;
    emStatus = GXIsImplemented(camera.handle, GX_ENUM_PIXEL_COLOR_FILTER, &bColorFilter);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to check Color Filter Implementation, Error Code:" << emStatus;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    if (bColorFilter)
    {
        emStatus = GXGetEnum(camera.handle, GX_ENUM_PIXEL_COLOR_FILTER, &camera.pixelColorFilter);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            qCritical() << "Failed to get Pixel Color Filter, Error Code:" << emStatus;
            GXCloseDevice(camera.handle);  // �ر�����豸
            return false;
        }
    }

    // �����ڴ�
    camera.bufferRGB = new BYTE[(size_t)(camera.imageWidth * camera.imageHeight * 3)];
    if (camera.bufferRGB == nullptr)
    {
        qCritical() << "Failed to allocate memory for RGB buffer";
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    camera.bufferRaw = new BYTE[(size_t)camera.payloadSize];
    if (camera.bufferRaw == nullptr)
    {
        qCritical() << "Failed to allocate memory for raw buffer";
        delete[] camera.bufferRGB;
        camera.bufferRGB = nullptr;
        GXCloseDevice(camera.handle);  // �ر�����豸
        return false;
    }

    camera.windowName = windowName;

    qDebug() << "Camera initialization completed successfully for index" << index;
    return true;
}

// �����ع�ʱ��ĺ���
bool CameraController::setExposureTime(double exposureTime)
{
    QMutexLocker locker(&mutex_); // ����������
    
    if (camera_.handle == nullptr)
    {
        qCritical() << "Camera handle is null. Cannot set exposure time.";
        return false;
    }

    GX_STATUS emStatus = GXSetFloatValue(camera_.handle, "ExposureTime", exposureTime);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to set ExposureTime, Error Code:" << emStatus;
        return false;
    }

        qDebug() << "Exposure time set to" << exposureTime;
    return true;
    /*double actualExposureTime;
    GX_STATUS emStatusCheck = GXGetFloatValue(camera_.handle, "ExposureTime", &actualExposureTime);
    if (emStatusCheck == GX_STATUS_SUCCESS && actualExposureTime == exposureTime)
    {
        qDebug() << "Verified exposure time set to" << actualExposureTime;
    }
    else
    {
        qCritical() << "Failed to verify ExposureTime, set value:" << exposureTime << ", actual value:" << actualExposureTime;
    }*/
}

// ��������ĺ������Ժ�����Ӹù���
bool CameraController::setGain(double gain)
{
    if (camera_.handle == nullptr)
    {
        qCritical() << "Camera handle is null. Cannot set gain.";
        return false;
    }

    GX_STATUS emStatus = GXSetFloatValue(camera_.handle, "Gain", gain);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to set Gain, Error Code:" << emStatus;
        return false;
    }

    qDebug() << "Gain set to" << gain;
    return true;
}


void CameraController::releaseCamera()
{

    QMutexLocker locker(&mutex_); // ����������
    
    Camera& camera = camera_;
    if (camera.handle != nullptr)
    {
        qDebug() << "Stopping acquisition for camera";
        GXSendCommand(camera.handle, GX_COMMAND_ACQUISITION_STOP);
        qDebug() << "Unregistering callback for camera";
        GXUnregisterCaptureCallback(camera.handle);

        if (camera.bufferRGB != nullptr)
        {
            delete[] camera.bufferRGB;
            camera.bufferRGB = nullptr;
        }
        if (camera.bufferRaw != nullptr)
        {
            delete[] camera.bufferRaw;
            camera.bufferRaw = nullptr;
        }
        qDebug() << "Closing device for camera";
        GXCloseDevice(camera.handle);
        camera.handle = nullptr;
    }
}

bool CameraController::captureImage(cv::Mat& image)
{
    QMutexLocker locker(&mutex_);

    if (camera_.handle == nullptr)
    {
        qCritical() << "Camera handle is null. Cannot capture image.";
        return false;
    }

    if (camera_.bufferRaw == nullptr || camera_.bufferRGB == nullptr)
    {
        qCritical() << "Buffers are null. Cannot capture image.";
        return false;
    }

    Camera& camera = camera_;
    GX_FRAME_DATA frameData;
    memset(&frameData, 0, sizeof(GX_FRAME_DATA));
    frameData.pImgBuf = camera.bufferRaw;
    frameData.nImgSize = camera.payloadSize;

    GX_STATUS emStatus = GXGetImage(camera.handle, &frameData, 1000);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to capture image, Error Code:" << emStatus;
        return false;
    }

    DxRaw8toRGB24(camera.bufferRaw, camera.bufferRGB, (VxUint32)camera.imageWidth, (VxUint32)camera.imageHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(camera.pixelColorFilter), false);
    memcpy(camera.frame.data, camera.bufferRGB, camera.imageWidth * camera.imageHeight * 3);
    image = camera.frame.clone();

    return true;
}

bool CameraController::startAcquisition()
{
    QMutexLocker locker(&mutex_);

    if (camera_.handle == nullptr)
    {
        qCritical() << "Camera handle is null. Cannot start acquisition.";
        return false;
    }

    GX_STATUS emStatus = GXSendCommand(camera_.handle, GX_COMMAND_ACQUISITION_START);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to start acquisition, Error Code:" << emStatus;
    }
    return emStatus == GX_STATUS_SUCCESS;
}

bool CameraController::stopAcquisition()
{
    QMutexLocker locker(&mutex_);

    if (camera_.handle == nullptr)
    {
        qCritical() << "Camera handle is null. Cannot stop acquisition.";
        return false;
    }

    GX_STATUS emStatus = GXSendCommand(camera_.handle, GX_COMMAND_ACQUISITION_STOP);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to stop acquisition, Error Code:" << emStatus;
    }
    return emStatus == GX_STATUS_SUCCESS;
}

bool CameraController::listDevices()
{
    GX_STATUS emStatus;
    uint32_t nDeviceNum = 0;
    emStatus = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((emStatus != GX_STATUS_SUCCESS) || (nDeviceNum < 2))
    {
        qCritical() << "At least two devices are required, Error Code:" << emStatus;
        return false;
    }

    qDebug() << "Number of devices found:" << nDeviceNum;


    // ��ȡ�豸��Ϣ�б�
    GX_DEVICE_BASE_INFO* pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
    size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
    emStatus = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);

    if (emStatus != GX_STATUS_SUCCESS)
    {
        qCritical() << "Failed to get device information, Error Code:" << emStatus;
        delete[] pBaseinfo;
        return false;
    }

    for (uint32_t i = 0; i < nDeviceNum; ++i)
    {
        qDebug() << "Device" << i + 1 << "SN:" << pBaseinfo[i].szSN;
    }

    delete[] pBaseinfo;
    return true;
}




