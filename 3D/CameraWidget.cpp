#include "CameraWidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QDebug>
#include <QDir>
#include <QFileDialog>


Camerawidget::Camerawidget(int num, QWidget* parent)//构造函数实现 ， 使用初始化列表实现参数初始化
    : QWidget(parent),
    cameraThreadLeft_(nullptr),
    cameraThreadRight_(nullptr),
    //index(num),未使用 暂时注销
    savePath_("")//初始化保存路径为空
{

    // 初始化 CameraController 实例
    cameraController_ = new CameraController(this);

    // 主布局为垂直布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 创建相机显示区域布局
    QHBoxLayout* cameraLayout = new QHBoxLayout();

    // 左相机画面
    leftCameraView_ = new QLabel("Left Camera View", this);
    leftCameraView_->setFixedSize(600, 600); // 设置显示尺寸
    cameraLayout->addWidget(leftCameraView_);

    // 右相机画面
    rightCameraView_ = new QLabel("Right Camera View", this);
    rightCameraView_->setFixedSize(600, 600); // 设置显示尺寸
    cameraLayout->addWidget(rightCameraView_);

    // 添加相机显示布局到主布局
    mainLayout->addLayout(cameraLayout);

    //添加相机曝光参数设置区域
    QHBoxLayout* setexposureLayout = new QHBoxLayout();

    QLabel* exposureLabel = new QLabel("Exposure Time:", this);
    QDoubleSpinBox* exposureSpinBox_= new QDoubleSpinBox(this);
    exposureSpinBox_->setRange(1000.0, 1000000.0);  // 设置曝光时间范围
    exposureSpinBox_->setValue(90000.0);           // 设置默认值
    setexposureLayout->addWidget(exposureLabel);
    setexposureLayout->addWidget(exposureSpinBox_);
    mainLayout->addLayout(setexposureLayout);//添加曝光布局到主布局


    ////添加相机增益参数设置区域
    //QHBoxLayout* setGainLayout = new QHBoxLayout();
    //QLabel* gainLabel = new QLabel("Gain:", this);
    //gainSpinBox_ = new QDoubleSpinBox(this);
    //gainSpinBox_->setRange(0.0, 12.0);              // 设置增益范围
    //gainSpinBox_->setValue(0.0);                    // 设置默认值
    //setGainLayout->addWidget(gainLabel);
    //setGainLayout->addWidget(gainSpinBox_);
    //mainLayout->addLayout(setexposureLayout);//添加增益布局到主布局

    // 将这些控件添加到主布局中
    /*mainLayout->addWidget(exposureLabel);
    mainLayout->addWidget(exposureSpinBox_);
    mainLayout->addWidget(gainLabel);
    mainLayout->addWidget(gainSpinBox_);*/

    // 创建按钮布局
    QHBoxLayout* buttonLayout = new QHBoxLayout();

    QPushButton* openCameraButton = new QPushButton("Open Cameras", this);
    buttonLayout->addWidget(openCameraButton);

    QPushButton* captureImageButton = new QPushButton("Capture Images", this);
    buttonLayout->addWidget(captureImageButton);

    QPushButton* closeCameraButton = new QPushButton("Close Cameras", this);
    buttonLayout->addWidget(closeCameraButton);

    // 添加按钮布局到主布局
    mainLayout->addLayout(buttonLayout);

    setLayout(mainLayout);

    // 连接信号和槽
    connect(openCameraButton, &QPushButton::clicked, this, &Camerawidget::openCamera);
    connect(captureImageButton, &QPushButton::clicked, this, &Camerawidget::captureImage);
    connect(closeCameraButton, &QPushButton::clicked, this, &Camerawidget::closeCamera);

    // 连接相机参数控件和相应的槽函数
    connect(exposureSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &Camerawidget::onExposureChanged);
    //connect(gainSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &Camerawidget::onGainChanged);//控制相机增益连接

}

Camerawidget::~Camerawidget()//析构函数的实现
{
    closeCamera(); // 确保相机在销毁前已关闭
}

void Camerawidget::openCamera()
{

    qDebug() << "Listing devices...";

    // 列举设备
    if (!cameraController_->listDevices()) 
    {
        qCritical() << "Failed to list devices. Aborting camera opening.";
        return;
    }

    // 打开左相机线程
    if (!cameraThreadLeft_)
    {
        cameraThreadLeft_ = new CameraThread(1, "Left Camera", this);
        connect(cameraThreadLeft_, &CameraThread::imageCaptured, this, &Camerawidget::onImageCaptured);
        cameraThreadLeft_->start();
    }

    // 打开右相机线程
    if (!cameraThreadRight_)
    {
        cameraThreadRight_ = new CameraThread(2, "Right Camera", this);
        connect(cameraThreadRight_, &CameraThread::imageCaptured, this, &Camerawidget::onImageCaptured);
        cameraThreadRight_->start();
    }
}

// void testwidget::captureImage()
// {
//     QString savePath = QDir::currentPath() + "/images";//图片保存在当前项目路径；

//     if (cameraThreadLeft_)
//     {
//         cameraThreadLeft_->captureAndSave(savePath);
//     }
//     if (cameraThreadRight_)
//     {
//         cameraThreadRight_->captureAndSave(savePath);
//     }
//

void Camerawidget::captureImage()
{
    // 如果路径为空，则弹出对话框让用户选择路径
    if (savePath_.isEmpty()) 
    {
        QString selectedPath = QFileDialog::getExistingDirectory(this, tr("Select Save Directory"), QDir::currentPath());

        // 如果用户取消选择，返回
        if (selectedPath.isEmpty()) 
        {
            qDebug() << "No directory selected. Aborting image capture.";
            return;
        }

        // 保存选择的路径
        savePath_ = selectedPath;
    }

    // 使用选择的路径进行图片保存
    if (cameraThreadLeft_) 
    {
        cameraThreadLeft_->captureAndSave(savePath_);
    }
    if (cameraThreadRight_) 
    {
        cameraThreadRight_->captureAndSave(savePath_);
    }
}

void Camerawidget::closeCamera()
{
    if (cameraThreadLeft_)
    {
        cameraThreadLeft_->stop();
        cameraThreadLeft_->deleteLater();
        cameraThreadLeft_ = nullptr;//滞空操作 ， 防止野指针出现
    }

    if (cameraThreadRight_)
    {
        cameraThreadRight_->stop();
        cameraThreadRight_->deleteLater();
        cameraThreadRight_ = nullptr;
    }
}

void Camerawidget::onImageCaptured(int cameraIndex, const QImage& image)
{
    if (cameraIndex == 1)
    {
        // 显示左相机图像
        leftCameraView_->setPixmap(QPixmap::fromImage(image).scaled(leftCameraView_->size(), Qt::KeepAspectRatio));
    }
    else if (cameraIndex == 2)
    {
        // 显示右相机图像
        rightCameraView_->setPixmap(QPixmap::fromImage(image).scaled(rightCameraView_->size(), Qt::KeepAspectRatio));
    }

}
void Camerawidget::onExposureChanged(double exposure)
{
    // 更新相机的曝光时间
    if (cameraThreadLeft_)
    {
        cameraThreadLeft_->setExposureTime(exposure);
    }
    if (cameraThreadRight_)
    {
        cameraThreadRight_->setExposureTime(exposure);
    }
}

//void Camerawidget::onGainChanged(double gain)//暂时不用设置增益的功能
//{
//    // 更新相机的增益
//    cameraController_->setGain(gain);
//}
