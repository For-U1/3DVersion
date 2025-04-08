#include "CameraWidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QDebug>
#include <QDir>
#include <QFileDialog>


Camerawidget::Camerawidget(int num, QWidget* parent)//���캯��ʵ�� �� ʹ�ó�ʼ���б�ʵ�ֲ�����ʼ��
    : QWidget(parent),
    cameraThreadLeft_(nullptr),
    cameraThreadRight_(nullptr),
    //index(num),δʹ�� ��ʱע��
    savePath_("")//��ʼ������·��Ϊ��
{

    // ��ʼ�� CameraController ʵ��
    cameraController_ = new CameraController(this);

    // ������Ϊ��ֱ����
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // ���������ʾ���򲼾�
    QHBoxLayout* cameraLayout = new QHBoxLayout();

    // ���������
    leftCameraView_ = new QLabel("Left Camera View", this);
    leftCameraView_->setFixedSize(600, 600); // ������ʾ�ߴ�
    cameraLayout->addWidget(leftCameraView_);

    // ���������
    rightCameraView_ = new QLabel("Right Camera View", this);
    rightCameraView_->setFixedSize(600, 600); // ������ʾ�ߴ�
    cameraLayout->addWidget(rightCameraView_);

    // ��������ʾ���ֵ�������
    mainLayout->addLayout(cameraLayout);

    //�������ع������������
    QHBoxLayout* setexposureLayout = new QHBoxLayout();

    QLabel* exposureLabel = new QLabel("Exposure Time:", this);
    QDoubleSpinBox* exposureSpinBox_= new QDoubleSpinBox(this);
    exposureSpinBox_->setRange(1000.0, 1000000.0);  // �����ع�ʱ�䷶Χ
    exposureSpinBox_->setValue(90000.0);           // ����Ĭ��ֵ
    setexposureLayout->addWidget(exposureLabel);
    setexposureLayout->addWidget(exposureSpinBox_);
    mainLayout->addLayout(setexposureLayout);//����عⲼ�ֵ�������


    ////���������������������
    //QHBoxLayout* setGainLayout = new QHBoxLayout();
    //QLabel* gainLabel = new QLabel("Gain:", this);
    //gainSpinBox_ = new QDoubleSpinBox(this);
    //gainSpinBox_->setRange(0.0, 12.0);              // �������淶Χ
    //gainSpinBox_->setValue(0.0);                    // ����Ĭ��ֵ
    //setGainLayout->addWidget(gainLabel);
    //setGainLayout->addWidget(gainSpinBox_);
    //mainLayout->addLayout(setexposureLayout);//������沼�ֵ�������

    // ����Щ�ؼ���ӵ���������
    /*mainLayout->addWidget(exposureLabel);
    mainLayout->addWidget(exposureSpinBox_);
    mainLayout->addWidget(gainLabel);
    mainLayout->addWidget(gainSpinBox_);*/

    // ������ť����
    QHBoxLayout* buttonLayout = new QHBoxLayout();

    QPushButton* openCameraButton = new QPushButton("Open Cameras", this);
    buttonLayout->addWidget(openCameraButton);

    QPushButton* captureImageButton = new QPushButton("Capture Images", this);
    buttonLayout->addWidget(captureImageButton);

    QPushButton* closeCameraButton = new QPushButton("Close Cameras", this);
    buttonLayout->addWidget(closeCameraButton);

    // ��Ӱ�ť���ֵ�������
    mainLayout->addLayout(buttonLayout);

    setLayout(mainLayout);

    // �����źźͲ�
    connect(openCameraButton, &QPushButton::clicked, this, &Camerawidget::openCamera);
    connect(captureImageButton, &QPushButton::clicked, this, &Camerawidget::captureImage);
    connect(closeCameraButton, &QPushButton::clicked, this, &Camerawidget::closeCamera);

    // ������������ؼ�����Ӧ�Ĳۺ���
    connect(exposureSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &Camerawidget::onExposureChanged);
    //connect(gainSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &Camerawidget::onGainChanged);//���������������

}

Camerawidget::~Camerawidget()//����������ʵ��
{
    closeCamera(); // ȷ�����������ǰ�ѹر�
}

void Camerawidget::openCamera()
{

    qDebug() << "Listing devices...";

    // �о��豸
    if (!cameraController_->listDevices()) 
    {
        qCritical() << "Failed to list devices. Aborting camera opening.";
        return;
    }

    // ��������߳�
    if (!cameraThreadLeft_)
    {
        cameraThreadLeft_ = new CameraThread(1, "Left Camera", this);
        connect(cameraThreadLeft_, &CameraThread::imageCaptured, this, &Camerawidget::onImageCaptured);
        cameraThreadLeft_->start();
    }

    // ��������߳�
    if (!cameraThreadRight_)
    {
        cameraThreadRight_ = new CameraThread(2, "Right Camera", this);
        connect(cameraThreadRight_, &CameraThread::imageCaptured, this, &Camerawidget::onImageCaptured);
        cameraThreadRight_->start();
    }
}

// void testwidget::captureImage()
// {
//     QString savePath = QDir::currentPath() + "/images";//ͼƬ�����ڵ�ǰ��Ŀ·����

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
    // ���·��Ϊ�գ��򵯳��Ի������û�ѡ��·��
    if (savePath_.isEmpty()) 
    {
        QString selectedPath = QFileDialog::getExistingDirectory(this, tr("Select Save Directory"), QDir::currentPath());

        // ����û�ȡ��ѡ�񣬷���
        if (selectedPath.isEmpty()) 
        {
            qDebug() << "No directory selected. Aborting image capture.";
            return;
        }

        // ����ѡ���·��
        savePath_ = selectedPath;
    }

    // ʹ��ѡ���·������ͼƬ����
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
        cameraThreadLeft_ = nullptr;//�Ϳղ��� �� ��ֹҰָ�����
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
        // ��ʾ�����ͼ��
        leftCameraView_->setPixmap(QPixmap::fromImage(image).scaled(leftCameraView_->size(), Qt::KeepAspectRatio));
    }
    else if (cameraIndex == 2)
    {
        // ��ʾ�����ͼ��
        rightCameraView_->setPixmap(QPixmap::fromImage(image).scaled(rightCameraView_->size(), Qt::KeepAspectRatio));
    }

}
void Camerawidget::onExposureChanged(double exposure)
{
    // ����������ع�ʱ��
    if (cameraThreadLeft_)
    {
        cameraThreadLeft_->setExposureTime(exposure);
    }
    if (cameraThreadRight_)
    {
        cameraThreadRight_->setExposureTime(exposure);
    }
}

//void Camerawidget::onGainChanged(double gain)//��ʱ������������Ĺ���
//{
//    // �������������
//    cameraController_->setGain(gain);
//}
