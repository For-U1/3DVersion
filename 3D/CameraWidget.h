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
        explicit Camerawidget(int num, QWidget* parent = nullptr);//���캯��
        ~Camerawidget();//��������

    private slots: //�ۺ���
        void openCamera();
        void captureImage();
        void closeCamera();
        void onImageCaptured(int cameraIndex, const QImage& image);
        void onExposureChanged(double exposure);  // �ۺ������ع�ʱ������
        //void onGainChanged(double gain);          // �ۺ�������������

    private:
        CameraThread* cameraThreadLeft_;//������߳�
        CameraThread* cameraThreadRight_;//������߳�
        QLabel* leftCameraView_;  // �����������ʾ
        QLabel* rightCameraView_; // �����������ʾ
        //int index;//������,δʹ�ã���ʱע��
        CameraController* cameraController_; // ��� CameraController ��Ա����
        QString savePath_;//ͼƬ����·��
        QDoubleSpinBox* exposureSpinBox_;  // ����ع�ʱ��ؼ�
        //QDoubleSpinBox* gainSpinBox_;      // ����ؼ�,��ʱ����
    };

