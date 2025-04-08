//#pragma once
//#include <QWidget>
//#include <QLabel>
//#include <QPushButton>
//#include <QVBoxLayout>
//#include <QProgressBar>
//#include <QHBoxLayout>
//#include "Calibratethread.h"
//
//class CalibWidget : public QWidget
//{
//    Q_OBJECT
//
//public:
//    explicit CalibWidget(QWidget* parent = nullptr);
//    ~CalibWidget();
//
//private slots:
//    void startCalibration();
//    void stopCalibration();
//    void displayCalibrationResults(const Mat& R, const Mat& T);  // ������ʾ�궨����Ĳۺ���
//    void updateProgress(int progress);
//
//    
//    //void updateImages(bool success);//������ʾ�궨������Ժ��ٵ��ԡ�
//
//private:
//    //QLabel* leftImageLabel;
//    //QLabel* rightImageLabel;
//    QPushButton* startButton;
//    QPushButton* stopButton;
//    QProgressBar* progressBar;  // �����������ؼ�
//    QLabel* rotationTranslationLabel;  // ������ʾ��תƽ�ƾ���ı�ǩ
//    CalibrateThread* calibrateThread;
//    QString calibDirPath;
//    QString savePath;
//
//
//    
//};

// calibwidget.h

#ifndef CALIBWIDGET_H
#define CALIBWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QProgressBar>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QMutex>
#include "calibratethread.h"

class CalibWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CalibWidget(QWidget* parent = nullptr);
    ~CalibWidget();

private slots:
    void startCalibration();
    void stopCalibration();
    void updateProgressBar(int value);
    void displayCalibrationResults(const QVector<double>& R_data, int R_rows, int R_cols,
        const QVector<double>& T_data, int T_rows, int T_cols);
    void calibrationFinished(bool success);

private:
    QPushButton* startButton;
    QPushButton* stopButton;
    QProgressBar* progressBar;
    QLabel* rotationTranslationLabel;
    CalibrateThread* calibrateThread;

    QSpinBox* rowsSpinBox;
    QSpinBox* colsSpinBox;
    QDoubleSpinBox* squareSizeSpinBox;

    QMutex mutex_;
};

#endif // CALIBWIDGET_H
