//#ifndef PHASEWIDGET_H
//#define PHASEWIDGET_H
//
//#include <QPushButton>
//#include <QVBoxLayout>
//#include <memory>
//#include "PhaseProcessorThread.h"
////#include "SettingsDialog.h"
//#include "Config.h"
//
//class PhaseWidget : public QWidget
//{
//    Q_OBJECT
//
//public:
//    explicit PhaseWidget(QWidget* parent = nullptr);
//    ~PhaseWidget();
//
//public slots:
//    void onStart();
//    void onStop();
//    //void onSettings();
//
//private slots:
//    void onProcessingStarted();
//    void onProcessingStopped();
//
//private:
//    QPushButton* startButton;
//    QPushButton* stopButton;
//    //QPushButton* settingsButton;
//    PhaseProcessorThread* phaseProcessorThread;
//    std::shared_ptr<Config> config;
//};
//
//#endif // PHASEWIDGET_H

//#ifndef PHASEWIDGET_H
//#define PHASEWIDGET_H
//
//#include <QPushButton>
//#include <QHBoxLayout>
//#include <QFormLayout>
//#include <QLineEdit>
//#include <QDoubleSpinBox>
//#include <QSpinBox>
//#include <QCheckBox>
//#include <QProgressBar>
//#include <memory>
//#include "PhaseProcessorThread.h"
//#include "Config.h"
//
//class PhaseWidget : public QWidget
//{
//    Q_OBJECT
//
//public:
//    explicit PhaseWidget(QWidget* parent = nullptr);
//    ~PhaseWidget();
//
//public slots:
//    void onStart();
//    void onStop();
//
//private slots:
//    void onProcessingStarted();
//    void onProcessingStopped();
//
//private:
//    // �ؼ�
//    QPushButton* m_startButton;
//    QPushButton* m_stopButton;
//    QPushButton* m_resetButton;
//
//    // ���ò�������ؼ�
//    QLineEdit* m_sepLineEdit;
//    QLineEdit* m_rootDirLineEdit;
//    QLineEdit* m_dataDirLineEdit;
//    QLineEdit* m_calibDirLineEdit;
//    QLineEdit* m_calibFileLineEdit;
//    QPushButton* m_calibBrowseButton;
//
//    QLineEdit* m_projectDirLineEdit;
//    QPushButton* m_projectDirBrowseButton;
//
//    QLineEdit* m_simuDirLineEdit;
//    QPushButton* m_simuDirBrowseButton;
//
//    QLineEdit* m_modelDirLineEdit;
//    QPushButton* m_modelDirBrowseButton;
//
//    QLineEdit* m_outputDirLineEdit;
//    QPushButton* m_outputDirBrowseButton;
//
//    QLineEdit* m_outputDirLLineEdit;
//    QPushButton* m_outputDirLBrowseButton;
//
//    QLineEdit* m_outputDirRLineEdit;
//    QPushButton* m_outputDirRBrowseButton;
//
//    QLineEdit* m_saveFilePoint3DLineEdit;
//    QPushButton* m_saveFilePoint3DBrowseButton;
//
//    QDoubleSpinBox* m_ASpinBox;
//    QDoubleSpinBox* m_BSpinBox;
//    QSpinBox* m_NSpinBox;
//    QSpinBox* m_TSpinBox;
//    QDoubleSpinBox* m_T1SpinBox;
//    QDoubleSpinBox* m_T2SpinBox;
//    QDoubleSpinBox* m_T3SpinBox;
//    QSpinBox* m_WSpinBox;
//    QSpinBox* m_HSpinBox;
//    QDoubleSpinBox* m_BMinSpinBox;
//
//    QSpinBox* m_winSpinBox;
//    QDoubleSpinBox* m_pdSpinBox;
//    QDoubleSpinBox* m_minZSpinBox;
//    QDoubleSpinBox* m_maxZSpinBox;
//
//    QCheckBox* m_writeCheckBox;
//    QCheckBox* m_showCheckBox;
//
//    QProgressBar* m_progressBar;
//
//    // ���úʹ����߳�
//    std::shared_ptr<Config> m_config;
//    std::unique_ptr<PhaseProcessorThread> m_phaseProcessorThread;
//
//    // ����
//    QFormLayout* m_formLayout;
//    QHBoxLayout* m_buttonLayout;
//    QVBoxLayout* m_mainLayout;
//
//    // ��������
//    void setupUI();
//    void connectSignals();
//};
//
//#endif // PHASEWIDGET_H
#ifndef PHASEWIDGET_H
#define PHASEWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>
#include <memory>
#include "PhaseProcessorThread.h"
#include "Config.h"

class PhaseWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PhaseWidget(QWidget* parent = nullptr);
    ~PhaseWidget();

public slots:
    void onStart();
    void onStop();

private slots:
    void onProcessingStarted();
    void onProcessingStopped();
    void onProcessingProgress(int value);

private:
    void createSettingsLayout(); // �������ÿؼ��Ĳ���
    void loadConfigToUI();       // �����ü��ص�����
    void saveUIToConfig();       // �������ϵ�ֵ���浽����

    QPushButton* startButton;
    QPushButton* stopButton;
    QProgressBar* progressBar;

    // ���ò����Ŀؼ�
    QLineEdit* rootDirLineEdit;
    QLineEdit* dataDirLineEdit;
    QLineEdit* calibDirLineEdit;
    QLineEdit* calibFileLineEdit;
    QLineEdit* projectDirLineEdit;
    QLineEdit* simuDirLineEdit;
    QLineEdit* modelDirLineEdit;
    QLineEdit* outputDirLineEdit;
    QLineEdit* outputDirLLineEdit; 
    QLineEdit* outputDirRLineEdit;
    QLineEdit* saveFilePoint3dLineEdit;

    QPushButton* browseRootDirButton;
    QPushButton* browseDataDirButton;
    QPushButton* browseCalibDirButton;
    QPushButton* browseCalibFileButton;
    QPushButton* browseProjectDirButton;
    QPushButton* browseSimuDirButton;
    QPushButton* browseModelDirButton;
    QPushButton* browseOutputDirButton;
    QPushButton* browseOutputDirLButton; 
    QPushButton* browseOutputDirRButton;
    QPushButton* browseSaveFilePoint3dButton;

    QCheckBox* writeCheckBox;
    QCheckBox* showCheckBox;

    QLineEdit* ALineEdit;
    QLineEdit* BLineEdit;
    QLineEdit* NLineEdit;
    QLineEdit* TLineEdit;
    QLineEdit* T1LineEdit;
    QLineEdit* T2LineEdit;
    QLineEdit* T3LineEdit;
    QLineEdit* WLineEdit;
    QLineEdit* HLineEdit;
    QLineEdit* BMinLineEdit;

    QLineEdit* winLineEdit;
    QLineEdit* pdLineEdit;
    QLineEdit* minZLineEdit;
    QLineEdit* maxZLineEdit;

    PhaseProcessorThread* phaseProcessorThread;
    std::shared_ptr<Config> config;
};

#endif // PHASEWIDGET_H
