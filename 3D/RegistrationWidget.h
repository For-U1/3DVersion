#ifndef REGISTRATIONWIDGET_H
#define REGISTRATIONWIDGET_H

#include <QWidget>
#include <QProgressBar>
#include <QComboBox>
#include <QProgressBar>
#include <QTimer>
#include <QTextEdit>
#include <QDateTime>
#include <QFileDialog>
#include "PointCloudProcessing.h"  // ���ƴ����߳����
#include "PCLViewer.h"           // ������ʾ�ؼ�
#include "AdvancedSettingsDialog.h"//RANSAC·��������

class RegistrationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RegistrationWidget(QWidget* parent = nullptr);
    ~RegistrationWidget();

private slots:
    void selectInputFolder();//�����ļ�·������

    void selectOutputFolder();//����ļ�·������

    void importPointCloudFile();  // ��������ļ�����ʾ���ƺ���

    void startProcessing();//��ʼƴ�Ӻ���

    void onAdvancedSettings();//�߼����òۺ���

    void updateProgress(int value);//����ƴ�ӽ��Ⱥ���

    void displayLogMessage(const QString& message);//��ʾ��־��Ϣ

    void displayFinalPointCloud(pcl::PointCloud<ViewerPointT>::Ptr cloud);//��ʾƴ����ɺ�ĵ��ƺ���

    void onMethodChanged(int index); //һ��ʼ��ƴ�ӷ���
    
    void processingFinished();//��ɴ�����

private:
    // ·����ؿؼ�
    QLineEdit* inputFolderLineEdit;
    QLineEdit* outputFolderLineEdit;
    QPushButton* selectInputButton;
    QPushButton* selectOutputButton;

    // ���ܰ�ť�ͽ�����
    QPushButton* importButton;
    QPushButton* startButton;
    QProgressBar* progressBar;
    QPushButton* advancedButton;//"�߼����ð�ť"
    QComboBox* methodSelector;//����ѡ��ƴ�ӷ���
    int currentMethod;  // 0: ICP, 1: RANSAC

    // �߼����ò��������Ը�����Ҫ��չ���������
    QString calibPath;
    QString imagePath;
    QString cloudPointdataPath;

    // ��־����͵�����ʾ
    QTextEdit* logTextEdit;
    PCLViewer* viewer;

    // ���ƴ����̣߳�����ƴ�ӣ�
    PointCloudProcessingThread* processingThread;
};

#endif // REGISTRATIONWIDGET_H
