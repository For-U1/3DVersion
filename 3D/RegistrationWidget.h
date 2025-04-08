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
#include "PointCloudProcessing.h"  // 点云处理线程相关
#include "PCLViewer.h"           // 点云显示控件
#include "AdvancedSettingsDialog.h"//RANSAC路径管理类

class RegistrationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RegistrationWidget(QWidget* parent = nullptr);
    ~RegistrationWidget();

private slots:
    void selectInputFolder();//输入文件路径函数

    void selectOutputFolder();//输出文件路径函数

    void importPointCloudFile();  // 导入点云文件，显示点云函数

    void startProcessing();//开始拼接函数

    void onAdvancedSettings();//高级设置槽函数

    void updateProgress(int value);//更新拼接进度函数

    void displayLogMessage(const QString& message);//显示日志信息

    void displayFinalPointCloud(pcl::PointCloud<ViewerPointT>::Ptr cloud);//显示拼接完成后的点云函数

    void onMethodChanged(int index); //一开始的拼接方法
    
    void processingFinished();//完成处理函数

private:
    // 路径相关控件
    QLineEdit* inputFolderLineEdit;
    QLineEdit* outputFolderLineEdit;
    QPushButton* selectInputButton;
    QPushButton* selectOutputButton;

    // 功能按钮和进度条
    QPushButton* importButton;
    QPushButton* startButton;
    QProgressBar* progressBar;
    QPushButton* advancedButton;//"高级设置按钮"
    QComboBox* methodSelector;//用于选择拼接方法
    int currentMethod;  // 0: ICP, 1: RANSAC

    // 高级设置参数（可以根据需要扩展更多参数）
    QString calibPath;
    QString imagePath;
    QString cloudPointdataPath;

    // 日志输出和点云显示
    QTextEdit* logTextEdit;
    PCLViewer* viewer;

    // 点云处理线程（处理拼接）
    PointCloudProcessingThread* processingThread;
};

#endif // REGISTRATIONWIDGET_H
