#include "RegistrationWidget.h"

RegistrationWidget::RegistrationWidget(QWidget* parent)
: QWidget(parent), processingThread(nullptr),currentMethod(0)
{
    //创建路径相关控件 
    QLabel* inputLabel = new QLabel("Input PC Icpfolder:");
    inputFolderLineEdit = new QLineEdit();
    selectInputButton = new QPushButton("Browse");

    QLabel* outputLabel = new QLabel("Output Icpfolder:");
    outputFolderLineEdit = new QLineEdit();
    selectOutputButton = new QPushButton("Browse");

    //添加拼接方法选择控件
    QLabel* methodLabel = new QLabel("Stitching Method:");
    methodSelector = new QComboBox();
    methodSelector->addItem("ICP");
    methodSelector->addItem("RANSAC");

    // 拼接方法选择按钮
    advancedButton = new QPushButton("Path Settings");

    QHBoxLayout* methodLayout = new QHBoxLayout();
    methodLayout->addWidget(methodLabel);
    methodLayout->addWidget(methodSelector);
    methodLayout->addWidget(advancedButton);

    // 将输入与输出路径控件放在一行中
    QHBoxLayout* folderLayout = new QHBoxLayout();
    folderLayout->addWidget(inputLabel);
    folderLayout->addWidget(inputFolderLineEdit);
    folderLayout->addWidget(selectInputButton);
    folderLayout->addSpacing(20);
    folderLayout->addWidget(outputLabel);
    folderLayout->addWidget(outputFolderLineEdit);
    folderLayout->addWidget(selectOutputButton);

    //创建功能按钮和进度条
    importButton = new QPushButton("View PC File");
    startButton = new QPushButton("Start Registration");
    progressBar = new QProgressBar();
    progressBar->setRange(0, 100);
    progressBar->setValue(0);

    // 功能按钮和进度条水平排列
    QHBoxLayout* functionLayout = new QHBoxLayout();
    functionLayout->addWidget(importButton);
    functionLayout->addWidget(startButton);
    functionLayout->addWidget(progressBar);

    // --- 创建日志输出和点云显示控件 ---
    logTextEdit = new QTextEdit();
    logTextEdit->setReadOnly(true);

    viewer = new PCLViewer(800, this); // 创建点云显示窗口

    // 将日志输出和点云显示水平排列
    QHBoxLayout* displayLayout = new QHBoxLayout();
    displayLayout->addWidget(logTextEdit);
    displayLayout->addWidget(viewer, 1); // viewer 占用剩余空间

    //组合总布局
    QVBoxLayout* mainLayout = new QVBoxLayout();
    mainLayout->addLayout(folderLayout);
    mainLayout->addLayout(methodLayout);
    mainLayout->addLayout(functionLayout);
    mainLayout->addLayout(displayLayout);

    setLayout(mainLayout);

    //连接信号槽
    connect(methodSelector, SIGNAL(currentIndexChanged(int)), this, SLOT(onMethodChanged(int)));
    connect(selectInputButton, &QPushButton::clicked, this, &RegistrationWidget::selectInputFolder);
    connect(selectOutputButton, &QPushButton::clicked, this, &RegistrationWidget::selectOutputFolder);
    connect(importButton, &QPushButton::clicked, this, &RegistrationWidget::importPointCloudFile);
    connect(startButton, &QPushButton::clicked, this, &RegistrationWidget::startProcessing);
    connect(advancedButton, &QPushButton::clicked, this, &RegistrationWidget::onAdvancedSettings);

    // 初始化高级设置参数（可设默认路径值也可为空）
    calibPath = "E:\\Pointcloud\\David\\calib\\stereoCalib.txt";
    imagePath = "E:\\Pointcloud\\David\\Image";
    cloudPointdataPath = "E:\\Pointcloud\\David\\DataPath";
}

RegistrationWidget::~RegistrationWidget()
{
    if (processingThread) 
    {
        processingThread->stop();
        processingThread->wait();
        delete processingThread;
        processingThread = nullptr;
    }
}

// 选择输入文件夹, 此函数只能选择文件夹 ，将要拼接的点云文件放在此文件夹下
void RegistrationWidget::selectInputFolder()
{
    QString folder = QFileDialog::getExistingDirectory(this, "Select Input pc Folder");
    if (!folder.isEmpty()) {
        inputFolderLineEdit->setText(folder);
    }
}

//这个输入函数可以选择输入文件夹中的pcd ,txt , ply点云文件
//void RegistrationWidget::selectInputFolder()
//{
//    // 修改为选择点云文件而非文件夹
//    QStringList files = QFileDialog::getOpenFileNames(this, "select pc file", "", "Point Cloud Files (*.txt *.pcd *.ply)");
//    // 将选中的文件路径（用分号或其他符号分隔）显示在输入框中
//    if (!files.isEmpty()) 
//    {
//        inputFolderLineEdit->setText(files.join(";"));
//    }
//}

// 选择点云拼接完成后的保存文件夹路径
void RegistrationWidget::selectOutputFolder()
{
    QString folder = QFileDialog::getExistingDirectory(this, "Select Output Folder");
    if (!folder.isEmpty()) 
    {
        outputFolderLineEdit->setText(folder);
    }
}

// 高级设置对话框槽函数
void RegistrationWidget::onAdvancedSettings()
{
    AdvancedSettingsDialog dialog(this);
    // 将当前高级设置参数预填到对话框中（如果需要，可扩展对话框提供 set 方法）
    // 例如：dialog.setCalibPath(calibPath);
    if (dialog.exec() == QDialog::Accepted) 
    {
        // 用户点击确定后，获取高级设置参数
        calibPath = dialog.getCalibPath();
        imagePath = dialog.getImagePath();
        cloudPointdataPath = dialog.getCloudPointdataPath();
        logTextEdit->append("Advanced settings updated.");
    }
}

//可以导入多个点云文件进行显示
void RegistrationWidget::importPointCloudFile()
{
    // 允许用户选择多个文件
    QStringList fileNames = QFileDialog::getOpenFileNames(this, "Select Point Cloud Files", "", "Point Cloud Files (*.pcd *.ply *.txt)");
    if (fileNames.isEmpty()) return;

    // 显示选中的文件路径
    //inputFolderLineEdit->setText(fileNames.join(";"));
    logTextEdit->append(QString("Select one pc file").arg(fileNames.size()));

    // 清空当前显示的点云
    viewer->clearPointClouds();

    // 颜色列表（不同文件用不同颜色）
    std::vector<std::tuple<int, int, int>> colors = 
    {
        {255,255,255},//白
        {255, 0, 0},  // 红
        {0, 255, 0},  // 绿
        {0, 0, 255},  // 蓝
        {255, 255, 0}, // 黄
        {0, 255, 255}, // 青
        {255, 0, 255}  // 紫
    };

    int colorIndex = 0;  // 用于分配颜色
    for (const QString& file : fileNames)
    {
        // 加载点云
        pcl::PointCloud<ViewerPointT>::Ptr cloud = loadPointCloud(file.toStdString());
        if (!cloud) {
            logTextEdit->append(QString("load fail: %1").arg(file));
            continue;
        }

        // 转换为显示用的 ViewerPointT 类型（假设 viewer 使用的是 pcl::PointXYZRGBA）
        pcl::PointCloud<ViewerPointT>::Ptr viewerCloud(new pcl::PointCloud<ViewerPointT>);
        auto [r, g, b] = colors[colorIndex % colors.size()]; // 取不同颜色
        for (const auto& pt : cloud->points) 
        {
            ViewerPointT vpt;
            vpt.x = pt.x;
            vpt.y = pt.y;
            vpt.z = pt.z;
            vpt.r = r;
            vpt.g = g;
            vpt.b = b;
            vpt.a = 255; // 透明度
            viewerCloud->points.push_back(vpt);
        }

        // 添加到显示窗口（不合并）
        viewer->addPointCloud(viewerCloud, file.toStdString());

        // 颜色索引递增
        colorIndex++;
    }

    logTextEdit->append("The selected point cloud files have been loaded and displayed separately.");
}

//启动点云拼接（使用输入文件夹中的点云文件作为拼接数据，待拼接的点云文件放在此文件夹下）
void RegistrationWidget::startProcessing()
{
    // 如果选择的是 ICP 方法，则要求输入文件夹和输出文件夹不为空
    if (currentMethod == 0)
    {
        QString inputFolder = inputFolderLineEdit->text();
        QString outputFolder = outputFolderLineEdit->text();
        if (inputFolder.isEmpty() || outputFolder.isEmpty())
        {
            logTextEdit->append("Error: Please select both stitching input folder and output folder!");
            return;
        }
    }
    else // 如果选择的是 RANSAC（基于标志点）的拼接方法
    {
        // 这里可以检查高级设置中的路径是否配置
        if (calibPath.isEmpty() || imagePath.isEmpty() || cloudPointdataPath.isEmpty())
        {
            logTextEdit->append("Error: Please configure advanced settings (calibPath, imagePath, cloudPointdataPath)!");
            return;
        }
    }

    // 清除日志和重置进度条
    logTextEdit->clear();
    progressBar->setValue(0);

    if (processingThread)
    {
        processingThread->stop();
        processingThread->wait();
        delete processingThread;
        processingThread = nullptr;
    }

    // 根据当前选择的拼接方法创建线程时，将当前方法和高级设置参数传递进去
    Stitchingsettings set;
    set.calibPath = calibPath.toStdString();
    set.imagePath = imagePath.toStdString();
    set.cloudPointdataPath = cloudPointdataPath.toStdString();

    // 如果采用 RANSAC 方法，则可能无需输入文件夹（可以设置默认值或忽略）
    QString inputFolder = (currentMethod == 0) ? inputFolderLineEdit->text() : QString();
    QString outputFolder = (currentMethod == 0) ? outputFolderLineEdit->text() : outputFolderLineEdit->text();  // 输出文件夹一般还是需要

    processingThread = new PointCloudProcessingThread(inputFolder.toStdString(), outputFolder.toStdString(), currentMethod, set);

    connect(processingThread, &PointCloudProcessingThread::logMessage, this, &RegistrationWidget::displayLogMessage);
    connect(processingThread, &PointCloudProcessingThread::updateProgress, this, &RegistrationWidget::updateProgress);
    connect(processingThread, &PointCloudProcessingThread::newPointCloud, this, &RegistrationWidget::displayFinalPointCloud);
    connect(processingThread, &PointCloudProcessingThread::processingFinished, this, &RegistrationWidget::processingFinished);

    // 启动线程，使用子线程可以避免阻塞主线程，start()方法会直接调用run()函数
    processingThread->start();
}

// 更新进度条
void RegistrationWidget::updateProgress(int value)
{
    if (progressBar->value() != value) 
    {
        QTimer::singleShot(50, this, [this, value]() { progressBar->setValue(value); });
    }
}

// 显示日志信息
void RegistrationWidget::displayLogMessage(const QString& message)
{
    logTextEdit->append(message);
}

// 显示最终拼接的点云
void RegistrationWidget::displayFinalPointCloud(pcl::PointCloud<ViewerPointT>::Ptr cloud)
{
    viewer->setPointCloud(cloud);
}

//传递开始的拼接方法
void RegistrationWidget::onMethodChanged(int index)
{
    currentMethod = index;
}

// 处理完成
void RegistrationWidget::processingFinished()
{
    logTextEdit->append("PointCloud Registration Finished!");
    progressBar->setValue(100);
}
