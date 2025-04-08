#include "RegistrationWidget.h"

RegistrationWidget::RegistrationWidget(QWidget* parent)
: QWidget(parent), processingThread(nullptr),currentMethod(0)
{
    //����·����ؿؼ� 
    QLabel* inputLabel = new QLabel("Input PC Icpfolder:");
    inputFolderLineEdit = new QLineEdit();
    selectInputButton = new QPushButton("Browse");

    QLabel* outputLabel = new QLabel("Output Icpfolder:");
    outputFolderLineEdit = new QLineEdit();
    selectOutputButton = new QPushButton("Browse");

    //���ƴ�ӷ���ѡ��ؼ�
    QLabel* methodLabel = new QLabel("Stitching Method:");
    methodSelector = new QComboBox();
    methodSelector->addItem("ICP");
    methodSelector->addItem("RANSAC");

    // ƴ�ӷ���ѡ��ť
    advancedButton = new QPushButton("Path Settings");

    QHBoxLayout* methodLayout = new QHBoxLayout();
    methodLayout->addWidget(methodLabel);
    methodLayout->addWidget(methodSelector);
    methodLayout->addWidget(advancedButton);

    // �����������·���ؼ�����һ����
    QHBoxLayout* folderLayout = new QHBoxLayout();
    folderLayout->addWidget(inputLabel);
    folderLayout->addWidget(inputFolderLineEdit);
    folderLayout->addWidget(selectInputButton);
    folderLayout->addSpacing(20);
    folderLayout->addWidget(outputLabel);
    folderLayout->addWidget(outputFolderLineEdit);
    folderLayout->addWidget(selectOutputButton);

    //�������ܰ�ť�ͽ�����
    importButton = new QPushButton("View PC File");
    startButton = new QPushButton("Start Registration");
    progressBar = new QProgressBar();
    progressBar->setRange(0, 100);
    progressBar->setValue(0);

    // ���ܰ�ť�ͽ�����ˮƽ����
    QHBoxLayout* functionLayout = new QHBoxLayout();
    functionLayout->addWidget(importButton);
    functionLayout->addWidget(startButton);
    functionLayout->addWidget(progressBar);

    // --- ������־����͵�����ʾ�ؼ� ---
    logTextEdit = new QTextEdit();
    logTextEdit->setReadOnly(true);

    viewer = new PCLViewer(800, this); // ����������ʾ����

    // ����־����͵�����ʾˮƽ����
    QHBoxLayout* displayLayout = new QHBoxLayout();
    displayLayout->addWidget(logTextEdit);
    displayLayout->addWidget(viewer, 1); // viewer ռ��ʣ��ռ�

    //����ܲ���
    QVBoxLayout* mainLayout = new QVBoxLayout();
    mainLayout->addLayout(folderLayout);
    mainLayout->addLayout(methodLayout);
    mainLayout->addLayout(functionLayout);
    mainLayout->addLayout(displayLayout);

    setLayout(mainLayout);

    //�����źŲ�
    connect(methodSelector, SIGNAL(currentIndexChanged(int)), this, SLOT(onMethodChanged(int)));
    connect(selectInputButton, &QPushButton::clicked, this, &RegistrationWidget::selectInputFolder);
    connect(selectOutputButton, &QPushButton::clicked, this, &RegistrationWidget::selectOutputFolder);
    connect(importButton, &QPushButton::clicked, this, &RegistrationWidget::importPointCloudFile);
    connect(startButton, &QPushButton::clicked, this, &RegistrationWidget::startProcessing);
    connect(advancedButton, &QPushButton::clicked, this, &RegistrationWidget::onAdvancedSettings);

    // ��ʼ���߼����ò���������Ĭ��·��ֵҲ��Ϊ�գ�
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

// ѡ�������ļ���, �˺���ֻ��ѡ���ļ��� ����Ҫƴ�ӵĵ����ļ����ڴ��ļ�����
void RegistrationWidget::selectInputFolder()
{
    QString folder = QFileDialog::getExistingDirectory(this, "Select Input pc Folder");
    if (!folder.isEmpty()) {
        inputFolderLineEdit->setText(folder);
    }
}

//������뺯������ѡ�������ļ����е�pcd ,txt , ply�����ļ�
//void RegistrationWidget::selectInputFolder()
//{
//    // �޸�Ϊѡ������ļ������ļ���
//    QStringList files = QFileDialog::getOpenFileNames(this, "select pc file", "", "Point Cloud Files (*.txt *.pcd *.ply)");
//    // ��ѡ�е��ļ�·�����÷ֺŻ��������ŷָ�����ʾ���������
//    if (!files.isEmpty()) 
//    {
//        inputFolderLineEdit->setText(files.join(";"));
//    }
//}

// ѡ�����ƴ����ɺ�ı����ļ���·��
void RegistrationWidget::selectOutputFolder()
{
    QString folder = QFileDialog::getExistingDirectory(this, "Select Output Folder");
    if (!folder.isEmpty()) 
    {
        outputFolderLineEdit->setText(folder);
    }
}

// �߼����öԻ���ۺ���
void RegistrationWidget::onAdvancedSettings()
{
    AdvancedSettingsDialog dialog(this);
    // ����ǰ�߼����ò���Ԥ��Ի����У������Ҫ������չ�Ի����ṩ set ������
    // ���磺dialog.setCalibPath(calibPath);
    if (dialog.exec() == QDialog::Accepted) 
    {
        // �û����ȷ���󣬻�ȡ�߼����ò���
        calibPath = dialog.getCalibPath();
        imagePath = dialog.getImagePath();
        cloudPointdataPath = dialog.getCloudPointdataPath();
        logTextEdit->append("Advanced settings updated.");
    }
}

//���Ե����������ļ�������ʾ
void RegistrationWidget::importPointCloudFile()
{
    // �����û�ѡ�����ļ�
    QStringList fileNames = QFileDialog::getOpenFileNames(this, "Select Point Cloud Files", "", "Point Cloud Files (*.pcd *.ply *.txt)");
    if (fileNames.isEmpty()) return;

    // ��ʾѡ�е��ļ�·��
    //inputFolderLineEdit->setText(fileNames.join(";"));
    logTextEdit->append(QString("Select one pc file").arg(fileNames.size()));

    // ��յ�ǰ��ʾ�ĵ���
    viewer->clearPointClouds();

    // ��ɫ�б���ͬ�ļ��ò�ͬ��ɫ��
    std::vector<std::tuple<int, int, int>> colors = 
    {
        {255,255,255},//��
        {255, 0, 0},  // ��
        {0, 255, 0},  // ��
        {0, 0, 255},  // ��
        {255, 255, 0}, // ��
        {0, 255, 255}, // ��
        {255, 0, 255}  // ��
    };

    int colorIndex = 0;  // ���ڷ�����ɫ
    for (const QString& file : fileNames)
    {
        // ���ص���
        pcl::PointCloud<ViewerPointT>::Ptr cloud = loadPointCloud(file.toStdString());
        if (!cloud) {
            logTextEdit->append(QString("load fail: %1").arg(file));
            continue;
        }

        // ת��Ϊ��ʾ�õ� ViewerPointT ���ͣ����� viewer ʹ�õ��� pcl::PointXYZRGBA��
        pcl::PointCloud<ViewerPointT>::Ptr viewerCloud(new pcl::PointCloud<ViewerPointT>);
        auto [r, g, b] = colors[colorIndex % colors.size()]; // ȡ��ͬ��ɫ
        for (const auto& pt : cloud->points) 
        {
            ViewerPointT vpt;
            vpt.x = pt.x;
            vpt.y = pt.y;
            vpt.z = pt.z;
            vpt.r = r;
            vpt.g = g;
            vpt.b = b;
            vpt.a = 255; // ͸����
            viewerCloud->points.push_back(vpt);
        }

        // ��ӵ���ʾ���ڣ����ϲ���
        viewer->addPointCloud(viewerCloud, file.toStdString());

        // ��ɫ��������
        colorIndex++;
    }

    logTextEdit->append("The selected point cloud files have been loaded and displayed separately.");
}

//��������ƴ�ӣ�ʹ�������ļ����еĵ����ļ���Ϊƴ�����ݣ���ƴ�ӵĵ����ļ����ڴ��ļ����£�
void RegistrationWidget::startProcessing()
{
    // ���ѡ����� ICP ��������Ҫ�������ļ��к�����ļ��в�Ϊ��
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
    else // ���ѡ����� RANSAC�����ڱ�־�㣩��ƴ�ӷ���
    {
        // ������Լ��߼������е�·���Ƿ�����
        if (calibPath.isEmpty() || imagePath.isEmpty() || cloudPointdataPath.isEmpty())
        {
            logTextEdit->append("Error: Please configure advanced settings (calibPath, imagePath, cloudPointdataPath)!");
            return;
        }
    }

    // �����־�����ý�����
    logTextEdit->clear();
    progressBar->setValue(0);

    if (processingThread)
    {
        processingThread->stop();
        processingThread->wait();
        delete processingThread;
        processingThread = nullptr;
    }

    // ���ݵ�ǰѡ���ƴ�ӷ��������߳�ʱ������ǰ�����͸߼����ò������ݽ�ȥ
    Stitchingsettings set;
    set.calibPath = calibPath.toStdString();
    set.imagePath = imagePath.toStdString();
    set.cloudPointdataPath = cloudPointdataPath.toStdString();

    // ������� RANSAC ��������������������ļ��У���������Ĭ��ֵ����ԣ�
    QString inputFolder = (currentMethod == 0) ? inputFolderLineEdit->text() : QString();
    QString outputFolder = (currentMethod == 0) ? outputFolderLineEdit->text() : outputFolderLineEdit->text();  // ����ļ���һ�㻹����Ҫ

    processingThread = new PointCloudProcessingThread(inputFolder.toStdString(), outputFolder.toStdString(), currentMethod, set);

    connect(processingThread, &PointCloudProcessingThread::logMessage, this, &RegistrationWidget::displayLogMessage);
    connect(processingThread, &PointCloudProcessingThread::updateProgress, this, &RegistrationWidget::updateProgress);
    connect(processingThread, &PointCloudProcessingThread::newPointCloud, this, &RegistrationWidget::displayFinalPointCloud);
    connect(processingThread, &PointCloudProcessingThread::processingFinished, this, &RegistrationWidget::processingFinished);

    // �����̣߳�ʹ�����߳̿��Ա����������̣߳�start()������ֱ�ӵ���run()����
    processingThread->start();
}

// ���½�����
void RegistrationWidget::updateProgress(int value)
{
    if (progressBar->value() != value) 
    {
        QTimer::singleShot(50, this, [this, value]() { progressBar->setValue(value); });
    }
}

// ��ʾ��־��Ϣ
void RegistrationWidget::displayLogMessage(const QString& message)
{
    logTextEdit->append(message);
}

// ��ʾ����ƴ�ӵĵ���
void RegistrationWidget::displayFinalPointCloud(pcl::PointCloud<ViewerPointT>::Ptr cloud)
{
    viewer->setPointCloud(cloud);
}

//���ݿ�ʼ��ƴ�ӷ���
void RegistrationWidget::onMethodChanged(int index)
{
    currentMethod = index;
}

// �������
void RegistrationWidget::processingFinished()
{
    logTextEdit->append("PointCloud Registration Finished!");
    progressBar->setValue(100);
}
