// #include "PointCloudWidget.h"
// #include <QDebug>

// PointCloudWidget::PointCloudWidget(QWidget *parent)
//     : QWidget(parent), viewer_(new PCLViewer(800, this)), pointCloudThread_(nullptr)
// {
//     QVBoxLayout* layout = new QVBoxLayout(this);

//     // 添加 PCLViewer 到布局
//     layout->addWidget(viewer_);

//     // 创建按钮
//     startButton_ = new QPushButton("Start Loading Point Cloud", this);
//     stopButton_ = new QPushButton("Stop Loading Point Cloud", this);

//     layout->addWidget(startButton_);
//     layout->addWidget(stopButton_);

//     // 连接按钮信号和槽
//     connect(startButton_, &QPushButton::clicked, this, &PointCloudWidget::startLoadingPointCloud);
//     connect(stopButton_, &QPushButton::clicked, this, &PointCloudWidget::stopLoadingPointCloud);

//     setLayout(layout);
// }

// PointCloudWidget::~PointCloudWidget()
// {
//     stopLoadingPointCloud(); // 确保线程停止
//     delete viewer_;
// }

// void PointCloudWidget::setViewerSize(int width, int height)
// {
//     if (viewer_)
//     {
//         viewer_->setFixedSize(400, 400); // 设置固定大小
//         // 或者使用下面的代码来调整大小
//         // viewer_->resize(width, height);
//     }
// }

// void PointCloudWidget::startLoadingPointCloud()
// {
//     if (!pointCloudThread_)
//     {
//         pointCloudThread_ = new PointCloudThread(viewer_);
//         connect(pointCloudThread_, &PointCloudThread::finished, pointCloudThread_, &QObject::deleteLater);
//         pointCloudThread_->start(); // 启动线程
//         qDebug() << "Point cloud loading started.";
//     }
// }

// void PointCloudWidget::stopLoadingPointCloud()
// {
//     if (pointCloudThread_)
//     {
//         pointCloudThread_->stop(); // 停止线程
//         pointCloudThread_ = nullptr; // 清理线程指针
//         qDebug() << "Point cloud loading stopped.";
//     }
// }

#include "PointCloudWidget.h"
#include <QVBoxLayout>
#include <QDebug>

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QWidget(parent)
{
    // 创建 PCLViewer 实例，用于显示点云
    viewer = new PCLViewer(600, this);

    // 创建按钮，用于加载点云文件
    loadButton = new QPushButton("Load Point Cloud", this);

    // 布局
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(viewer);
    layout->addWidget(loadButton);

    // 连接按钮点击信号到槽函数
    connect(loadButton, &QPushButton::clicked, this, &PointCloudWidget::onLoadPointCloud);
}

void PointCloudWidget::onLoadPointCloud()
{
    // 使用 QFileDialog 打开文件选择窗口
    QString filePath = QFileDialog::getOpenFileName(this, "Open Point Cloud", "", "Point Cloud Files (*.pcd *.ply)");

    if (!filePath.isEmpty()) {
        qDebug() << "Selected file:" << filePath;

        // 将文件路径设置到 PCLViewer 中并加载显示点云
        viewer->setFilePath(filePath.toStdString());
        viewer->loadPointCloudFromFile();
    }
}
