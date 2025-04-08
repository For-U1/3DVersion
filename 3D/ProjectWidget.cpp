//// ProjectWidget.cpp
//#include "ProjectWidget.h"
//#include <QFileDialog>
//#include <QMessageBox>
//#include <QDateTime>
//
//ProjectWidget::ProjectWidget(QWidget* parent)
//    : QWidget(parent),
//    projectionThread(nullptr),
//    threadRunning(false)
//{
//    // Initialize UI Components
//    leftPathLabel = new QLabel("Left Image Save Path:",this);
//    leftPathLineEdit = new QLineEdit(this);
//    browseLeftPathButton = new QPushButton("Browse", this);
//
//    rightPathLabel = new QLabel("Right Image Save Path:", this);
//    rightPathLineEdit = new QLineEdit(this);
//    browseRightPathButton = new QPushButton("Browse", this);
//
//    startProjectionButton = new QPushButton("Start Projection", this);
//    stopProjectionButton = new QPushButton("Stop Projection", this);
//    stopProjectionButton->setEnabled(false); // ��ʼ����
//
//    logTextEdit = new QTextEdit(this);
//    logTextEdit->setReadOnly(true);
//
//    // Layouts
//    QVBoxLayout* mainLayout = new QVBoxLayout(this);
//
//    // ��·������
//    QHBoxLayout* leftLayout = new QHBoxLayout();
//    leftLayout->addWidget(leftPathLabel);
//    leftLayout->addWidget(leftPathLineEdit);
//    leftLayout->addWidget(browseLeftPathButton);
//
//    // ��·������
//    QHBoxLayout* rightLayout = new QHBoxLayout();
//    rightLayout->addWidget(rightPathLabel);
//    rightLayout->addWidget(rightPathLineEdit);
//    rightLayout->addWidget(browseRightPathButton);
//
//    // ��ť����
//    QHBoxLayout* buttonLayout = new QHBoxLayout();
//    buttonLayout->addWidget(startProjectionButton);
//    buttonLayout->addWidget(stopProjectionButton);
//
//    // ��ӵ�������
//    mainLayout->addLayout(leftLayout);
//    mainLayout->addLayout(rightLayout);
//    mainLayout->addLayout(buttonLayout);
//    mainLayout->addWidget(logTextEdit);
//
//    setLayout(mainLayout);
//
//    // Connect Signals and Slots
//    connect(browseLeftPathButton, &QPushButton::clicked, this, &ProjectWidget::browseLeftPath);
//    connect(browseRightPathButton, &QPushButton::clicked, this, &ProjectWidget::browseRightPath);
//    connect(startProjectionButton, &QPushButton::clicked, this, &ProjectWidget::startProjection);
//    connect(stopProjectionButton, &QPushButton::clicked, this, &ProjectWidget::stopProjection);
//}
//
//ProjectWidget::~ProjectWidget()
//{
//    if (threadRunning)
//    {
//        projectionThread->stop();
//        if (controlThread.joinable())
//            controlThread.join();
//    }
//}
//
//void ProjectWidget::browseLeftPath()
//{
//    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Left Image Save Path"), QString(),
//        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
//    if (!dir.isEmpty()) 
//    {
//        leftPathLineEdit->setText(dir);
//        appendLog(QString("Left image save path set to: %1").arg(dir));
//    }
//}
//
//void ProjectWidget::browseRightPath()
//{
//    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Right Image Save Path"), QString(),
//        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
//    if (!dir.isEmpty()) 
//    {
//        rightPathLineEdit->setText(dir);
//        appendLog(QString("Right image save path set to: %1").arg(dir));
//    }
//}
//
//void ProjectWidget::startProjection()
//{
//    QString leftPath = leftPathLineEdit->text();
//    QString rightPath = rightPathLineEdit->text();
//
//    if (leftPath.isEmpty() || rightPath.isEmpty()) 
//    {
//        QMessageBox::warning(this, tr("Path Not Selected"), tr("Please select save paths for both the left and right images."));
//        return;
//    }
//
//    // ת��Ϊ std::string
//    std::string leftSavePath = leftPath.toStdString();
//    std::string rightSavePath = rightPath.toStdString();
//
//    // ���� ProjectionControlThread ʵ��
//    projectionThread = std::make_unique<ProjectionControlThread>(leftSavePath, rightSavePath);
//
//    // ������־�ź�
//    connect(projectionThread.get(), &ProjectionControlThread::logMessage, this, &ProjectWidget::appendLog);
//
//    // ���������߳�
//    threadRunning = true;
//    controlThread = std::thread([this]() 
//        {
//        projectionThread->run();
//        });
//
//    // ���� UI ״̬
//    startProjectionButton->setEnabled(false);
//    stopProjectionButton->setEnabled(true);
//    appendLog("Projector and cameras started.");
//}
//
//void ProjectWidget::stopProjection()
//{
//    if (projectionThread && threadRunning)
//    {
//        projectionThread->stop();
//
//        if (controlThread.joinable())
//            controlThread.join();
//
//        threadRunning = false;
//
//        // ���� UI ״̬
//        startProjectionButton->setEnabled(true);
//        stopProjectionButton->setEnabled(false);
//        appendLog("Projector and cameras stopped.");
//    }
//}
//
//void ProjectWidget::appendLog(const QString& message)
//{
//    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
//    logTextEdit->append(QString("[%1] %2").arg(timestamp, message));
//}
//
